// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <robocup_ssl_msgs/grSim_Commands.pb.h>
#include <robocup_ssl_msgs/grSim_Packet.pb.h>
#include <robocup_ssl_msgs/ssl_simulation_robot_control.pb.h>

#include <array>
#include <boost/asio.hpp>
#include <cmath>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <format>
#include <iomanip>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "crane_comm/udp_sender.hpp"
#include "crane_geometry/geometry_operations.hpp"
#include "crane_sender/robot_packet.h"
#include "crane_sender/sender_base.hpp"

namespace crane
{

// 通信設定の定数
namespace CommConfig
{
constexpr int DEFAULT_PORT = 12345;
constexpr const char * BROADCAST_ADDRESS = "192.168.20.255";
constexpr int AI_CMD_V2_SIZE = 64;
constexpr int AI_CMD_V2_ROBOT_NUM = 11;
}  // namespace CommConfig

// 送信パケット種別
enum class PacketType { IBIS, SSL, GRSIM };

class IbisSenderNode : public SenderBase
{
private:
  int debug_id;

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_subscriber;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_handle;

  PacketType packet_type_{PacketType::IBIS};

  // IBIS type: ブロードキャスト送信用ソケット
  boost::asio::io_service broadcast_io_service_;
  boost::asio::ip::udp::endpoint broadcast_endpoint_;
  boost::asio::ip::udp::socket broadcast_socket_;

  // SSL type
  std::unique_ptr<UDPSender> ssl_blue_sender_;
  std::unique_ptr<UDPSender> ssl_yellow_sender_;

  // GRSIM type
  std::unique_ptr<UDPSender> grsim_sender_;

  // SSL/GRSIM type: per-robot state for theta control and acceleration limiting
  struct PerRobotState
  {
    double prev_vx = 0.0;
    double prev_vy = 0.0;
  };
  std::array<PerRobotState, CommConfig::AI_CMD_V2_ROBOT_NUM> robot_states_;

  double theta_p_gain_{4.0};
  double chip_angle_deg_{30.0};

  int counter_{0};

public:
  explicit IbisSenderNode(const rclcpp::NodeOptions & options)
  : SenderBase("ibis_sender", options),
    broadcast_socket_(
      broadcast_io_service_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0))
  {
    declare_parameter("debug_id", -1);
    get_parameter("debug_id", debug_id);

    parameter_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
    parameter_callback_handle =
      parameter_subscriber->add_parameter_callback("debug_id", [&](const rclcpp::Parameter & p) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          debug_id = p.as_int();
        } else {
          RCLCPP_WARN(get_logger(), "Warning: debug_id must be an integer");
        }
      });

    // packet_type パラメータ: ibis / ssl / grsim
    declare_parameter("packet_type", std::string("ibis"));
    const std::string packet_type_str = get_parameter("packet_type").as_string();

    // 送信先アドレスとポートの設定
    declare_parameter("target_address", std::string(CommConfig::BROADCAST_ADDRESS));
    declare_parameter("target_port", CommConfig::DEFAULT_PORT);
    declare_parameter("theta_p_gain", theta_p_gain_);
    declare_parameter("chip_angle_deg", chip_angle_deg_);

    const std::string target_address = get_parameter("target_address").as_string();
    const int target_port = get_parameter("target_port").as_int();
    theta_p_gain_ = get_parameter("theta_p_gain").as_double();
    chip_angle_deg_ = get_parameter("chip_angle_deg").as_double();

    if (packet_type_str == "ssl") {
      packet_type_ = PacketType::SSL;
      // SSL制御ポートはgrSimの標準ポートを使用（target_addressのホストに送信）
      declare_parameter("ssl_blue_port", 10301);
      declare_parameter("ssl_yellow_port", 10302);
      const int blue_port = get_parameter("ssl_blue_port").as_int();
      const int yellow_port = get_parameter("ssl_yellow_port").as_int();
      ssl_blue_sender_ = std::make_unique<UDPSender>(target_address, blue_port);
      ssl_yellow_sender_ = std::make_unique<UDPSender>(target_address, yellow_port);
      RCLCPP_INFO(
        get_logger(), "ibis_sender_node started [packet_type=ssl] (blue: %s:%d, yellow: %s:%d)",
        target_address.c_str(), blue_port, target_address.c_str(), yellow_port);
    } else if (packet_type_str == "grsim") {
      packet_type_ = PacketType::GRSIM;
      declare_parameter("grsim_port", 20011);
      const int grsim_port = get_parameter("grsim_port").as_int();
      grsim_sender_ = std::make_unique<UDPSender>(target_address, grsim_port);
      RCLCPP_INFO(
        get_logger(), "ibis_sender_node started [packet_type=grsim] (%s:%d)",
        target_address.c_str(), grsim_port);
    } else {
      if (packet_type_str != "ibis") {
        RCLCPP_WARN(
          get_logger(), "Unknown packet_type '%s', falling back to 'ibis'",
          packet_type_str.c_str());
      }
      packet_type_ = PacketType::IBIS;

      try {
        // ブロードキャスト許可フラグを設定
        broadcast_socket_.set_option(boost::asio::socket_base::broadcast(true));
        RCLCPP_INFO(get_logger(), "✓ SO_BROADCAST flag set");

        broadcast_endpoint_ = boost::asio::ip::udp::endpoint(
          boost::asio::ip::address::from_string(target_address), target_port);

        // インターフェース情報の確認（デバッグ用）
        checkNetworkInterfaces();

        RCLCPP_INFO(get_logger(), "【Real Robot Broadcast Mode Initialized】");
        RCLCPP_INFO(get_logger(), "  Target Address: %s:%d", target_address.c_str(), target_port);
        RCLCPP_INFO(
          get_logger(), "  Resolved Endpoint: %s:%d",
          broadcast_endpoint_.address().to_string().c_str(), broadcast_endpoint_.port());
        RCLCPP_INFO(
          get_logger(), "  Local Socket: %s:%d",
          broadcast_socket_.local_endpoint().address().to_string().c_str(),
          broadcast_socket_.local_endpoint().port());
      } catch (std::exception & e) {
        RCLCPP_ERROR(get_logger(), "❌ Broadcast Socket Init Error: %s", e.what());
      }

      RCLCPP_INFO(
        get_logger(), "ibis_sender_node started [packet_type=ibis] (%s:%d)", target_address.c_str(),
        target_port);
    }
  }

private:
  void checkNetworkInterfaces() const
  {
    struct ifaddrs * interfaces = nullptr;

    RCLCPP_INFO(get_logger(), "🌐 Available Network Interfaces:");

    if (getifaddrs(&interfaces) == -1) {
      RCLCPP_ERROR(get_logger(), "❌ Failed to get network interface info");
      return;
    }

    for (struct ifaddrs * ifa = interfaces; ifa != nullptr; ifa = ifa->ifa_next) {
      if (ifa->ifa_addr == nullptr) continue;

      // IPv4アドレスのみ表示
      if (ifa->ifa_addr->sa_family == AF_INET) {
        struct sockaddr_in * addr_in = reinterpret_cast<struct sockaddr_in *>(ifa->ifa_addr);
        char ip_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(addr_in->sin_addr), ip_str, INET_ADDRSTRLEN);

        // ブロードキャストアドレス情報も取得
        char broadcast_str[INET_ADDRSTRLEN] = "N/A";
        if (ifa->ifa_flags & IFF_BROADCAST && ifa->ifa_broadaddr) {
          struct sockaddr_in * broadcast_in =
            reinterpret_cast<struct sockaddr_in *>(ifa->ifa_broadaddr);
          inet_ntop(AF_INET, &(broadcast_in->sin_addr), broadcast_str, INET_ADDRSTRLEN);
        }

        std::string log_msg = "  Interface: " + std::string(ifa->ifa_name) +
                              " IP: " + std::string(ip_str) +
                              " Broadcast: " + std::string(broadcast_str);

        // インターフェースの状態を表示
        if (ifa->ifa_flags & IFF_UP) log_msg += " [UP]";
        if (ifa->ifa_flags & IFF_RUNNING) log_msg += " [RUNNING]";
        if (ifa->ifa_flags & IFF_BROADCAST) log_msg += " [BROADCAST]";

        RCLCPP_INFO(get_logger(), "%s", log_msg.c_str());

        // 設定されたブロードキャストアドレスとの照合
        if (std::string(broadcast_str) == CommConfig::BROADCAST_ADDRESS) {
          RCLCPP_INFO(get_logger(), "    ✅ Matches configured broadcast address!");
        }
      }
    }

    freeifaddrs(interfaces);
  }

  // IBIS バイナリパケット生成
  RobotCommandV2 createRobotPacket(
    const crane_msgs::msg::RobotCommand & command, int counter,
    const std::vector<uint8_t> & available_ids)
  {
    RobotCommandV2 packet;
    const float resolved_max_velocity =
      std::max(0.0f, command.local_planner_config.final_planned_max_velocity.value);
    const float resolved_max_acceleration =
      std::max(0.0f, command.local_planner_config.final_planned_max_acceleration.value);
    const float target_velocity_r = !command.polar_velocity_target_mode.empty()
                                      ? command.polar_velocity_target_mode.front().target_velocity_r
                                      : 0.0f;
    const float target_velocity_theta =
      !command.polar_velocity_target_mode.empty()
        ? command.polar_velocity_target_mode.front().target_velocity_theta
        : 0.0f;

    packet.header = 0x00;
    packet.check_counter = counter;
    packet.vision_global_pos[0] = command.current_pose.x;
    packet.vision_global_pos[1] = command.current_pose.y;
    packet.vision_global_theta = command.current_pose.theta;
    packet.is_vision_available =
      std::count(available_ids.begin(), available_ids.end(), command.robot_id) == 1;
    packet.target_global_theta = command.target_theta;
    packet.kick_power = command.kick_power;
    packet.dribble_power = std::clamp(command.dribble_power, 0.f, 1.f);
    packet.enable_chip = command.chip_enable;
    packet.stop_emergency = command.stop_flag;

    double current_speed = std::hypot(command.current_velocity.x, command.current_velocity.y);
    double target_speed = resolved_max_velocity;
    double selected_acceleration = calculateAccelerationLimit(current_speed, target_speed);

    packet.acceleration_limit =
      resolved_max_acceleration > 0.0f
        ? std::min(static_cast<float>(selected_acceleration), resolved_max_acceleration)
        : selected_acceleration;
    packet.linear_velocity_limit = resolved_max_velocity;
    packet.angular_velocity_limit = command.omega_limit;
    packet.latency_time_ms = static_cast<uint16_t>(command.latency_ms);
    packet.elapsed_time_ms_since_last_vision = command.elapsed_time_ms_since_last_vision;

    // POSITION_TARGET_WITH_TERMINAL_VELOCITY_MODE (mode=4) は、上位レイヤが明示的に
    // POSITION_TARGET_MODE を指定した場合のみ使用する。
    // 旧受信機 (ER-Force ibis branch / 現行実機ファーム) は mode=3 のみ対応しているため、
    // crane_local_planner が設定した control_mode フィールドを尊重することで後方互換性を維持する。
    if (
      command.control_mode == crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE &&
      !command.position_target_mode.empty()) {
      const auto & pos_mode = command.position_target_mode.front();
      packet.control_mode = POSITION_TARGET_WITH_TERMINAL_VELOCITY_MODE;
      packet.mode_args.position_target.terminal_velocity_x = pos_mode.terminal_velocity_x;
      packet.mode_args.position_target.terminal_velocity_y = pos_mode.terminal_velocity_y;
      packet.target_global_pos[0] = pos_mode.target_x;
      packet.target_global_pos[1] = pos_mode.target_y;
      packet.terminal_velocity = pos_mode.speed_limit_at_target;
    } else {
      packet.control_mode = POLAR_VELOCITY_TARGET_MODE;
      packet.mode_args.polar_velocity.target_global_velocity_r = target_velocity_r;
      packet.mode_args.polar_velocity.target_global_velocity_theta = target_velocity_theta;
      packet.target_global_pos[0] = command.current_pose.x;
      packet.target_global_pos[1] = command.current_pose.y;
      packet.terminal_velocity = 0.0f;
    }

    return packet;
  }

  // SSL/GRSIM 共通：極座標速度＋theta制御 → ロボットローカル速度変換
  struct LocalVelocity
  {
    double vx;  // forward
    double vy;  // left
    double omega;
  };

  LocalVelocity convertToLocalVelocity(
    const crane_msgs::msg::RobotCommand & command, PerRobotState & state)
  {
    const double target_velocity_r =
      !command.polar_velocity_target_mode.empty()
        ? command.polar_velocity_target_mode.front().target_velocity_r
        : 0.0;
    const double target_velocity_theta =
      !command.polar_velocity_target_mode.empty()
        ? command.polar_velocity_target_mode.front().target_velocity_theta
        : 0.0;

    // Theta P制御
    const double theta_error = getAngleDiff(command.target_theta, command.current_pose.theta);
    double omega = theta_p_gain_ * (-theta_error);
    omega = std::clamp(
      omega, -static_cast<double>(command.omega_limit), static_cast<double>(command.omega_limit));

    // 遅延補正後の現在角度
    constexpr double dt = 1.0 / 30.0;
    const double current_theta = command.current_pose.theta + omega * delay_s;

    // グローバル極座標 → ロボットローカル直交座標
    const double velocity_theta = target_velocity_theta - current_theta;
    const double target_vx = target_velocity_r * std::cos(velocity_theta);
    const double target_vy = target_velocity_r * std::sin(velocity_theta);

    // 加速度制限
    const double current_speed = std::hypot(state.prev_vx, state.prev_vy);
    const double target_speed = std::hypot(target_vx, target_vy);
    const double acc_limit = calculateAccelerationLimit(current_speed, target_speed);
    const double max_delta = acc_limit * dt;

    const double delta_vx = target_vx - state.prev_vx;
    const double delta_vy = target_vy - state.prev_vy;
    const double delta_norm = std::hypot(delta_vx, delta_vy);

    double out_vx, out_vy;
    if (delta_norm > max_delta && delta_norm > 1e-9) {
      out_vx = state.prev_vx + (delta_vx / delta_norm) * max_delta;
      out_vy = state.prev_vy + (delta_vy / delta_norm) * max_delta;
    } else {
      out_vx = target_vx;
      out_vy = target_vy;
    }

    if (command.stop_flag) {
      out_vx = 0.0;
      out_vy = 0.0;
      omega = 0.0;
    }

    state.prev_vx = out_vx;
    state.prev_vy = out_vy;
    return {out_vx, out_vy, omega};
  }

  struct KickParams
  {
    double speed;
    double angle_rad;  // 0 for flat kick
  };

  KickParams computeKick(float kick_power, bool chip_enable) const
  {
    constexpr double MAX_KICK_SPEED = 8.0;
    const double kick_speed = MAX_KICK_SPEED * kick_power;
    if (chip_enable) {
      return {kick_speed * 0.5, chip_angle_deg_ * M_PI / 180.0};
    }
    return {kick_speed, 0.0};
  }

  void sendSSL(const crane_msgs::msg::RobotCommands & msg)
  {
    auto & sender = msg.is_yellow ? ssl_yellow_sender_ : ssl_blue_sender_;

    robocup_ssl::RobotControl packet;
    for (const auto & command : msg.robot_commands) {
      auto cmd = packet.add_robot_commands();
      cmd->set_id(command.robot_id);

      auto & state = robot_states_[command.robot_id];
      const auto vel = convertToLocalVelocity(command, state);

      auto move_command = new robocup_ssl::RobotMoveCommand();
      auto move_local_velocity = new robocup_ssl::MoveLocalVelocity();
      move_local_velocity->set_forward(vel.vx);
      move_local_velocity->set_left(vel.vy);
      move_local_velocity->set_angular(vel.omega);
      move_command->set_allocated_local_velocity(move_local_velocity);
      cmd->set_allocated_move_command(move_command);

      const auto kick = computeKick(command.kick_power, command.chip_enable);
      cmd->set_kick_angle(kick.angle_rad);
      cmd->set_kick_speed(kick.speed);
      cmd->set_dribbler_speed(command.dribble_power * 1000.0);
    }

    std::string output;
    packet.SerializeToString(&output);
    sender->send(output);
  }

  void sendGrSim(const crane_msgs::msg::RobotCommands & msg)
  {
    robocup_ssl::grSim_Packet packet;
    auto * commands = packet.mutable_commands();
    commands->set_isteamyellow(msg.is_yellow);
    commands->set_timestamp(0.0);

    for (const auto & command : msg.robot_commands) {
      auto * robot_cmd = commands->add_robot_commands();
      robot_cmd->set_id(command.robot_id);

      auto & state = robot_states_[command.robot_id];
      const auto vel = convertToLocalVelocity(command, state);

      robot_cmd->set_veltangent(static_cast<float>(vel.vx));
      robot_cmd->set_velnormal(static_cast<float>(vel.vy));
      robot_cmd->set_velangular(static_cast<float>(vel.omega));
      robot_cmd->set_spinner(command.dribble_power > 0.001f);
      robot_cmd->set_wheelsspeed(false);

      const auto kick = computeKick(command.kick_power, command.chip_enable);
      robot_cmd->set_kickspeedx(static_cast<float>(kick.speed * std::cos(kick.angle_rad)));
      robot_cmd->set_kickspeedz(static_cast<float>(kick.speed * std::sin(kick.angle_rad)));
    }

    std::string output;
    packet.SerializeToString(&output);
    grsim_sender_->send(output);
  }

  void sendIbis(const crane_msgs::msg::RobotCommands & msg)
  {
    if (++counter_ > 200) {
      counter_ = 0;
    }

    const auto available_ids = world_model->ours().robotsWhere().available().getIds();

    std::array<std::pair<uint8_t, RobotCommandSerializedV2>, CommConfig::AI_CMD_V2_ROBOT_NUM>
      robot_packets{};
    for (int i = 0; i < CommConfig::AI_CMD_V2_ROBOT_NUM; i++) {
      robot_packets[i] = {static_cast<uint8_t>(i), RobotCommandSerializedV2{}};
    }

    for (const auto & command : msg.robot_commands) {
      if (command.robot_id < CommConfig::AI_CMD_V2_ROBOT_NUM) {
        RobotCommandV2 packet = createRobotPacket(command, counter_, available_ids);
        RobotCommandSerializedV2 serialized_packet;
        RobotCommandSerializedV2_serialize(&serialized_packet, &packet);
        robot_packets[command.robot_id] = {command.robot_id, serialized_packet};
      }
    }

    // パケット組み立て
    char broadcast_buf[(CommConfig::AI_CMD_V2_SIZE + 1) * CommConfig::AI_CMD_V2_ROBOT_NUM] = {};
    for (size_t i = 0; i < CommConfig::AI_CMD_V2_ROBOT_NUM; i++) {
      int offset = static_cast<int>(i) * (CommConfig::AI_CMD_V2_SIZE + 1);
      broadcast_buf[offset] = static_cast<char>(i);
      memcpy(&broadcast_buf[offset + 1], robot_packets[i].second.data, CommConfig::AI_CMD_V2_SIZE);
    }

    // パケット送信
    try {
      broadcast_socket_.send_to(boost::asio::buffer(broadcast_buf), broadcast_endpoint_);
    } catch (boost::system::system_error & e) {
      RCLCPP_ERROR(get_logger(), "❌ Packet Send Error (boost): %s", e.what());
      RCLCPP_ERROR(get_logger(), "  Error Code: %d", e.code().value());
      RCLCPP_ERROR(get_logger(), "  Error Message: %s", e.code().message().c_str());
    } catch (std::exception & e) {
      RCLCPP_ERROR(get_logger(), "❌ Packet Send Exception: %s", e.what());
    }
  }

public:
  void sendCommands(const crane_msgs::msg::RobotCommands & msg) override
  {
    switch (packet_type_) {
      case PacketType::SSL:
        sendSSL(msg);
        break;
      case PacketType::GRSIM:
        sendGrSim(msg);
        break;
      case PacketType::IBIS:
      default:
        sendIbis(msg);
        break;
    }
  }
};
}  // namespace crane

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<crane::IbisSenderNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
