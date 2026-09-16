// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <robocup_ssl_msgs/ssl_simulation_robot_control.pb.h>

#include <array>
#include <boost/asio.hpp>
#include <chrono>
#include <cmath>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_utils/parameter.hpp>
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
#include "crane_sender/sim_position_controller.hpp"

namespace crane
{

// 通信設定の定数
namespace CommConfig
{
constexpr int DEFAULT_PORT = 12345;
constexpr const char * BROADCAST_ADDRESS = "192.168.20.255";
constexpr int AI_CMD_V2_SIZE = 64;
constexpr int AI_CMD_V2_ROBOT_NUM = 11;
constexpr int MAX_ROBOT_NUM = 20;
}  // namespace CommConfig

// 送信パケット種別
enum class PacketType { IBIS, SSL };

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

  // IBIS type: 位置制御設定パケットの送信先（指令と同じアドレスでポートだけ分ける）
  boost::asio::ip::udp::endpoint position_control_config_endpoint_;
  std::chrono::steady_clock::time_point last_position_control_config_send_;
  // CM4 の position_tolerance。715 バイトの指令パケットには載らないので
  // SimPositionControllerConfig にもフィールドが無い。設定パケットで送る。
  double position_control_tolerance_ = 0.01;

  // SSL type
  std::unique_ptr<UDPSender> ssl_blue_sender_;
  std::unique_ptr<UDPSender> ssl_yellow_sender_;

  // SSL type: per-robot state for theta control and acceleration limiting
  struct PerRobotState
  {
    double prev_vx = 0.0;
    double prev_vy = 0.0;
    uint8_t previous_control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
    std::chrono::steady_clock::time_point previous_update{};
    bool initialized = false;
  };
  std::array<PerRobotState, CommConfig::MAX_ROBOT_NUM> robot_states_;

  double theta_p_gain_{4.0};
  double chip_angle_deg_{30.0};
  SimPositionControllerConfig position_controller_config_;

  int counter_{0};

public:
  explicit IbisSenderNode(const rclcpp::NodeOptions & options)
  : SenderBase("ibis_sender", options),
    broadcast_socket_(
      broadcast_io_service_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0))
  {
    crane::get_or_declare_parameter(this, "debug_id", debug_id);

    parameter_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
    parameter_callback_handle =
      parameter_subscriber->add_parameter_callback("debug_id", [&](const rclcpp::Parameter & p) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          debug_id = p.as_int();
        } else {
          RCLCPP_WARN(get_logger(), "Warning: debug_id must be an integer");
        }
      });

    // packet_type パラメータ: ibis / ssl
    const std::string packet_type_str =
      crane::get_or_declare_parameter(this, "packet_type", "ibis");

    // 送信先アドレスとポートの設定
    const std::string target_address =
      crane::get_or_declare_parameter(this, "target_address", CommConfig::BROADCAST_ADDRESS);
    const int target_port =
      crane::get_or_declare_parameter(this, "target_port", CommConfig::DEFAULT_PORT);
    crane::get_or_declare_parameter(this, "theta_p_gain", theta_p_gain_);
    crane::get_or_declare_parameter(this, "chip_angle_deg", chip_angle_deg_);
    // position_control.* の効き方は packet_type で変わる。
    //
    // packet_type=ssl: crane 側の calculateSimGlobalVelocity がこの値で位置制御する。
    // packet_type=ibis: 位置制御は CM4 側で閉じる。crane は 1 秒ごとに設定パケット
    //   （Orion_CM4 cm4/bridge/config_packet.h）でこの値を CM4 へ送り、CM4 の
    //   position_controller のゲインを稼働中に上書きする。
    //
    // 実装の正本は CM4 側の position_controller で、crane は遠隔から設定する側である。
    // 値は 1 秒ごとに get_parameter() で読み直すので、ros2 param set で変えれば
    // ロボットを再起動せずに反映される。
    // 詳細: framework/docs/robot-side-position-control.md
    crane::get_or_declare_parameter(
      this, "position_control.kp", position_controller_config_.position_gain);
    crane::get_or_declare_parameter(
      this, "position_control.deceleration", position_controller_config_.deceleration);
    crane::get_or_declare_parameter(
      this, "position_control.tolerance", position_control_tolerance_);

    if (packet_type_str == "ssl") {
      packet_type_ = PacketType::SSL;
      // SSL制御ポートはgrSimの標準ポートを使用（target_addressのホストに送信）
      const int blue_port = crane::get_or_declare_parameter(this, "ssl_blue_port", 10301);
      const int yellow_port = crane::get_or_declare_parameter(this, "ssl_yellow_port", 10302);
      ssl_blue_sender_ = std::make_unique<UDPSender>(target_address, blue_port);
      ssl_yellow_sender_ = std::make_unique<UDPSender>(target_address, yellow_port);
      RCLCPP_INFO(
        get_logger(), "ibis_sender_node started [packet_type=ssl] (blue: %s:%d, yellow: %s:%d)",
        target_address.c_str(), blue_port, target_address.c_str(), yellow_port);
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

        // 位置制御設定パケットは指令と同じ broadcast アドレスでポートだけ分ける。
        // CM4 側の既定は 12350（ai_cmd_v2.out / cm4_sim.out の --config-port）。
        const int position_control_config_port =
          crane::get_or_declare_parameter(this, "position_control.config_port", 12350);
        position_control_config_endpoint_ = boost::asio::ip::udp::endpoint(
          boost::asio::ip::address::from_string(target_address), position_control_config_port);

        // インターフェース情報の確認（デバッグ用）
        checkNetworkInterfaces();

        RCLCPP_INFO(get_logger(), "【Real Robot Broadcast Mode Initialized】");
        RCLCPP_INFO(get_logger(), "  Target Address: %s:%d", target_address.c_str(), target_port);
        RCLCPP_INFO(
          get_logger(), "  Position Control Config: %s:%d", target_address.c_str(),
          position_control_config_port);
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

    // ROS 側とワイヤ側で control_mode の番号が異なる点に注意（意図的な対応付け）:
    //   ROS  crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE       = 1
    //   ワイヤ POSITION_TARGET_WITH_TERMINAL_VELOCITY_MODE (robot_packet.h) = 4
    //   ROS/ワイヤ とも POLAR_VELOCITY_TARGET_MODE = 3（こちらは偶然一致している）
    //
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
      if (command.control_mode == crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE) {
        // 上位レイヤは位置指令を要求しているのに position_target_mode が空。
        // このまま mode 3 に落ちると polar_velocity_target_mode も空なので速度 0 になり、
        // 「ロボットが無言で動かない」という最も切り分けにくい形で現れる。必ず可聴にする。
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "robot_id=%d: control_mode=POSITION_TARGET_MODE but position_target_mode is empty; "
          "falling back to POLAR_VELOCITY_TARGET_MODE (wire mode 3)",
          static_cast<int>(command.robot_id));
      }
      packet.control_mode = POLAR_VELOCITY_TARGET_MODE;
      packet.mode_args.polar_velocity.target_global_velocity_r = target_velocity_r;
      packet.mode_args.polar_velocity.target_global_velocity_theta = target_velocity_theta;
      packet.target_global_pos[0] = command.current_pose.x;
      packet.target_global_pos[1] = command.current_pose.y;
      packet.terminal_velocity = 0.0f;
    }

    return packet;
  }

  // SSL 用：極座標速度＋theta制御 → ロボットローカル速度変換
  struct LocalVelocity
  {
    double vx;  // forward
    double vy;  // left
    double omega;
  };

  // 呼び出し元は sendSSL() のみ。sendIbis() からは呼ばれない。
  // packet_type=ibis の経路に位置制御ループを持ち込まないこと（CM4 側が唯一の位置ループ）。
  LocalVelocity convertToLocalVelocity(
    const crane_msgs::msg::RobotCommand & command, PerRobotState & state)
  {
    // Theta P制御
    const double theta_error = getAngleDiff(command.target_theta, command.current_pose.theta);
    double omega = theta_p_gain_ * (-theta_error);
    omega = std::clamp(
      omega, -static_cast<double>(command.omega_limit), static_cast<double>(command.omega_limit));

    // 遅延補正後の現在角度
    const double current_theta = command.current_pose.theta + omega * delay_s;

    double global_vx = 0.0;
    double global_vy = 0.0;
    if (command.control_mode == crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE) {
      const auto velocity = calculateSimGlobalVelocity(command, position_controller_config_);
      global_vx = velocity.x();
      global_vy = velocity.y();
    } else if (!command.polar_velocity_target_mode.empty()) {
      const auto & polar = command.polar_velocity_target_mode.front();
      global_vx = polar.target_velocity_r * std::cos(polar.target_velocity_theta);
      global_vy = polar.target_velocity_r * std::sin(polar.target_velocity_theta);
    }

    // グローバル直交座標 → ロボットローカル直交座標
    const auto target_local_vel = rotate(Vector2(global_vx, global_vy), -current_theta);
    const double target_vx = target_local_vel.x();
    const double target_vy = target_local_vel.y();

    const auto update_time = std::chrono::steady_clock::now();
    double dt = 1.0 / 60.0;
    if (state.initialized) {
      dt = std::clamp(
        std::chrono::duration<double>(update_time - state.previous_update).count(), 1e-3, 0.1);
    }
    if (!state.initialized || state.previous_control_mode != command.control_mode) {
      const double offset = command.field_coordinate_theta_offset;
      const auto rotated_current_velocity =
        rotateFieldVector(Vector2(command.current_velocity.x, command.current_velocity.y), offset);
      const auto current_local_vel = rotate(rotated_current_velocity, -current_theta);
      state.prev_vx = current_local_vel.x();
      state.prev_vy = current_local_vel.y();
    }
    state.previous_update = update_time;
    state.previous_control_mode = command.control_mode;
    state.initialized = true;

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
      return {kick_speed * 0.5, deg2rad(chip_angle_deg_)};
    }
    return {kick_speed, 0.0};
  }

  void sendSSL(const crane_msgs::msg::RobotCommands & msg)
  {
    auto & sender = msg.is_yellow ? ssl_yellow_sender_ : ssl_blue_sender_;

    robocup_ssl::RobotControl packet;
    for (const auto & command : msg.robot_commands) {
      if (command.robot_id >= robot_states_.size()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "robot_id=%d is out of sender state range",
          static_cast<int>(command.robot_id));
        continue;
      }
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
      cmd->set_kick_angle(rad2deg(kick.angle_rad));
      cmd->set_kick_speed(kick.speed);
      cmd->set_dribbler_speed(command.dribble_power * 1000.0);
    }

    std::string output;
    packet.SerializeToString(&output);
    sender->send(output);
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

    sendPositionControlConfig();
  }

  /**
   * @brief 位置制御ゲインを CM4 へ送る（Orion_CM4 cm4/bridge/config_packet.h、20 バイト固定）
   *
   * 位置制御ループは CM4 側で閉じているので、ゲインを変えるにはロボットへ届ける必要がある。
   * 指令パケットに相乗りさせないのは、64 バイトのレイアウトが crane / G474 / framework /
   * CM4 の 4 者一致を不変条件にしているためである。別ポートなら他の 3 者は変わらない。
   *
   * 1 秒ごとに get_parameter() で読み直して送る。ros2 param set で変えた値がそのまま乗り、
   * ロボットの再起動は要らない。同じ値の再送は無害で（CM4 は値が変わったときだけログを
   * 出す）、CM4 が再起動しても次の送信で追いつく。範囲外の値は CM4 側でクランプされず
   * データグラムごと破棄され、拒否理由が CM4 のログに出る。
   */
  void sendPositionControlConfig()
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_position_control_config_send_ < std::chrono::seconds(1)) {
      return;
    }
    last_position_control_config_send_ = now;

    const float values[3] = {
      static_cast<float>(get_parameter("position_control.kp").as_double()),
      static_cast<float>(get_parameter("position_control.deceleration").as_double()),
      static_cast<float>(get_parameter("position_control.tolerance").as_double())};

    uint8_t buf[20] = {};
    buf[0] = 'O';
    buf[1] = 'C';
    buf[2] = '4';
    buf[3] = 'C';
    buf[4] = 1;     // version
    buf[5] = 0xFF;  // 全機宛
    memcpy(&buf[8], values, sizeof(values));

    try {
      broadcast_socket_.send_to(boost::asio::buffer(buf), position_control_config_endpoint_);
    } catch (std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "位置制御設定パケットの送信に失敗: %s", e.what());
    }
  }

public:
  void sendCommands(const crane_msgs::msg::RobotCommands & msg) override
  {
    switch (packet_type_) {
      case PacketType::SSL:
        sendSSL(msg);
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
