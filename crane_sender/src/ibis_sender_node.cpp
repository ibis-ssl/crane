// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>

#include <array>
#include <boost/asio.hpp>
#include <chrono>
#include <cmath>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/position_control_config.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_utils/parameter.hpp>
#include <format>
#include <iomanip>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

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
// 位置制御設定パケットのフォーマット版（Orion_CM4 cm4/bridge/config_packet.h の byte 4）
constexpr uint8_t POSITION_CONTROL_CONFIG_VERSION = 2;
}  // namespace CommConfig

class IbisSenderNode : public SenderBase
{
private:
  int debug_id;

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_subscriber;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_handle;

  // ブロードキャスト送信用ソケット
  boost::asio::io_service broadcast_io_service_;
  boost::asio::ip::udp::endpoint broadcast_endpoint_;
  boost::asio::ip::udp::socket broadcast_socket_;

  // 位置制御設定パケットの送信先（指令と同じアドレスでポートだけ分ける）
  boost::asio::ip::udp::endpoint position_control_config_endpoint_;
  std::chrono::steady_clock::time_point last_position_control_config_send_;
  // CM4 の位置制御 PID のゲイン。ここにある値は既定値で、実際に送るのは
  // 1 秒ごとに get_parameter() で読み直した現在値である（sendPositionControlConfig）。
  // 既定は CM4 側の PositionControllerConfig と一致させること。
  double position_control_kp_ = 2.0;
  double position_control_deceleration_ = 3.0;
  double position_control_tolerance_ = 0.01;
  double position_control_ki_ = 0.0;
  double position_control_kd_ = 0.0;

  // 送ったゲインを bag に残すための publisher（sendPositionControlConfig で使う）
  rclcpp::Publisher<crane_msgs::msg::PositionControlConfig>::SharedPtr position_control_config_pub_;

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

    // 送信先アドレスとポートの設定
    const std::string target_address =
      crane::get_or_declare_parameter(this, "target_address", CommConfig::BROADCAST_ADDRESS);
    const int target_port =
      crane::get_or_declare_parameter(this, "target_port", CommConfig::DEFAULT_PORT);
    // position_control.* は CM4 側の位置制御ゲインである。
    //
    // 位置制御ループは CM4 側で閉じているので、crane は 1 秒ごとに設定パケット
    // （Orion_CM4 cm4/bridge/config_packet.h）でこの値を CM4 へ送るだけである。
    // CM4 側は PID で、ki / kd の既定 0 は P 制御（従来の挙動）を意味する。
    //
    // 実装の正本は CM4 側の position_controller で、crane は遠隔から設定する側である。
    // 値は 1 秒ごとに get_parameter() で読み直すので、ros2 param set で変えれば
    // ロボットを再起動せずに反映される。
    // 詳細: framework/docs/robot-side-position-control.md
    crane::get_or_declare_parameter(this, "position_control.kp", position_control_kp_);
    crane::get_or_declare_parameter(
      this, "position_control.deceleration", position_control_deceleration_);
    crane::get_or_declare_parameter(
      this, "position_control.tolerance", position_control_tolerance_);
    crane::get_or_declare_parameter(this, "position_control.ki", position_control_ki_);
    crane::get_or_declare_parameter(this, "position_control.kd", position_control_kd_);

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

    // 1 Hz 以下の設定トピックなので、後から繋いだ購読者にも現在値が届くよう transient local。
    position_control_config_pub_ = create_publisher<crane_msgs::msg::PositionControlConfig>(
      "/position_control_config", rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(
      get_logger(), "ibis_sender_node started (%s:%d)", target_address.c_str(), target_port);
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
    packet.latency_time_ms = static_cast<uint8_t>(command.latency_ms);
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
   * @brief 位置制御ゲインを CM4 へ送る（Orion_CM4 cm4/bridge/config_packet.h、28 バイト固定）
   *
   * 位置制御ループは CM4 側で閉じているので、ゲインを変えるにはロボットへ届ける必要がある。
   * 指令パケットに相乗りさせないのは、64 バイトのレイアウトが crane / G474 / framework /
   * CM4 の 4 者一致を不変条件にしているためである。別ポートなら他の 3 者は変わらない。
   *
   * 1 秒ごとに get_parameter() で読み直して送る。ros2 param set で変えた値がそのまま乗り、
   * ロボットの再起動は要らない。同じ値の再送は無害で（CM4 は値が変わったときだけログを
   * 出す）、CM4 が再起動しても次の送信で追いつく。範囲外の値は CM4 側でクランプされず
   * データグラムごと破棄され、拒否理由が CM4 のログに出る。
   *
   * 【フォーマット v2（28 バイト）について】
   * CM4 側が PID になったので ki / kd を追加した v2 を送る。**後方互換は無い。**
   * CM4 は旧 v1（20 バイト）を WrongSize として拒否するので、crane と CM4 は
   * 同時に配ること。片方だけ古い機体は停止せず既定ゲイン（kp = 2.0）のまま走る。
   * 中途半端に互換を残すと「kp だけ効いて ki/kd が効いていない機体」が黙って混ざり、
   * 現地では「なんとなく追従が悪い」以外の症状が出ない。
   * リリース順序は docs/cm4_position_control.md に書いてある。
   */
  void sendPositionControlConfig()
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_position_control_config_send_ < std::chrono::seconds(1)) {
      return;
    }
    last_position_control_config_send_ = now;

    // 送る値と bag に残す値を二重に書かないよう、メッセージを先に組んでから
    // そこからパケットの並びを作る。片方だけ直して記録がずれるのを防ぐため。
    crane_msgs::msg::PositionControlConfig config_msg;
    config_msg.header.stamp = get_clock()->now();
    config_msg.packet_version = CommConfig::POSITION_CONTROL_CONFIG_VERSION;
    config_msg.target_robot_id = crane_msgs::msg::PositionControlConfig::TARGET_ALL_ROBOTS;
    config_msg.kp = static_cast<float>(get_parameter("position_control.kp").as_double());
    config_msg.deceleration =
      static_cast<float>(get_parameter("position_control.deceleration").as_double());
    config_msg.tolerance =
      static_cast<float>(get_parameter("position_control.tolerance").as_double());
    config_msg.ki = static_cast<float>(get_parameter("position_control.ki").as_double());
    config_msg.kd = static_cast<float>(get_parameter("position_control.kd").as_double());

    // 並び順は Orion_CM4 cm4/bridge/config_packet.h の byte 8..27 と対応する。
    const float values[5] = {
      config_msg.kp, config_msg.deceleration, config_msg.tolerance, config_msg.ki, config_msg.kd};

    uint8_t buf[28] = {};
    buf[0] = 'O';
    buf[1] = 'C';
    buf[2] = '4';
    buf[3] = 'C';
    buf[4] = config_msg.packet_version;
    buf[5] = config_msg.target_robot_id;
    memcpy(&buf[8], values, sizeof(values));

    config_msg.sent = true;
    try {
      broadcast_socket_.send_to(boost::asio::buffer(buf), position_control_config_endpoint_);
    } catch (std::exception & e) {
      config_msg.sent = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "位置制御設定パケットの送信に失敗: %s", e.what());
    }

    // 送信のたびに残す。理由と読み方は docs/cm4_position_control.md に書いてある。
    position_control_config_pub_->publish(config_msg);
  }

public:
  void sendCommands(const crane_msgs::msg::RobotCommands & msg) override { sendIbis(msg); }
};
}  // namespace crane

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<crane::IbisSenderNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
