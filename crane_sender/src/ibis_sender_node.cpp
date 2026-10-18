// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>

#include <algorithm>
#include <array>
#include <boost/asio.hpp>
#include <chrono>
#include <cmath>
#include <crane_msg_wrappers/command_wrapper_base.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/position_control_config.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_utils/parameter.hpp>
#include <cstring>
#include <format>
#include <memory>
#include <ranges>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "crane_sender/latency_time.hpp"
#include "crane_sender/robot_packet.h"

namespace crane
{

namespace CommConfig
{
constexpr int DEFAULT_PORT = 12345;
constexpr int AI_CMD_V2_SIZE = 64;
constexpr int AI_CMD_V2_ROBOT_NUM = 11;
// 機体ごとのデータグラム: robot_id 1B + コマンド 64B（Orion_CM4 forward_ai_cmd_v2.cpp）
constexpr int AI_CMD_V2_PACKET_SIZE = AI_CMD_V2_SIZE + 1;
// 位置制御設定パケットのフォーマット版（Orion_CM4 cm4/bridge/config_packet.h の byte 4）
constexpr uint8_t POSITION_CONTROL_CONFIG_VERSION = 2;
}  // namespace CommConfig

class IbisSenderNode : public rclcpp::Node
{
private:
  using VelocityCommandsMsg = crane_msgs::msg::RobotCommands;

  std::shared_ptr<WorldModelWrapper> world_model;

  // ノードの TimeSource には繋がっていない（use_sim_time を見ない）。crane は sender で
  // use_sim_time を使わないので get_clock() と同値だが、切り替えるなら挙動変更として扱う
  rclcpp::Clock clock;

  bool no_movement{false};

  double current_latency_ms{0.0};

  double kick_power_limit_straight{1.0};

  double kick_power_limit_chip{1.0};

  double robot_acceleration_acceleration_{2.5};
  double robot_acceleration_deceleration_high_{3.0};
  double robot_acceleration_deceleration_low_{2.0};
  double robot_acceleration_velocity_threshold_{1.5};

  VelocityCommandsMsg previous_commands;

  // 実際に送った指令（前処理済み + 送信側の planning_factors）。/robot_commands には
  // elapsed_time_ms_since_last_vision や is_vision_available が載らず、bag だけでは
  // 機体停止が crane 側の指令か機体側の安全停止か切り分けられないため残す
  rclcpp::Publisher<VelocityCommandsMsg>::SharedPtr sent_commands_pub_;

  rclcpp::Subscription<VelocityCommandsMsg>::SharedPtr sub_commands;

  boost::asio::io_service udp_io_service_;
  boost::asio::ip::udp::socket udp_socket_;

  // 位置制御設定パケットは指令と同じアドレスでポートだけ分ける。
  std::array<boost::asio::ip::udp::endpoint, CommConfig::AI_CMD_V2_ROBOT_NUM> command_endpoints_;
  std::array<boost::asio::ip::udp::endpoint, CommConfig::AI_CMD_V2_ROBOT_NUM> config_endpoints_;
  // 直近の sendCommands で指令を送った機体。設定パケットはこの機体にだけ送る
  std::vector<uint8_t> last_sent_robot_ids_;
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
  : Node("ibis_sender", options),
    clock(RCL_ROS_TIME),
    udp_socket_(udp_io_service_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0))
  {
    crane::get_or_declare_parameter(this, "no_movement", no_movement);
    crane::get_or_declare_parameter(this, "kick_power_limit_straight", kick_power_limit_straight);
    crane::get_or_declare_parameter(this, "kick_power_limit_chip", kick_power_limit_chip);
    crane::get_or_declare_parameter(this, "latency_ms", current_latency_ms);
    crane::get_or_declare_parameter(
      this, "robot_acceleration.acceleration", robot_acceleration_acceleration_);
    crane::get_or_declare_parameter(
      this, "robot_acceleration.deceleration_high", robot_acceleration_deceleration_high_);
    crane::get_or_declare_parameter(
      this, "robot_acceleration.deceleration_low", robot_acceleration_deceleration_low_);
    crane::get_or_declare_parameter(
      this, "robot_acceleration.velocity_threshold", robot_acceleration_velocity_threshold_);

    world_model = std::make_shared<WorldModelWrapper>(*this);

    sent_commands_pub_ = create_publisher<VelocityCommandsMsg>("/sent_robot_commands", 10);
    sub_commands = create_subscription<VelocityCommandsMsg>(
      "/robot_commands", 10, [this](const VelocityCommandsMsg & msg) { callback(msg); });

    // true: 機体 N の CM4（192.168.20.(100+N)）へ個別に送る（実機）。
    // false: 全機体を target_address へ送る（シミュレーション。cm4_sim が byte 0 で振り分ける）
    const bool unicast_to_robot_ip =
      crane::get_or_declare_parameter(this, "unicast_to_robot_ip", true);
    const std::string target_address =
      crane::get_or_declare_parameter(this, "target_address", std::string("127.0.0.1"));
    const int target_port =
      crane::get_or_declare_parameter(this, "target_port", CommConfig::DEFAULT_PORT);
    // position_control.* は CM4 側の位置制御 PID のゲイン（ki / kd の既定 0 は P 制御）。
    // 送り方は sendPositionControlConfig。詳細: framework/docs/robot-side-position-control.md
    crane::get_or_declare_parameter(this, "position_control.kp", position_control_kp_);
    crane::get_or_declare_parameter(
      this, "position_control.deceleration", position_control_deceleration_);
    crane::get_or_declare_parameter(
      this, "position_control.tolerance", position_control_tolerance_);
    crane::get_or_declare_parameter(this, "position_control.ki", position_control_ki_);
    crane::get_or_declare_parameter(this, "position_control.kd", position_control_kd_);

    // CM4 側の既定は 12350（ai_cmd_v2.out / cm4_sim.out の --config-port）。
    const int position_control_config_port =
      crane::get_or_declare_parameter(this, "position_control.config_port", 12350);

    try {
      for (int id = 0; id < CommConfig::AI_CMD_V2_ROBOT_NUM; ++id) {
        const auto address = boost::asio::ip::address::from_string(
          unicast_to_robot_ip ? robotAddress(id) : target_address);
        command_endpoints_[id] = boost::asio::ip::udp::endpoint(address, target_port);
        config_endpoints_[id] =
          boost::asio::ip::udp::endpoint(address, position_control_config_port);
      }

      checkNetworkInterfaces();

      RCLCPP_INFO(
        get_logger(), "【Unicast Mode Initialized】 %s",
        unicast_to_robot_ip ? "per-robot CM4 address" : "single target address");
      RCLCPP_INFO(
        get_logger(), "  Robot 0 Endpoint: %s:%d (config: %d)",
        command_endpoints_[0].address().to_string().c_str(), target_port,
        position_control_config_port);
      RCLCPP_INFO(
        get_logger(), "  Local Socket: %s:%d",
        udp_socket_.local_endpoint().address().to_string().c_str(),
        udp_socket_.local_endpoint().port());
    } catch (std::exception & e) {
      RCLCPP_ERROR(get_logger(), "❌ Socket Init Error: %s", e.what());
    }

    // 1 Hz 以下の設定トピックなので、後から繋いだ購読者にも現在値が届くよう transient local。
    position_control_config_pub_ = create_publisher<crane_msgs::msg::PositionControlConfig>(
      "/position_control_config", rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(
      get_logger(), "ibis_sender_node started (%s:%d)",
      command_endpoints_[0].address().to_string().c_str(), target_port);
  }

private:
  // 実機 CM4 のアドレス規則（crane_robot_receiver の ping_status_node と同じ）
  static std::string robotAddress(int id) { return std::format("192.168.20.{}", 100 + id); }

  void callback(const VelocityCommandsMsg & msg)
  {
    if (!world_model->hasUpdated()) {
      return;
    }

    const auto now = clock.now();

    VelocityCommandsMsg preprocessed_msg = msg;

    for (auto & command : preprocessed_msg.robot_commands) {
      if (command.control_mode == crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE) {
        if (command.polar_velocity_target_mode.empty()) {
          command.polar_velocity_target_mode.emplace_back();
        }
      }

      command.latency_ms = current_latency_ms;
      command.kick_power = std::clamp(
        command.kick_power, 0.f,
        static_cast<float>(
          command.chip_enable ? kick_power_limit_chip : kick_power_limit_straight));
      command.dribble_power = std::clamp(command.dribble_power, 0.f, 1.f);

      try {
        const auto elapsed =
          now - world_model->getOurRobot(command.robot_id)->vision_detection_stamp;
        // 浮動小数から符号なし整数への範囲外変換は未定義動作なので、代入前に飽和させる。
        // 飽和させないと 65536ms(65.5秒) が 0 になり、vision を 1 分以上見失っている状態で
        // 「たった今検出した」と主張してしまう。受信側(G474 / cm4_sim)の
        // elapsed_time_ms_since_last_vision > 500 による停止判定を素通りさせる最悪の値になる。
        const double elapsed_ms = elapsed.nanoseconds() / 1e6;
        command.elapsed_time_ms_since_last_vision =
          static_cast<uint16_t>(std::clamp(elapsed_ms, 0.0, 65535.0));
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "Failed to get elapsed time of vision from world_model");
        // world_model からロボットを引けない状況で 0（＝たった今検出した）を送るのは
        // 最も危険な向きの fail-open なので、最大陳腐化を送って受信側を停止させる。
        // 実機 G474 / CM4 / simulator-cli はいずれも
        // elapsed_time_ms_since_last_vision > 500 で停止する。
        // なお is_vision_available は createRobotPacket で available_ids から
        // 独立に計算されるが、この catch は robots.at(id) が範囲外 id で投げた場合にしか
        // 入らず、その id は available_ids にも含まれないため false になる。
        // 両フィールドが矛盾する組み合わせにはならない。
        command.elapsed_time_ms_since_last_vision = 65535;
      }

      if (
        auto previous_command = std::ranges::find_if(
          previous_commands.robot_commands,
          [command](const auto & prev_cmd) { return command.robot_id == prev_cmd.robot_id; });
        previous_command != previous_commands.robot_commands.end()) {
        if (
          std::abs(command.target_theta - previous_command->target_theta) <
          command.local_planner_config.theta_tolerance) {
          command.target_theta = previous_command->target_theta;
        }
      }
    }

    if (no_movement) {
      for (auto & command : preprocessed_msg.robot_commands) {
        command.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
        if (command.polar_velocity_target_mode.empty()) {
          command.polar_velocity_target_mode.emplace_back();
        }
        command.polar_velocity_target_mode.front().target_velocity_r = 0.0f;
        command.polar_velocity_target_mode.front().target_velocity_theta = 0.0f;
        command.omega_limit = 0.0f;
        command.chip_enable = false;
        command.dribble_power = 0.0;
        command.kick_power = 0.0;
      }
    }

    previous_commands = preprocessed_msg;
    sendCommands(preprocessed_msg);

    // 実際に送った内容を残す。hasUpdated() は world model を一度でも受信したかの判定なので、
    // 起動直後を除けば /robot_commands の 1 フレームに /sent_robot_commands が 1 メッセージ
    // 対応する。
    // 送信の成否は sendCommands が planning_factors の SenderSent に残す。
    preprocessed_msg.header.stamp = now;
    sent_commands_pub_->publish(preprocessed_msg);
  }

  double calculateAccelerationLimit(double current_speed, double target_speed) const
  {
    if (current_speed < target_speed) {
      return robot_acceleration_acceleration_;
    } else if (current_speed >= robot_acceleration_velocity_threshold_) {
      return robot_acceleration_deceleration_high_;
    } else {
      return robot_acceleration_deceleration_low_;
    }
  }

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

      if (ifa->ifa_addr->sa_family == AF_INET) {
        struct sockaddr_in * addr_in = reinterpret_cast<struct sockaddr_in *>(ifa->ifa_addr);
        char ip_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(addr_in->sin_addr), ip_str, INET_ADDRSTRLEN);

        std::string log_msg =
          "  Interface: " + std::string(ifa->ifa_name) + " IP: " + std::string(ip_str);

        if (ifa->ifa_flags & IFF_UP) log_msg += " [UP]";
        if (ifa->ifa_flags & IFF_RUNNING) log_msg += " [RUNNING]";

        RCLCPP_INFO(get_logger(), "%s", log_msg.c_str());
      }
    }

    freeifaddrs(interfaces);
  }

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
    packet.latency_time_ms = toLatencyTimeMs(command.latency_ms);
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

  void sendCommands(crane_msgs::msg::RobotCommands & msg)
  {
    if (++counter_ > 200) {
      counter_ = 0;
    }

    const auto available_ids = world_model->ours().robotsWhere().available().getIds();

    last_sent_robot_ids_.clear();
    for (auto & command : msg.robot_commands) {
      if (command.robot_id >= CommConfig::AI_CMD_V2_ROBOT_NUM) {
        continue;
      }
      RobotCommandV2 packet = createRobotPacket(command, counter_, available_ids);
      RobotCommandSerializedV2 serialized_packet{};
      RobotCommandSerializedV2_serialize(&serialized_packet, &packet);

      // ワイヤに載せた安全停止関連の値を /sent_robot_commands 用に残す。
      // CM4 / G474 は is_vision_available=0 や elapsed>500ms、linear_velocity_limit=0 で止まる
      addOrUpdatePlanningFactor(
        command, "SenderVisionAvailable", packet.is_vision_available ? "1" : "0");
      addOrUpdatePlanningFactor(
        command, "SenderWireMode", std::to_string(static_cast<int>(packet.control_mode)));
      addOrUpdatePlanningFactor(
        command, "SenderLinearVelocityLimit", formatPlanningDouble(packet.linear_velocity_limit));
      addOrUpdatePlanningFactor(
        command, "SenderAccelerationLimit", formatPlanningDouble(packet.acceleration_limit));
      addOrUpdatePlanningFactor(
        command, "SenderElapsedVisionMs", std::to_string(packet.elapsed_time_ms_since_last_vision));

      std::array<uint8_t, CommConfig::AI_CMD_V2_PACKET_SIZE> buf{};
      buf[0] = command.robot_id;
      memcpy(&buf[1], serialized_packet.data, CommConfig::AI_CMD_V2_SIZE);

      // 機体ごとに送るので、1 台の CM4 に届かなくても他の機体への送信は続ける。
      // 電源の入っていない機体宛ては毎フレーム失敗しうるのでログは間引く
      bool sent = false;
      try {
        udp_socket_.send_to(boost::asio::buffer(buf), command_endpoints_[command.robot_id]);
        sent = true;
        last_sent_robot_ids_.push_back(command.robot_id);
      } catch (std::exception & e) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000, "❌ Packet Send Error (robot_id=%d, %s): %s",
          static_cast<int>(command.robot_id),
          command_endpoints_[command.robot_id].address().to_string().c_str(), e.what());
      }
      // 送信に失敗したフレームも /sent_robot_commands には残るので、送れたかどうかを区別する。
      // CM4 は ACK を返さないため、1 でもロボットが受け取った証拠にはならない
      addOrUpdatePlanningFactor(command, "SenderSent", sent ? "1" : "0");
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
   * 直近の sendCommands で指令を送った機体それぞれへユニキャストで送る。byte 5 は 0xFF
   * （全機に適用する値）のままにし、宛先は UDP で機体ごとに指定する。シミュレーションでは
   * 全機体の宛先が同じ cm4_sim なので、同じ宛先へは 1 回だけ送る。
   *
   * 1 秒ごとに get_parameter() で読み直して送る。ros2 param set で変えた値がそのまま乗り、
   * ロボットの再起動は要らない。同じ値の再送は無害で（CM4 は値が変わったときだけログを
   * 出す）、CM4 が再起動しても次の送信で追いつく。範囲外の値は CM4 側でクランプされず
   * データグラムごと破棄され、拒否理由が CM4 のログに出る。
   *
   * 【フォーマット v2（28 バイト）について】
   * ki / kd を含む v2 を送る。**後方互換は無い。**
   * CM4 は旧 v1（20 バイト）を WrongSize として拒否するので、crane と CM4 は
   * 同時に配ること。片方だけ古い機体は停止せず既定ゲイン（kp = 2.0）のまま走る。
   * 中途半端に互換を残すと「kp だけ効いて ki/kd が効いていない機体」が黙って混ざり、
   * 現地では「なんとなく追従が悪い」以外の症状が出ない。
   * リリース順序は docs/network.md に書いてある。
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

    config_msg.sent = !last_sent_robot_ids_.empty();
    std::vector<boost::asio::ip::udp::endpoint> sent_endpoints;
    for (const auto id : last_sent_robot_ids_) {
      const auto & endpoint = config_endpoints_[id];
      if (std::ranges::find(sent_endpoints, endpoint) != sent_endpoints.end()) {
        continue;
      }
      sent_endpoints.push_back(endpoint);
      try {
        udp_socket_.send_to(boost::asio::buffer(buf), endpoint);
      } catch (std::exception & e) {
        config_msg.sent = false;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "位置制御設定パケットの送信に失敗 (robot_id=%d): %s",
          static_cast<int>(id), e.what());
      }
    }

    // 送信のたびに残す。理由と読み方は docs/network.md に書いてある。
    position_control_config_pub_->publish(config_msg);
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
