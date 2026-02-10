// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <math.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <array>
#include <boost/asio.hpp>
#include <class_loader/visibility_control.hpp>
#include <cmath>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <format>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "crane_sender/broadcast_command_sender.hpp"
#include "crane_sender/robot_packet.h"
#include "crane_sender/sender_base.hpp"

namespace crane
{

class IbisSenderNode : public SenderBase
{
private:
  int debug_id;

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_subscriber;

  std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_handle;

  std::shared_ptr<BroadcastCommandSender> broadcast_sender;

  // ロボット送信用の加速度パラメータ
  double robot_acceleration_acceleration;
  double robot_acceleration_deceleration_high;
  double robot_acceleration_deceleration_low;
  double robot_acceleration_velocity_threshold;

public:
  CLASS_LOADER_PUBLIC
  explicit IbisSenderNode(const rclcpp::NodeOptions & options) : SenderBase("ibis_sender", options)
  {
    declare_parameter("debug_id", -1);
    get_parameter("debug_id", debug_id);

    // ロボット送信用の加速度パラメータを読み込み
    declare_parameter("robot_acceleration.acceleration", 2.5);
    get_parameter("robot_acceleration.acceleration", robot_acceleration_acceleration);

    declare_parameter("robot_acceleration.deceleration_high", 3.0);
    get_parameter("robot_acceleration.deceleration_high", robot_acceleration_deceleration_high);

    declare_parameter("robot_acceleration.deceleration_low", 2.0);
    get_parameter("robot_acceleration.deceleration_low", robot_acceleration_deceleration_low);

    declare_parameter("robot_acceleration.velocity_threshold", 1.5);
    get_parameter("robot_acceleration.velocity_threshold", robot_acceleration_velocity_threshold);

    parameter_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
    parameter_callback_handle =
      parameter_subscriber->add_parameter_callback("debug_id", [&](const rclcpp::Parameter & p) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          debug_id = p.as_int();
        } else {
          RCLCPP_WARN(get_logger(), "Warning: debug_id must be an integer");
        }
      });

    // ブロードキャスト送信システム初期化
    broadcast_sender = std::make_shared<BroadcastCommandSender>();
    RCLCPP_INFO(get_logger(), "ibis_sender_node started");
  }

private:
  RobotCommandV2 createRobotPacket(const crane_msgs::msg::RobotCommand & command, int counter)
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
    packet.is_vision_available = [&]() -> bool {
      std::vector<uint8_t> available_ids = world_model->ours().robotsWhere().available().getIds();
      return std::count(available_ids.begin(), available_ids.end(), command.robot_id) == 1;
    }();
    packet.target_global_theta = command.target_theta;
    packet.kick_power = command.kick_power;
    packet.dribble_power = std::clamp(command.dribble_power, 0.f, 1.f);
    packet.enable_chip = command.chip_enable;
    packet.stop_emergency = command.stop_flag;

    // 現在の速度から加速度を選択
    double current_speed = std::hypot(command.current_velocity.x, command.current_velocity.y);
    double target_speed = resolved_max_velocity;

    double selected_acceleration;
    if (current_speed < target_speed) {
      // 加速時
      selected_acceleration = robot_acceleration_acceleration;
    } else {
      // 減速時：速度に応じて選択
      if (current_speed >= robot_acceleration_velocity_threshold) {
        // 高速域
        selected_acceleration = robot_acceleration_deceleration_high;
      } else {
        // 低速域
        selected_acceleration = robot_acceleration_deceleration_low;
      }
    }

    packet.acceleration_limit =
      resolved_max_acceleration > 0.0f
        ? std::min(static_cast<float>(selected_acceleration), resolved_max_acceleration)
        : selected_acceleration;
    packet.linear_velocity_limit = resolved_max_velocity;
    packet.angular_velocity_limit = command.omega_limit;
    packet.latency_time_ms = static_cast<uint8_t>(command.latency_ms);
    packet.elapsed_time_ms_since_last_vision = command.elapsed_time_ms_since_last_vision;

    // RobotCommand は常に極座標速度モード
    packet.control_mode = POLAR_VELOCITY_TARGET_MODE;
    packet.mode_args.polar_velocity.target_global_velocity_r = target_velocity_r;
    packet.mode_args.polar_velocity.target_global_velocity_theta = target_velocity_theta;

    return packet;
  }

public:
  void sendCommands(const crane_msgs::msg::RobotCommands & msg) override
  {
    static int counter = 0;
    static int call_count = 0;
    call_count++;

    if (++counter > 200) {
      counter = 0;
    }

    RCLCPP_DEBUG(
      get_logger(), "🚀 sendCommands method called #%d (counter=%d)", call_count, counter);
    RCLCPP_DEBUG(get_logger(), "  Received robot command count: %ld", msg.robot_commands.size());

    // ブロードキャストモード：全ロボットのパケットを作成して一括送信
    std::vector<std::pair<uint8_t, RobotCommandSerializedV2>> robot_packets;

    // 全11スロット分の配列を初期化（ロボットIDでインデックス指定）
    std::array<std::optional<RobotCommandSerializedV2>, CommConfig::AI_CMD_V2_ROBOT_NUM>
      packet_slots;

    int processed_commands = 0;
    for (auto command : msg.robot_commands) {
      if (command.robot_id < CommConfig::AI_CMD_V2_ROBOT_NUM) {
        RobotCommandV2 packet = createRobotPacket(command, counter);
        RobotCommandSerializedV2 serialized_packet;
        RobotCommandSerializedV2_serialize(&serialized_packet, &packet);
        packet_slots[command.robot_id] = serialized_packet;
        processed_commands++;
      }
    }

    RCLCPP_DEBUG(
      get_logger(), "  Processed command count: %d/%ld", processed_commands,
      msg.robot_commands.size());

    // 全スロットを順番にパケットリストに追加（空のスロットは空パケット）
    int filled_slots = 0;
    for (int i = 0; i < CommConfig::AI_CMD_V2_ROBOT_NUM; i++) {
      if (packet_slots[i]) {
        robot_packets.push_back(std::make_pair(i, packet_slots[i].value()));
        filled_slots++;
      } else {
        // 空パケットを作成
        RobotCommandSerializedV2 empty_packet = {};
        robot_packets.push_back(std::make_pair(i, empty_packet));
      }
    }

    RCLCPP_DEBUG(
      get_logger(), "  Packet slot usage: %d/%d slots used", filled_slots,
      CommConfig::AI_CMD_V2_ROBOT_NUM);
    RCLCPP_DEBUG(get_logger(), "  Broadcast preparation complete -> Start sending packet");

    broadcast_sender->sendBroadcastPackets(robot_packets, counter);

    // 定期的な詳細ログ（100回に1回）
    if (call_count % 100 == 0) {
      RCLCPP_INFO(
        get_logger(), "📈 Statistics (every 100 calls): Total calls=%d Avg cmd count=%ld",
        call_count, msg.robot_commands.size());
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
