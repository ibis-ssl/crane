// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <array>
#include <class_loader/visibility_control.hpp>
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

  int counter_{0};
  int call_count_{0};

public:
  CLASS_LOADER_PUBLIC
  explicit IbisSenderNode(const rclcpp::NodeOptions & options) : SenderBase("ibis_sender", options)
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

    // ブロードキャスト送信システム初期化
    broadcast_sender = std::make_shared<BroadcastCommandSender>();
    RCLCPP_INFO(get_logger(), "ibis_sender_node started");
  }

private:
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

    // 現在の速度から加速度を選択
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

    // RobotCommand は常に極座標速度モード
    packet.control_mode = POLAR_VELOCITY_TARGET_MODE;
    packet.mode_args.polar_velocity.target_global_velocity_r = target_velocity_r;
    packet.mode_args.polar_velocity.target_global_velocity_theta = target_velocity_theta;

    return packet;
  }

public:
  void sendCommands(const crane_msgs::msg::RobotCommands & msg) override
  {
    call_count_++;

    if (++counter_ > 200) {
      counter_ = 0;
    }

    RCLCPP_DEBUG(
      get_logger(), "🚀 sendCommands method called #%d (counter=%d)", call_count_, counter_);
    RCLCPP_DEBUG(get_logger(), "  Received robot command count: %ld", msg.robot_commands.size());

    const auto available_ids = world_model->ours().robotsWhere().available().getIds();

    // ブロードキャストモード：全ロボットのパケットを作成して一括送信
    // 11スロット分を空パケットで初期化
    std::vector<std::pair<uint8_t, RobotCommandSerializedV2>> robot_packets(
      CommConfig::AI_CMD_V2_ROBOT_NUM);
    for (int i = 0; i < CommConfig::AI_CMD_V2_ROBOT_NUM; i++) {
      robot_packets[i] = {static_cast<uint8_t>(i), RobotCommandSerializedV2{}};
    }

    int processed_commands = 0;
    for (const auto & command : msg.robot_commands) {
      if (command.robot_id < CommConfig::AI_CMD_V2_ROBOT_NUM) {
        RobotCommandV2 packet = createRobotPacket(command, counter_, available_ids);
        RobotCommandSerializedV2 serialized_packet;
        RobotCommandSerializedV2_serialize(&serialized_packet, &packet);
        robot_packets[command.robot_id] = {command.robot_id, serialized_packet};
        processed_commands++;
      }
    }

    RCLCPP_DEBUG(
      get_logger(), "  Processed command count: %d/%ld", processed_commands,
      msg.robot_commands.size());

    RCLCPP_DEBUG(
      get_logger(), "  Packet slot usage: %d/%d slots used", processed_commands,
      CommConfig::AI_CMD_V2_ROBOT_NUM);
    RCLCPP_DEBUG(get_logger(), "  Broadcast preparation complete -> Start sending packet");

    broadcast_sender->sendBroadcastPackets(robot_packets, counter_);
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
