// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_sender/sender_base.hpp"

#include <algorithm>
#include <cmath>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <ranges>

namespace crane
{

SenderBase::SenderBase(const std::string & name, const rclcpp::NodeOptions & options)
: Node(name, options),
  sub_commands(
    create_subscription<VelocityCommandsMsg>(
      "/robot_commands", 10, [this](const VelocityCommandsMsg & msg) { callback(msg); })),
  clock(RCL_ROS_TIME)
{
  declare_parameter<bool>("no_movement", false);
  get_parameter("no_movement", no_movement);

  declare_parameter<double>("delay_s", 0.0);
  get_parameter("delay_s", delay_s);

  declare_parameter<double>("kick_power_limit_straight", kick_power_limit_straight);
  get_parameter("kick_power_limit_straight", kick_power_limit_straight);

  declare_parameter<double>("kick_power_limit_chip", kick_power_limit_chip);
  get_parameter("kick_power_limit_chip", kick_power_limit_chip);

  declare_parameter<double>("latency_ms", current_latency_ms);
  get_parameter("latency_ms", current_latency_ms);

  declare_parameter("robot_acceleration.acceleration", robot_acceleration_acceleration_);
  get_parameter("robot_acceleration.acceleration", robot_acceleration_acceleration_);

  declare_parameter("robot_acceleration.deceleration_high", robot_acceleration_deceleration_high_);
  get_parameter("robot_acceleration.deceleration_high", robot_acceleration_deceleration_high_);

  declare_parameter("robot_acceleration.deceleration_low", robot_acceleration_deceleration_low_);
  get_parameter("robot_acceleration.deceleration_low", robot_acceleration_deceleration_low_);

  declare_parameter(
    "robot_acceleration.velocity_threshold", robot_acceleration_velocity_threshold_);
  get_parameter("robot_acceleration.velocity_threshold", robot_acceleration_velocity_threshold_);

  world_model = std::make_shared<WorldModelWrapper>(*this);

  RCLCPP_INFO(
    get_logger(),
    "[DIAG] robot_acceleration: acc=%.2f decel_high=%.2f decel_low=%.2f thresh=%.2f",
    robot_acceleration_acceleration_, robot_acceleration_deceleration_high_,
    robot_acceleration_deceleration_low_, robot_acceleration_velocity_threshold_);
}

void SenderBase::callback(const VelocityCommandsMsg & msg)
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
      static_cast<float>(command.chip_enable ? kick_power_limit_chip : kick_power_limit_straight));
    command.dribble_power = std::clamp(command.dribble_power, 0.f, 1.f);

    try {
      const auto elapsed = now - world_model->getOurRobot(command.robot_id)->vision_detection_stamp;
      command.elapsed_time_ms_since_last_vision = elapsed.nanoseconds() / 1e6;
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Failed to get elapsed time of vision from world_model");
      command.elapsed_time_ms_since_last_vision = 0;
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
}

double SenderBase::calculateAccelerationLimit(double current_speed, double target_speed) const
{
  if (current_speed < target_speed) {
    return robot_acceleration_acceleration_;
  } else if (current_speed >= robot_acceleration_velocity_threshold_) {
    return robot_acceleration_deceleration_high_;
  } else {
    return robot_acceleration_deceleration_low_;
  }
}

}  // namespace crane
