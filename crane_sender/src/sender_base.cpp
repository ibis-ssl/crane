// Copyright (c) 2025 ibis-ssl

#include "crane_sender/sender_base.hpp"

#include <algorithm>
#include <cmath>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <iostream>
#include <ranges>

namespace crane
{

SenderBase::SenderBase(const std::string & name, const rclcpp::NodeOptions & options)
: Node(name, options),
  sub_commands(
    create_subscription<RobotCommandsMsg>(
      "/robot_commands", 10, [this](const RobotCommandsMsg & msg) { callback(msg); })),
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

  world_model = std::make_shared<WorldModelWrapper>(*this);
}

void SenderBase::callback(const RobotCommandsMsg & msg)
{
  if (!world_model->hasUpdated()) {
    return;
  }

  const auto now = clock.now();

  RobotCommandsMsg preprocessed_msg = msg;

  for (auto & command : preprocessed_msg.robot_commands) {
    command.latency_ms = current_latency_ms;
    command.kick_power = std::clamp(command.kick_power, 0.f, [this, command]() -> float {
      return command.chip_enable ? kick_power_limit_chip : kick_power_limit_straight;
    }());
    command.dribble_power = std::clamp(command.dribble_power, 0.f, 1.f);

    try {
      const auto elapsed = now - world_model->getOurRobot(command.robot_id)->vision_detection_stamp;
      command.elapsed_time_ms_since_last_vision = elapsed.nanoseconds() / 1e6;
    } catch (...) {
      std::cerr << "Error: Failed to get elapsed time of vision from world_model" << std::endl;
      command.elapsed_time_ms_since_last_vision = 0;
    }

    if (auto previous_command = std::ranges::find_if(
          previous_commands.robot_commands,
          [command](const auto & prev_cmd) { return command.robot_id == prev_cmd.robot_id; });
        previous_command != previous_commands.robot_commands.end()) {
      if (
        std::abs(command.target_theta - previous_command->target_theta) <
        command.local_planner_config.theta_tolerance) {
        command.target_theta = previous_command->target_theta;
      }
    }

    switch (command.control_mode) {
      case crane_msgs::msg::RobotCommand::LOCAL_CAMERA_MODE:
        break;
      case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE:
        break;
      case crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE:
        break;
      case crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE:
        break;
    }
  }

  if (no_movement) {
    for (auto & command : preprocessed_msg.robot_commands) {
      command.control_mode = crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE;
      command.omega_limit = 0.0F;
      command.simple_velocity_target_mode.clear();
      crane_msgs::msg::SimpleVelocityTargetMode target;
      target.target_vx = 0.0F;
      target.target_vy = 0.0F;
      target.speed_limit_at_target = 0.0F;
      command.simple_velocity_target_mode.push_back(target);
      command.chip_enable = false;
      command.dribble_power = 0.0;
      command.kick_power = 0.0;
    }
  }

  previous_commands = preprocessed_msg;
  sendCommands(preprocessed_msg);
}

}  // namespace crane
