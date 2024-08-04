// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_basics/time.hpp>

#include "crane_local_planner/local_planner.hpp"

namespace crane
{
void LocalPlannerComponent::callbackRobotCommands(const crane_msgs::msg::RobotCommands & msg)
{
  if (!world_model->hasUpdated()) {
    return;
  }
  ScopedTimer process_timer(process_time_pub);

  crane_msgs::msg::RobotCommands commands;
  for (const auto & raw_command : msg.robot_commands) {
    bool is_valid = true;
    switch (raw_command.control_mode) {
      case crane_msgs::msg::RobotCommand::LOCAL_CAMERA_MODE:
        if (raw_command.local_camera_mode.empty()) {
          is_valid = false;
          std::stringstream what;
          what << "The robot " << static_cast<int>(raw_command.robot_id)
               << " is specified as LOCAL_CAMERA_MODE by \"" << raw_command.skill_name
               << "\" skill , but no local_camera_mode is set.";
          RCLCPP_ERROR(get_logger(), what.str().c_str());
        }
        break;
      case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE:
        if (raw_command.position_target_mode.empty()) {
          is_valid = false;
          std::stringstream what;
          what << "The robot " << static_cast<int>(raw_command.robot_id)
               << " is specified as POSITION_TARGET_MODE by \"" << raw_command.skill_name
               << "\" skill , but no position_target_mode is set.";
          RCLCPP_ERROR(get_logger(), what.str().c_str());
        }
        break;
      case crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE:
        if (raw_command.simple_velocity_target_mode.empty()) {
          is_valid = false;
          std::stringstream what;
          what << "The robot " << static_cast<int>(raw_command.robot_id)
               << " is specified as SIMPLE_VELOCITY_TARGET_MODE by \"" << raw_command.skill_name
               << "\" skill , but simple_velocity_target_mode "
                  "is set.";
          RCLCPP_ERROR(get_logger(), what.str().c_str());
        }
        break;
      case crane_msgs::msg::RobotCommand::VELOCITY_TARGET_MODE:
        if (raw_command.velocity_target_mode.empty()) {
          is_valid = false;
          std::stringstream what;
          what << "The robot " << static_cast<int>(raw_command.robot_id)
               << " is specified as VELOCITY_TARGET_MODE by \"" << raw_command.skill_name
               << "\" skill , but no velocity_target_mode is set.";
          RCLCPP_ERROR(get_logger(), what.str().c_str());
        }
        break;
      default:
        is_valid = false;
        std::stringstream what;
        what << "The robot " << static_cast<int>(raw_command.robot_id)
             << " is specified as an unknown control mode by \"" << raw_command.skill_name
             << "\" skill.";
        RCLCPP_ERROR(get_logger(), what.str().c_str());
        break;
    }
    if (is_valid) {
      crane_msgs::msg::RobotCommand command = raw_command;
      auto robot = world_model->getOurRobot(command.robot_id);
      command.current_pose.x = robot->pose.pos.x();
      command.current_pose.y = robot->pose.pos.y();
      command.current_pose.theta = robot->pose.theta;
      command.current_velocity.x = robot->vel.linear.x();
      command.current_velocity.y = robot->vel.linear.y();
      command.current_velocity.theta = robot->vel.omega;
      commands.robot_commands.push_back(command);
    }
  }

  auto pub_msg = calculate_control_target(commands);
  pub_msg.header.stamp = rclcpp::Clock().now();
  pub_msg.is_yellow = world_model->isYellow();
  commands_pub->publish(pub_msg);
}
}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::LocalPlannerComponent)
