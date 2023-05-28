// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_teleop/joystick_component.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

using namespace std::chrono_literals;

float theta;

namespace joystick
{
JoystickComponent::JoystickComponent(const rclcpp::NodeOptions & options)
: Node("crane_teleop", options)
{
  auto callback = [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {
    publish_robot_commands(msg);
  };

  pub_commands = create_publisher<crane_msgs::msg::RobotCommands>("/robot_commands", 10);
  sub_joy = create_subscription<sensor_msgs::msg::Joy>("joy", 10, callback);
}

void JoystickComponent::publish_robot_commands(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // FIXME: WE HAVE TO USE ROS_PARAM
  const int MAX_ID = 10;
  const int BUTTON_SHUTDOWN_1 = 5;
  const int BUTTON_SHUTDOWN_2 = 6;
  const int BUTTON_MOVE_ENABLE = 4;

  const int AXIS_VEL_SURGE = 1;
  const int AXIS_VEL_SWAY = 0;
  const int AXIS_VEL_ANGULAR = 3;

  const int BUTTON_KICK_ENABLE = 2;
  const int BUTTON_KICK_STRAIGHT = 0;
  const int BUTTON_KICK_CHIP = 3;
  const int AXIS_KICK_POWER = 4;

  const int BUTTON_DRIBBLE_ENABLE = 4;
  const int AXIS_DRIBBLE_POWER = 4;

  const int BUTTON_ID_ENABLE = 6;
  const int AXIS_ID_CHANGE = 4;

  const int BUTTON_COLOR_ENABLE = 5;
  const int AXIS_COLOR_CHANGE = 5;

  const int BUTTON_ALL_ID_1 = 1;
  const int BUTTON_ALL_ID_2 = 2;
  const int BUTTON_ALL_ID_3 = 4;
  const int BUTTON_ALL_ID_4 = 6;
  const int BUTTON_PATH_ENABLE = 5;
  const int BUTTON_ADD_POSE = 2;
  const int BUTTON_DELETE_PATH = 1;
  const int BUTTON_SEND_TARGET = 0;

  const double MAX_VEL_SURGE = 1.0;
  const double MAX_VEL_SWAY = 1.0;
  const double MAX_VEL_ANGULAR = M_PI;

  const double MAX_KICK_POWER = 1.0;  // DO NOT EDIT
  const double KICK_POWER_CONTROL = 0.1;

  const double MAX_DRIBBLE_POWER = 1.0;  // DO NOT EDIT
  const double DRIBBLE_POWER_CONTROL = 0.1;

  // shutdown
  if (msg->buttons[BUTTON_SHUTDOWN_1] && msg->buttons[BUTTON_SHUTDOWN_2]) {
    rclcpp::shutdown();
    return;
  }

  // change team color
  if (msg->buttons[BUTTON_COLOR_ENABLE]) {
    static bool is_pushed = false;
    if (fabs(msg->axes[AXIS_COLOR_CHANGE] > 0)) {
      if (!is_pushed) {
        is_yellow = !is_yellow;
        RCLCPP_INFO(get_logger(), "is_yellow : %d", static_cast<int>(is_yellow));
      }
      is_pushed = true;
    } else {
      is_pushed = false;
    }
  }

  // change id
  if (msg->buttons[BUTTON_ID_ENABLE]) {
    static bool is_pushed = false;
    if (fabs(msg->axes[AXIS_ID_CHANGE] > 0)) {
      if (!is_pushed) {
        robot_id += int(msg->axes[AXIS_ID_CHANGE]);
        if (robot_id > MAX_ID) {
          robot_id = MAX_ID;
        } else if (robot_id < 0) {
          robot_id = 0;
        }
        RCLCPP_INFO(get_logger(), "robot id : %d", static_cast<int>(robot_id));
      }
      is_pushed = true;
    } else {
      is_pushed = false;
    }
  }

  // switch all member mode
  if (
    msg->buttons[BUTTON_ALL_ID_1] && msg->buttons[BUTTON_ALL_ID_2] &&
    msg->buttons[BUTTON_ALL_ID_3] && msg->buttons[BUTTON_ALL_ID_4]) {
    static bool is_pushed = false;
    if (fabs(msg->axes[AXIS_COLOR_CHANGE] > 0)) {
      if (!is_pushed) {
        all_member = !all_member;
        RCLCPP_INFO(get_logger(), "all member control mode : %d", static_cast<int>(is_yellow));
      }
      is_pushed = true;
    } else {
      is_pushed = false;
    }
  }

  crane_msgs::msg::RobotCommand command;

  if (msg->buttons[BUTTON_MOVE_ENABLE]) {
    // run
    command.target_velocity.x = msg->axes[AXIS_VEL_SURGE] * MAX_VEL_SURGE;
    command.target_velocity.y = msg->axes[AXIS_VEL_SWAY] * MAX_VEL_SWAY;
    command.target_velocity.theta = msg->axes[AXIS_VEL_ANGULAR] * MAX_VEL_ANGULAR;

    // dribble
    if (msg->buttons[BUTTON_DRIBBLE_ENABLE]) {
      command.dribble_power = dribble_power;
      static bool is_pushed = false;
      if (fabs(msg->axes[AXIS_DRIBBLE_POWER] > 0)) {
        if (!is_pushed) {
          dribble_power += std::copysign(DRIBBLE_POWER_CONTROL, msg->axes[AXIS_DRIBBLE_POWER]);
          RCLCPP_INFO(get_logger(), "dribble power : %f", dribble_power);
        }
        is_pushed = true;
      } else {
        is_pushed = false;
      }
    } else {
      command.dribble_power = 0.0;
    }

    // kick
    if (msg->buttons[BUTTON_KICK_ENABLE]) {
      // kick mode
      if (msg->buttons[BUTTON_KICK_STRAIGHT]) {
        command.kick_power = kick_power;
        command.chip_enable = false;
      } else if (msg->buttons[BUTTON_KICK_CHIP]) {
        command.kick_power = kick_power;
        command.chip_enable = true;
      }

      // change power
      static bool is_pushed = false;
      if (fabs(msg->axes[AXIS_KICK_POWER] > 0)) {
        if (!is_pushed) {
          kick_power += std::copysign(KICK_POWER_CONTROL, msg->axes[AXIS_KICK_POWER]);
          if (kick_power > MAX_KICK_POWER) {
            kick_power = MAX_KICK_POWER;
          } else if (kick_power < 0.0) {
            kick_power = 0.0;
          }
          RCLCPP_INFO(get_logger(), "kick power : %f", kick_power);
        }
        is_pushed = true;
      } else {
        is_pushed = false;
      }
    }
  }

  // set team color
  //  command.is_yellow = is_yellow;

  crane_msgs::msg::RobotCommands robot_commands;
  // set command
  if (all_member) {
    for (int i = 0; i < MAX_ID; i++) {
      command.robot_id = i;
      robot_commands.robot_commands.emplace_back(command);
    }
  } else {
    command.robot_id = robot_id;
    robot_commands.robot_commands.emplace_back(command);
  }
  pub_commands->publish(robot_commands);

  RCLCPP_INFO(
    get_logger(), "ID=%d Vx=%.3f Vy=%.3f theta=%.3f", command.robot_id, command.target_velocity.x,
    command.target_velocity.y, command.target_velocity.theta);
}

}  // namespace joystick

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(joystick::JoystickComponent)
