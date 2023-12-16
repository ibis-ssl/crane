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
  declare_parameter("robot_id", 1);
  get_parameter("robot_id", robot_id);
  robot_id_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
  robot_id_callback_handle =
    robot_id_subscriber->add_parameter_callback("robot_id", [&](const rclcpp::Parameter & p) {
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        robot_id = p.as_int();
      } else {
        std::cout << "robot_id is not integer" << std::endl;
      }
    });

  auto callback = [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {
    publish_robot_commands(msg);
  };

  pub_commands = create_publisher<crane_msgs::msg::RobotCommands>("/robot_commands", 10);
  sub_joy = create_subscription<sensor_msgs::msg::Joy>("joy", 10, callback);
}

void JoystickComponent::publish_robot_commands(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  const int BUTTON_POWER_ENABLE = 9;

  const int AXIS_VEL_SURGE = 1;
  const int AXIS_VEL_SWAY = 0;
  const int AXIS_VEL_ANGULAR = 2;

  const int BUTTON_KICK_TOGGLE = 4;
  const int BUTTON_KICK_STRAIGHT = 13;
  const int BUTTON_KICK_CHIP = 14;
  const int BUTTON_ADJUST_KICK = 0;

  const int BUTTON_ADJUST = 10;
  const int BUTTON_ADJUST_UP = 11;
  const int BUTTON_ADJUST_DOWN = 12;

  const int BUTTON_DRIBBLE_TOGGLE = 6;
  const int BUTTON_ADJUST_DRIBBLE = 1;

  const double MAX_VEL_SURGE = 1.0;
  const double MAX_VEL_SWAY = 1.0;
  const double MAX_VEL_ANGULAR = M_PI;

  static bool is_kick_mode_straight = true;
  static bool is_kick_enable = false;
  static bool is_dribble_enable = false;

  if (msg->buttons[BUTTON_KICK_CHIP]) {
    is_kick_mode_straight = false;
  }
  if (msg->buttons[BUTTON_KICK_STRAIGHT]) {
    is_kick_mode_straight = true;
  }

  auto update_mode = [msg](bool & mode_variable, const int button, bool & is_pushed) {
    // trigger button up
    if (msg->buttons[button]) {
      if (!is_pushed) {
        std::cout << "toggle mode!" << std::endl;
        mode_variable = not mode_variable;
      }
      is_pushed = true;
    } else {
      is_pushed = false;
    }
  };

  static bool is_pushed_kick = false;
  static bool is_pushed_dribble = false;

  update_mode(is_kick_enable, BUTTON_KICK_TOGGLE, is_pushed_kick);
  update_mode(is_dribble_enable, BUTTON_DRIBBLE_TOGGLE, is_pushed_dribble);

  auto adjust_value = [](double & value, const double step) {
    value += step;
    value = std::clamp(value, 0.0, 1.0);
  };

  if (msg->buttons[BUTTON_ADJUST]) {
    static bool is_pushed = false;
    if (msg->buttons[BUTTON_ADJUST_UP]) {
      // trigger button up
      if (!is_pushed) {
        if (msg->buttons[BUTTON_ADJUST_KICK]) {
          adjust_value(kick_power, 0.1);
          std::cout << "kick up: " << kick_power << std::endl;
        }

        if (msg->buttons[BUTTON_ADJUST_DRIBBLE]) {
          adjust_value(dribble_power, 0.1);
          std::cout << "dribble up:" << dribble_power << std::endl;
        }
      }
      is_pushed = true;
    } else if (msg->buttons[BUTTON_ADJUST_DOWN]) {
      // trigger button up
      if (!is_pushed) {
        if (msg->buttons[BUTTON_ADJUST_KICK]) {
          adjust_value(kick_power, -0.1);
          std::cout << "kick down: " << kick_power << std::endl;
        }
        if (msg->buttons[BUTTON_ADJUST_DRIBBLE]) {
          adjust_value(dribble_power, -0.1);
          std::cout << "dribble down: " << dribble_power << std::endl;
        }
      }
      is_pushed = true;
    } else {
      is_pushed = false;
    }
  }

  crane_msgs::msg::RobotCommand command;

  command.robot_id = robot_id;

  // run
  command.target_velocity.x = msg->axes[AXIS_VEL_SURGE] * MAX_VEL_SURGE;
  command.target_velocity.y = msg->axes[AXIS_VEL_SWAY] * MAX_VEL_SWAY;
  command.target_theta.push_back(msg->axes[AXIS_VEL_ANGULAR] * MAX_VEL_ANGULAR);

  // dribble
  if (is_dribble_enable) {
    command.dribble_power = dribble_power;
  } else {
    command.dribble_power = 0.0;
  }

  // kick
  command.chip_enable = is_kick_mode_straight;
  if (is_kick_enable) {
    command.kick_power = kick_power;
    // kick mode
  }

  RCLCPP_INFO(
    get_logger(), "ID=%d Vx=%.3f Vy=%.3f theta=%.3f kick=%s, %.1f dribble=%s, %.1f chip=%s",
    command.robot_id, command.target_velocity.x, command.target_velocity.y,
    command.target_velocity.theta, is_kick_enable ? "ON" : "OFF", kick_power,
    is_dribble_enable ? "ON" : "OFF", dribble_power, command.chip_enable ? "ON" : "OFF");

  if (not msg->buttons[BUTTON_POWER_ENABLE]) {
    crane_msgs::msg::RobotCommand empty_command;
    command = empty_command;
  }

  crane_msgs::msg::RobotCommands robot_commands;

  robot_commands.robot_commands.emplace_back(command);

  pub_commands->publish(robot_commands);
}

}  // namespace joystick

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(joystick::JoystickComponent)
