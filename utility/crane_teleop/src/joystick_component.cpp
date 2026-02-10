// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_teleop/joystick_component.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

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
        RCLCPP_WARN(get_logger(), "robot_id is not integer");
      }
    });

  auto callback = [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {
    publish_robot_commands(msg);
  };

  pub_commands = create_publisher<crane_msgs::msg::RobotCommands>("/robot_commands", 10);
  sub_joy = create_subscription<sensor_msgs::msg::Joy>("joy", 10, callback);
}

auto JoystickComponent::publish_robot_commands(const sensor_msgs::msg::Joy::SharedPtr msg) -> void
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

  auto update_mode = [this, msg](bool & mode_variable, const int button, bool & is_pushed) {
    // trigger button up
    if (msg->buttons[button]) {
      if (!is_pushed) {
        RCLCPP_INFO(get_logger(), "toggle mode!");
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
          RCLCPP_INFO(get_logger(), "kick up: %f", kick_power);
        }

        if (msg->buttons[BUTTON_ADJUST_DRIBBLE]) {
          adjust_value(dribble_power, 0.1);
          RCLCPP_INFO(get_logger(), "dribble up: %f", dribble_power);
        }
      }
      is_pushed = true;
    } else if (msg->buttons[BUTTON_ADJUST_DOWN]) {
      // trigger button up
      if (!is_pushed) {
        if (msg->buttons[BUTTON_ADJUST_KICK]) {
          adjust_value(kick_power, -0.1);
          RCLCPP_INFO(get_logger(), "kick down: %f", kick_power);
        }
        if (msg->buttons[BUTTON_ADJUST_DRIBBLE]) {
          adjust_value(dribble_power, -0.1);
          RCLCPP_INFO(get_logger(), "dribble down: %f", dribble_power);
        }
      }
      is_pushed = true;
    } else {
      is_pushed = false;
    }
  }

  crane_msgs::msg::RobotCommand command;

  command.robot_id = robot_id;
  constexpr double TELEOP_DT = 1.0 / 60.0;
  const double target_vx = msg->axes[AXIS_VEL_SURGE] * MAX_VEL_SURGE;
  const double target_vy = msg->axes[AXIS_VEL_SWAY] * MAX_VEL_SWAY;
  const double target_omega = msg->axes[AXIS_VEL_ANGULAR] * MAX_VEL_ANGULAR;
  const double target_vel_r = std::hypot(target_vx, target_vy);
  const double target_vel_theta = std::atan2(target_vy, target_vx);

  command.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
  command.polar_velocity_target_mode.emplace_back();
  command.polar_velocity_target_mode.front().target_velocity_r = target_vel_r;
  command.polar_velocity_target_mode.front().target_velocity_theta = target_vel_theta;
  theta += target_omega * TELEOP_DT;
  theta = std::atan2(std::sin(theta), std::cos(theta));
  command.target_theta = theta;
  command.omega_limit = MAX_VEL_ANGULAR;
  command.local_planner_config.final_planned_max_velocity.name = "teleop";
  command.local_planner_config.final_planned_max_velocity.value =
    std::max(std::hypot(MAX_VEL_SURGE, MAX_VEL_SWAY), MAX_VEL_SURGE);
  command.local_planner_config.final_planned_max_acceleration.name = "teleop";
  command.local_planner_config.final_planned_max_acceleration.value = 2.5;

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
    command.robot_id, target_vx, target_vy, target_omega, is_kick_enable ? "ON" : "OFF", kick_power,
    is_dribble_enable ? "ON" : "OFF", dribble_power, command.chip_enable ? "ON" : "OFF");

  if (not msg->buttons[BUTTON_POWER_ENABLE]) {
    crane_msgs::msg::RobotCommand empty_command;
    command = empty_command;
  }

  crane_msgs::msg::RobotCommands robot_commands;
  robot_commands.header.stamp = this->get_clock()->now();

  robot_commands.robot_commands.emplace_back(command);

  pub_commands->publish(robot_commands);
}

}  // namespace joystick

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(joystick::JoystickComponent)
