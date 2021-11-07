// Copyright (c) 2019 SSL-Roots
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "crane_teleop/joystick_component.hpp"

using namespace std::chrono_literals;

float theta;

namespace joystick
{
JoystickComponent::JoystickComponent(const rclcpp::NodeOptions & options)
: Node("crane_teleop", options)
{
  RCLCPP_INFO(this->get_logger(), "hello world");
  auto callback =
    [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void
    {
      publish_robot_commands(msg);
    };

  pub_commands_ =
    create_publisher<crane_msgs::msg::RobotCommands>("/robot_commands", 10);
  sub_joy_ = create_subscription<sensor_msgs::msg::Joy>("joy", 10, callback);
}

void JoystickComponent::publish_robot_commands(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // FIXME: WE HAVE TO USE ROS_PARAM
  const int BUTTON_MOVE_ENABLE = 4;
  const int AXIS_VEL_SURGE = 1;
  const int AXIS_VEL_SWAY = 0;
  const int AXIS_VEL_ANGULAR = 3;

  const double MAX_VEL_SURGE = 1.0;
  const double MAX_VEL_SWAY = 1.0;
  const double MAX_VEL_ANGULAR = M_PI / 15;

  crane_msgs::msg::RobotCommand command;

  if (msg->buttons[BUTTON_MOVE_ENABLE]) {
    command.target.x = msg->axes[AXIS_VEL_SURGE] * MAX_VEL_SURGE;
    command.target.y = msg->axes[AXIS_VEL_SWAY] * MAX_VEL_SWAY;
    theta = theta + msg->axes[AXIS_VEL_ANGULAR] * MAX_VEL_ANGULAR;
    command.target.theta = theta;
    if (msg->buttons[2]) {
      command.dribble_power = 0.5;
    } else {
      command.dribble_power = 0.0;
    }

    if (msg->buttons[4]) {
      if (msg->buttons[0]) {
        command.chip_enable = 1;
      } else {
        command.chip_enable = 0;
      }
      command.kick_power = 0.5;
    } else {
      command.kick_power = 0.0;
    }
  } else {
    theta = 0.0;
  }

  command.robot_id = 0;

  crane_msgs::msg::RobotCommands robot_commands;
  robot_commands.robot_commands.push_back(command);

  pub_commands_->publish(robot_commands);
  printf("ID=%d Vx=%.3f Vy=%.3f theta=%.3f\r\n", command.robot_id, command.target.x,
    command.target.y, command.target.theta);
}


}  // namespace joystick

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(joystick::JoystickComponent)
