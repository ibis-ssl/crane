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

#ifndef CRANE_TELEOP__JOYSTICK_COMPONENT_HPP_
#define CRANE_TELEOP__JOYSTICK_COMPONENT_HPP_

#include "crane_msgs/msg/robot_commands.hpp"
#include "crane_teleop/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace joystick
{
class JoystickComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit JoystickComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<crane_msgs::msg::RobotCommands>::SharedPtr pub_commands_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;

  void publish_robot_commands(const sensor_msgs::msg::Joy::SharedPtr msg);

  double kick_power = 0.5;
  double dribble_power = 0.5;
  int robot_id = 0;
  bool is_yellow = false;
  bool all_member = false;
};

}  // namespace joystick

#endif  // CRANE_TELEOP__JOYSTICK_COMPONENT_HPP_
