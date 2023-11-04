// Copyright (c) 2019 SSL-Roots
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TELEOP__JOYSTICK_COMPONENT_HPP_
#define CRANE_TELEOP__JOYSTICK_COMPONENT_HPP_

#include <crane_msgs/msg/robot_commands.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "visibility_control.h"

namespace joystick
{
class JoystickComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit JoystickComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<crane_msgs::msg::RobotCommands>::SharedPtr pub_commands;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;

  void publish_robot_commands(const sensor_msgs::msg::Joy::SharedPtr msg);

  double kick_power = 0.5;
  double dribble_power = 0.5;
  int robot_id = 0;

  std::shared_ptr<rclcpp::ParameterEventHandler> robot_id_subscriber;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> robot_id_callback_handle;
};

}  // namespace joystick

#endif  // CRANE_TELEOP__JOYSTICK_COMPONENT_HPP_
