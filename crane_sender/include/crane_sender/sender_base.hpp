// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SENDER__SENDER_BASE_HPP_
#define CRANE_SENDER__SENDER_BASE_HPP_

#include <crane_basics/pid_controller.hpp>
#include <crane_basics/boost_geometry.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace crane
{
class SenderBase : public rclcpp::Node
{
public:
  explicit SenderBase(const std::string name, const rclcpp::NodeOptions & options)
  : Node(name, options),
    sub_commands(create_subscription<crane_msgs::msg::RobotCommands>(
      "/robot_commands", 10, [this](const crane_msgs::msg::RobotCommands & msg) { callback(msg); }))
  {
    declare_parameter<bool>("no_movement", false);
    get_parameter("no_movement", no_movement);

    declare_parameter<double>("delay_s", 0.0);
    get_parameter("delay_s", delay_s);

    // the parameters of the PID controller
    declare_parameter<float>("theta_kp", 4.0);
    declare_parameter<float>("theta_ki", 0.0);
    declare_parameter<float>("theta_kd", 0.1);
    float kp, ki, kd;
    get_parameter("theta_kp", kp);
    get_parameter("theta_ki", ki);
    get_parameter("theta_kd", kd);

    for (auto & controller : theta_controllers) {
      controller.setGain(kp, ki, kd);
    }
  }

protected:
  const rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr sub_commands;

  std::array<PIDController, 20> theta_controllers;

  bool no_movement;

  double delay_s;

  double current_latency_ms = 0.0;

  virtual void sendCommands(const crane_msgs::msg::RobotCommands & msg) = 0;

  float normalizeAngle(float angle_rad) const
  {
    while (angle_rad > M_PI) {
      angle_rad -= 2.0f * M_PI;
    }
    while (angle_rad < -M_PI) {
      angle_rad += 2.0f * M_PI;
    }
    return angle_rad;
  }

  float getAngleDiff(float angle_rad1, float angle_rad2) const
  {
    angle_rad1 = normalizeAngle(angle_rad1);
    angle_rad2 = normalizeAngle(angle_rad2);
    if (abs(angle_rad1 - angle_rad2) > M_PI) {
      if (angle_rad1 - angle_rad2 > 0) {
        return angle_rad1 - angle_rad2 - 2.0f * M_PI;
      } else {
        return angle_rad1 - angle_rad2 + 2.0f * M_PI;
      }
    } else {
      return angle_rad1 - angle_rad2;
    }
  }

private:
  void callback(const crane_msgs::msg::RobotCommands & msg)
  {
    crane_msgs::msg::RobotCommands msg_robot_coordinates = msg;

    for (auto & command : msg_robot_coordinates.robot_commands) {
      command.latency_ms = current_latency_ms;

      switch (command.control_mode) {
        case crane_msgs::msg::RobotCommand::LOCAL_CAMERA_MODE:
          // NOT SUPPORTED NOW
          break;
        case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE:
          break;
        case crane_msgs::msg::RobotCommand::VELOCITY_TARGET_MODE: {
        } break;
        case crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE:
          break;
      }

      if (no_movement) {
        for (auto & command : msg_robot_coordinates.robot_commands) {
          command.control_mode = crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE;
          command.omega_limit = 0.0f;
          command.simple_velocity_target_mode.clear();
          crane_msgs::msg::SimpleVelocityTargetMode target;
          target.target_vx = 0.0f;
          target.target_vy = 0.0f;
          target.speed_limit_at_target = 0.0f;
          command.simple_velocity_target_mode.push_back(target);
          command.chip_enable = false;
          command.dribble_power = 0.0;
          command.kick_power = 0.0;
        }
      }
    }
    sendCommands(msg_robot_coordinates);
  }
};
}  // namespace crane

#endif  // CRANE_SENDER__SENDER_BASE_HPP_
