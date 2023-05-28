// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SENDER__SENDER_BASE_HPP_
#define CRANE_SENDER__SENDER_BASE_HPP_

#include <rclcpp/rclcpp.hpp>

namespace crane
{
class PIDController
{
public:
  PIDController() = default;

  void setGain(float kp, float ki, float kd)
  {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    error_prev = 0.0f;
  }

  float update(float error, float dt)
  {
    float p = kp * error;
    float i = ki * (error + error_prev) * dt / 2.0f;
    float d = kd * (error - error_prev) / dt;
    error_prev = error;
    return p + i + d;
  }

private:
  float kp;

  float ki;

  float kd;

  float error_prev;
};

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

    for (auto & controller : theta_controllers) {
      controller.setGain(4, 0.00, 0.1);
    }
    //    sub_replacement_ = this->create_subscription<robocup_ssl_msgs::msg::Replacement>(
    //      "sim_sender/replacements", 10, std::bind(&SimSender::send_replacement, this, std::placeholders::_1));
  }

protected:
  const rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr sub_commands;

  std::array<PIDController, 11> theta_controllers;

  bool no_movement;

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
    crane_msgs::msg::RobotCommands msg_robot_coordinates(msg);
    // 座標変換（ワールド->各ロボット）
    for (auto & command : msg_robot_coordinates.robot_commands) {
      command.target_velocity.x = command.target_velocity.x * cos(-command.current_pose.theta) -
                                  command.target_velocity.y * sin(-command.current_pose.theta);
      command.target_velocity.y = command.target_velocity.x * sin(-command.current_pose.theta) +
                                  command.target_velocity.y * cos(-command.current_pose.theta);
      //
      if (not command.target_theta.empty()) {
        auto omega =
          theta_controllers.at(command.robot_id)
            .update(getAngleDiff(command.current_pose.theta, command.target_velocity.theta), 0.033);
        command.target_velocity.theta = omega;
      }
    }
  }
};
}  // namespace crane

#endif  // CRANE_SENDER__SENDER_BASE_HPP_
