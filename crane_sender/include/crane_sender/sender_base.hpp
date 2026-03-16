// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SENDER__SENDER_BASE_HPP_
#define CRANE_SENDER__SENDER_BASE_HPP_

#include <array>
#include <crane_comm/parameter_with_event.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_physics/pid_controller.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace crane
{
struct WorldModelWrapper;

class SenderBase : public rclcpp::Node
{
public:
  explicit SenderBase(const std::string & name, const rclcpp::NodeOptions & options);
  ~SenderBase() override = default;

protected:
  using VelocityCommandsMsg = crane_msgs::msg::RobotCommands;

  const rclcpp::Subscription<VelocityCommandsMsg>::SharedPtr sub_commands;

  std::array<PIDController, 20> theta_controllers;

  virtual void sendCommands(const VelocityCommandsMsg & msg) = 0;

  double calculateAccelerationLimit(double current_speed, double target_speed) const;

  double delay_s{};

  std::shared_ptr<WorldModelWrapper> world_model;

  rclcpp::Clock clock;

  bool no_movement{false};

  double robot_acceleration_acceleration_{2.5};
  double robot_acceleration_deceleration_high_{3.0};
  double robot_acceleration_deceleration_low_{2.0};
  double robot_acceleration_velocity_threshold_{1.5};

private:
  void callback(const VelocityCommandsMsg & msg);

  double current_latency_ms{0.0};

  double kick_power_limit_straight{1.0};

  double kick_power_limit_chip{1.0};

  VelocityCommandsMsg previous_commands;
};
}  // namespace crane

#endif  // CRANE_SENDER__SENDER_BASE_HPP_
