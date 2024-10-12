// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__TELEOP_HPP_
#define CRANE_ROBOT_SKILLS__TELEOP_HPP_

#include <Eigen/Geometry>
#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <sensor_msgs/msg/joy.hpp>

namespace crane::skills
{
class Teleop : public SkillBase<RobotCommandWrapperPosition>, public rclcpp::Node
{
public:
  explicit Teleop(RobotCommandWrapperBase::SharedPtr & base);

  Status update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) override;

  void print(std::ostream & os) const override { os << "[Teleop]"; }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscription;

  sensor_msgs::msg::Joy last_joy_msg;

  double kick_power = 0.5;

  double dribble_power = 0.3;

  double theta = 0.0;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__TELEOP_HPP_
