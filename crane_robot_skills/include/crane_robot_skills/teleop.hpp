// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__TELEOP_HPP_
#define CRANE_ROBOT_SKILLS__TELEOP_HPP_

#include <crane_basics/vector2d_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <sensor_msgs/msg/joy.hpp>

namespace crane::skills
{
class Teleop : public SkillBase, public rclcpp::Node
{
public:
  template <typename... Args>
  explicit Teleop(Args &&... args)
  : SkillBase("Teleop", std::forward<Args>(args)...), Node("teleop_skill")
  {
    setParameter("rotation_deg", 0.);
    setParameter("use_local_coordinate", false);
    joystick_subscription = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, [this](const sensor_msgs::msg::Joy & msg) { last_joy_msg = msg; });
  }

  Status update() override;

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
