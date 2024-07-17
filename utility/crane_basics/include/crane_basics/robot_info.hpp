// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__ROBOT_INFO_HPP_
#define CRANE_BASICS__ROBOT_INFO_HPP_

#include <crane_basics/ball_contact.hpp>
#include <crane_basics/boost_geometry.hpp>
#include <memory>

namespace crane
{
struct RobotIdentifier
{
  bool is_ours;

  uint8_t robot_id;

  [[nodiscard]] bool operator==(const RobotIdentifier & other) const
  {
    return is_ours == other.is_ours && robot_id == other.robot_id;
  }

  [[nodiscard]] bool operator!=(const RobotIdentifier & other) const { return not(*this == other); }
};

struct RobotInfo
{
  uint8_t id;

  [[nodiscard]] RobotIdentifier getID() const { return {true, id}; }

  Pose2D pose;

  Velocity2D vel;

  bool available = false;

  using SharedPtr = std::shared_ptr<RobotInfo>;

  [[nodiscard]] Vector2 center_to_kicker() const { return getNormVec(pose.theta) * 0.090; }

  [[nodiscard]] Point kicker_center() const { return pose.pos + center_to_kicker(); }

  BallContact ball_contact;

  auto geometry() { return Circle{pose.pos, 0.060}; }

  double getDistance(Point pos) { return (pos - pose.pos).norm(); }

  double getDistance(Pose2D pose2d) { return (this->pose.pos - pose2d.pos).norm(); }
};
}  // namespace crane

#endif  // CRANE_BASICS__ROBOT_INFO_HPP_
