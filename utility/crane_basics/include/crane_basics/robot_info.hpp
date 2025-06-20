// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__ROBOT_INFO_HPP_
#define CRANE_BASICS__ROBOT_INFO_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_physics/ball_contact.hpp>
#include <memory>
#include <rclcpp/time.hpp>

namespace crane
{
struct RobotIdentifier
{
  bool is_ours;

  uint8_t id;

  [[nodiscard]] auto operator==(const RobotIdentifier & other) const -> bool
  {
    return is_ours == other.is_ours && id == other.id;
  }

  [[nodiscard]] auto operator!=(const RobotIdentifier & other) const -> bool
  {
    return not(*this == other);
  }
};

struct RobotInfo
{
  uint8_t id;

  [[nodiscard]] auto getID() const -> RobotIdentifier { return {true, id}; }

  Pose2D pose;

  Velocity2D vel;

  bool available = false;

  rclcpp::Time vision_detection_stamp;

  rclcpp::Time ball_sensor_stamp;

  bool ball_sensor = false;

  auto getBallSensorAvailable(
    rclcpp::Time now, rclcpp::Duration interval = rclcpp::Duration::from_seconds(0.001)) const
    -> bool
  {
    return now.get_clock_type() == ball_sensor_stamp.get_clock_type() &&
           (now - ball_sensor_stamp).seconds() < interval.seconds();
  }

  using SharedPtr = std::shared_ptr<RobotInfo>;

  [[nodiscard]] auto getDribblerDistance() const -> double { return 0.090; }

  [[nodiscard]] auto center_to_kicker() const -> Vector2
  {
    return getNormVec(pose.theta) * getDribblerDistance();
  }

  [[nodiscard]] auto kicker_center() const -> Point { return pose.pos + center_to_kicker(); }

  BallContact ball_contact;

  auto geometry() const { return Circle{pose.pos, 0.060}; }

  auto getDistance(const Point & pos) const -> double { return (pos - pose.pos).norm(); }

  auto getDistance(const Pose2D & pose2d) const -> double
  {
    return (this->pose.pos - pose2d.pos).norm();
  }
};
}  // namespace crane

#endif  // CRANE_BASICS__ROBOT_INFO_HPP_
