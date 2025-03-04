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
#include <rclcpp/time.hpp>

namespace crane
{
struct RobotIdentifier
{
  bool is_ours;

  uint8_t id;

  [[nodiscard]] bool operator==(const RobotIdentifier & other) const
  {
    return is_ours == other.is_ours && id == other.id;
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

  rclcpp::Time vision_detection_stamp;

  rclcpp::Time ball_sensor_stamp;

  bool ball_sensor = false;

  bool getBallSensorAvailable(
    rclcpp::Time now, rclcpp::Duration interval = rclcpp::Duration::from_seconds(0.001)) const
  {
    return now.get_clock_type() == ball_sensor_stamp.get_clock_type() &&
           (now - ball_sensor_stamp).seconds() < interval.seconds();
  }

  using SharedPtr = std::shared_ptr<RobotInfo>;

  [[nodiscard]] double getDribblerDistance() const { return 0.090; }

  [[nodiscard]] Vector2 center_to_kicker() const
  {
    return getNormVec(pose.theta) * getDribblerDistance();
  }

  [[nodiscard]] Point kicker_center() const { return pose.pos + center_to_kicker(); }

  BallContact ball_contact;

  auto geometry() { return Circle{pose.pos, 0.060}; }

  double getDistance(const Point & pos) { return (pos - pose.pos).norm(); }

  double getDistance(const Pose2D & pose2d) { return (this->pose.pos - pose2d.pos).norm(); }
};
}  // namespace crane

#endif  // CRANE_BASICS__ROBOT_INFO_HPP_
