// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__ROBOT_COMMAND_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__ROBOT_COMMAND_WRAPPER_HPP_

#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "crane_geometry/boost_geometry.hpp"
#include "crane_geometry/geometry_operations.hpp"
#include "crane_msgs/msg/robot_command.hpp"
#include "eigen3/Eigen/Core"

namespace crane
{
struct RobotCommandWrapper
{
  typedef std::shared_ptr<RobotCommandWrapper> SharedPtr;

  RobotCommandWrapper & kickWithChip(double power)
  {
    latest_msg_.chip_enable = true;
    latest_msg_.kick_power = power;
    return *this;
  }

  RobotCommandWrapper & kickStraight(double power)
  {
    latest_msg_.chip_enable = false;
    latest_msg_.kick_power = power;
    return *this;
  }

  RobotCommandWrapper & dribble(double power)
  {
    latest_msg_.dribble_power = power;
    latest_msg_.kick_power = 0.0;
    return *this;
  }

  RobotCommandWrapper & setVelocity(Velocity velocity)
  {
    return setVelocity(velocity.x(), velocity.y());
  }

  RobotCommandWrapper & setVelocity(double x, double y)
  {
    latest_msg_.motion_mode_enable = true;
    latest_msg_.target.x = x;
    latest_msg_.target.y = y;
    return *this;
  }

  RobotCommandWrapper & setTargetPosition(double x, double y, double theta)
  {
    latest_msg_.motion_mode_enable = false;
    latest_msg_.target.x = x;
    latest_msg_.target.y = y;
    latest_msg_.target.theta = theta;
    return *this;
  }

  RobotCommandWrapper & setTargetPosition(double x, double y)
  {
    return setTargetPosition(x, y, latest_msg_.target.theta);
  }

  RobotCommandWrapper & setTargetPosition(Point position)
  {
    return setTargetPosition(position.x(), position.y());
  }

  RobotCommandWrapper & setTargetPosition(Point position, double theta)
  {
    return setTargetPosition(position.x(), position.y(), theta);
  }

  RobotCommandWrapper & setTargetTheta(double theta)
  {
    latest_msg_.target.theta = theta;
    return *this;
  }

  RobotCommandWrapper & disablePlacementAvoidance()
  {
    latest_msg_.disable_placement_avoidance = true;
    return *this;
  }

  RobotCommandWrapper & disableCollisionAvoidance()
  {
    latest_msg_.disable_collision_avoidance = true;
    return *this;
  }

  RobotCommandWrapper & disableGoalAreaAvoidance()
  {
    latest_msg_.disable_goal_area_avoidance = true;
    return *this;
  }

  RobotCommandWrapper & setGoalieDefault()
  {
    disableCollisionAvoidance();
    disableGoalAreaAvoidance();
    return *this;
  }

  RobotCommandWrapper & enableBallCenteringControl()
  {
    latest_msg_.enable_ball_centering_control = true;
    return *this;
  }

  RobotCommandWrapper & enableLocalGoalie()
  {
    latest_msg_.local_goalie_enable = true;
    return *this;
  }

  RobotCommandWrapper & setID(uint8_t id)
  {
    latest_msg_.robot_id = id;
    return *this;
  }

  const crane_msgs::msg::RobotCommand & getMsg() const { return latest_msg_; }
  crane_msgs::msg::RobotCommand & getEditableMsg() { return latest_msg_; }
  crane_msgs::msg::RobotCommand latest_msg_;
};
}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__ROBOT_COMMAND_WRAPPER_HPP_
