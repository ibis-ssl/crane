// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__ROBOT_COMMAND_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__ROBOT_COMMAND_WRAPPER_HPP_

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "crane_geometry/boost_geometry.hpp"
#include "crane_geometry/geometry_operations.hpp"
#include "crane_msgs/msg/robot_command.hpp"

namespace crane
{
struct RobotCommandWrapper
{
  typedef std::shared_ptr<RobotCommandWrapper> SharedPtr;

  RobotCommandWrapper & kickWithChip(double power)
  {
    latest_msg.chip_enable = true;
    latest_msg.kick_power = power;
    return *this;
  }

  RobotCommandWrapper & kickStraight(double power)
  {
    latest_msg.chip_enable = false;
    latest_msg.kick_power = power;
    return *this;
  }

  RobotCommandWrapper & dribble(double power)
  {
    latest_msg.dribble_power = power;
    latest_msg.kick_power = 0.0;
    return *this;
  }

  RobotCommandWrapper & setVelocity(Velocity velocity)
  {
    return setVelocity(velocity.x(), velocity.y());
  }

  RobotCommandWrapper & setVelocity(double x, double y)
  {
    latest_msg.motion_mode_enable = true;
    latest_msg.target_velocity.x = x;
    latest_msg.target_velocity.y = y;
    return *this;
  }

  RobotCommandWrapper & setTargetPosition(double x, double y, double theta)
  {
    latest_msg.motion_mode_enable = false;
    auto set_target = [&](auto & target_array, auto value) {
      if (not target_array.empty()) {
        target_array.front() = value;
      } else {
        target_array.emplace_back(value);
      }
    };

    set_target(latest_msg.target_x, x);
    set_target(latest_msg.target_y, y);
    set_target(latest_msg.target_theta, theta);
    return *this;
  }

  RobotCommandWrapper & setTargetPosition(double x, double y)
  {
    latest_msg.motion_mode_enable = false;
    auto set_target = [&](auto & target_array, auto value) {
      if (not target_array.empty()) {
        target_array.front() = value;
      } else {
        target_array.emplace_back(value);
      }
    };

    set_target(latest_msg.target_x, x);
    set_target(latest_msg.target_y, y);
    return *this;
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
    if (not latest_msg.target_theta.empty()) {
      latest_msg.target_theta.front() = theta;
    } else {
      latest_msg.target_theta.emplace_back(theta);
    }
    return *this;
  }

  RobotCommandWrapper & disablePlacementAvoidance()
  {
    latest_msg.local_planner_config.disable_placement_avoidance = true;
    return *this;
  }

  RobotCommandWrapper & enablePlacementAvoidance()
  {
    latest_msg.local_planner_config.disable_placement_avoidance = false;
    return *this;
  }

  RobotCommandWrapper & disableCollisionAvoidance()
  {
    latest_msg.local_planner_config.disable_collision_avoidance = true;
    return *this;
  }

  RobotCommandWrapper & enableCollisionAvoidance()
  {
    latest_msg.local_planner_config.disable_collision_avoidance = false;
    return *this;
  }

  RobotCommandWrapper & disableGoalAreaAvoidance()
  {
    latest_msg.local_planner_config.disable_goal_area_avoidance = true;
    return *this;
  }

  RobotCommandWrapper & enableGoalAreaAvoidance()
  {
    latest_msg.local_planner_config.disable_goal_area_avoidance = false;
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
    latest_msg.enable_ball_centering_control = true;
    return *this;
  }

  RobotCommandWrapper & enableLocalGoalie()
  {
    latest_msg.local_goalie_enable = true;
    return *this;
  }

  RobotCommandWrapper & setID(uint8_t id)
  {
    latest_msg.robot_id = id;
    return *this;
  }

  RobotCommandWrapper & setBallPosition(Point position)
  {
    latest_msg.current_ball_x = position.x();
    latest_msg.current_ball_y = position.y();
    return *this;
  }

//  RobotCommandWrapper & setBallRelativeVelocity(Velocity velocity)
//  {
//    latest_msg.ball_relative_velocity_x = velocity.x();
//    latest_msg.ball_relative_velocity_y = velocity.y();
//    return *this;
//  }

  RobotCommandWrapper & setLaytencyMs(double laytency_ms)
  {
    latest_msg.laytency_ms = laytency_ms;
    return *this;
  }

  const crane_msgs::msg::RobotCommand & getMsg() const { return latest_msg; }

  crane_msgs::msg::RobotCommand & getEditableMsg() { return latest_msg; }

  crane_msgs::msg::RobotCommand latest_msg;
};
}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__ROBOT_COMMAND_WRAPPER_HPP_
