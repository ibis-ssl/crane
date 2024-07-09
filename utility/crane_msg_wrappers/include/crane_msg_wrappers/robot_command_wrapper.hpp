// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__ROBOT_COMMAND_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__ROBOT_COMMAND_WRAPPER_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <crane_basics/geometry_operations.hpp>
#include <crane_msgs/msg/robot_command.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "world_model_wrapper.hpp"

namespace crane
{
struct RobotCommandWrapper
{
  typedef std::shared_ptr<RobotCommandWrapper> SharedPtr;

  RobotCommandWrapper(uint8_t id, WorldModelWrapper::SharedPtr world_model_wrapper)
  : robot(world_model_wrapper->getOurRobot(id)), world_model(world_model_wrapper)
  {
    latest_msg.robot_id = id;

    latest_msg.current_pose.x = robot->pose.pos.x();
    latest_msg.current_pose.y = robot->pose.pos.y();
    latest_msg.current_pose.theta = robot->pose.theta;
  }

  virtual ~RobotCommandWrapper() = default;

  RobotCommandWrapper setID(uint8_t id)
  {
    robot = world_model->getOurRobot(id);
    latest_msg.robot_id = id;
    return *this;
  }

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

  RobotCommandWrapper & setTargetTheta(double theta)
  {
    latest_msg.target_theta = theta;
    return *this;
  }

  RobotCommandWrapper & stopHere()
  {
    latest_msg.stop_flag = true;
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

  RobotCommandWrapper & disableBallAvoidance()
  {
    latest_msg.local_planner_config.disable_ball_avoidance = true;
    return *this;
  }

  RobotCommandWrapper & enableBallAvoidance()
  {
    latest_msg.local_planner_config.disable_ball_avoidance = false;
    return *this;
  }

  RobotCommandWrapper & disableRuleAreaAvoidance()
  {
    latest_msg.local_planner_config.disable_rule_area_avoidance = true;
    return *this;
  }

  RobotCommandWrapper & enableRuleAreaAvoidance()
  {
    latest_msg.local_planner_config.disable_rule_area_avoidance = false;
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

  RobotCommandWrapper & setMaxVelocity(double max_velocity)
  {
    latest_msg.local_planner_config.max_velocity = max_velocity;
    return *this;
  }

  RobotCommandWrapper & setMaxAcceleration(double max_acceleration)
  {
    latest_msg.local_planner_config.max_acceleration = max_acceleration;
    return *this;
  }

  RobotCommandWrapper & setMaxOmega(double max_omega)
  {
    latest_msg.local_planner_config.max_omega = max_omega;
    return *this;
  }

  RobotCommandWrapper & setTerminalVelocity(double terminal_velocity)
  {
    latest_msg.local_planner_config.terminal_velocity = terminal_velocity;
    return *this;
  }

  RobotCommandWrapper & stopEmergency(bool flag = true)
  {
    latest_msg.stop_flag = flag;
    return *this;
  }

  RobotCommandWrapper & liftUpDribbler(bool flag = true)
  {
    latest_msg.lift_up_dribbler_flag = flag;
    return *this;
  }

  RobotCommandWrapper & setLatencyMs(double latency_ms)
  {
    latest_msg.latency_ms = latency_ms;
    return *this;
  }

  RobotCommandWrapper & lookAt(Point pos)
  {
    return setTargetTheta(getAngle(pos - robot->pose.pos));
  }

  RobotCommandWrapper & lookAtBall() { return lookAt(world_model->ball.pos); }

  RobotCommandWrapper & lookAtBallFrom(Point from)
  {
    return lookAtFrom(world_model->ball.pos, from);
  }

  RobotCommandWrapper & lookAtFrom(Point at, Point from)
  {
    return setTargetTheta(getAngle(at - from));
  }

  const crane_msgs::msg::RobotCommand & getMsg() const { return latest_msg; }

  crane_msgs::msg::RobotCommand & getEditableMsg() { return latest_msg; }

  crane_msgs::msg::RobotCommand latest_msg;

  std::shared_ptr<RobotInfo> robot;

  WorldModelWrapper::SharedPtr world_model;
};

struct RobotCommandWrapperPosition : public RobotCommandWrapper
{
  typedef std::shared_ptr<RobotCommandWrapperPosition> SharedPtr;

  RobotCommandWrapperPosition(uint8_t id, WorldModelWrapper::SharedPtr world_model_wrapper)
  : RobotCommandWrapper(id, world_model_wrapper)
  {
    latest_msg.control_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
  }

  RobotCommandWrapperPosition & setTargetPosition(double x, double y, double theta)
  {
    latest_msg.target_theta = theta;

    return setTargetPosition(x, y);
  }

  RobotCommandWrapperPosition & setTargetPosition(double x, double y)
  {
    latest_msg.control_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
    if (latest_msg.position_target_mode.empty()) {
      latest_msg.position_target_mode.emplace_back();
    }

    latest_msg.position_target_mode.front().target_x = x;
    latest_msg.position_target_mode.front().target_y = y;

    return *this;
  }

  RobotCommandWrapperPosition & setDribblerTargetPosition(Point position)
  {
    return setTargetPosition(position - robot->center_to_kicker());
  }

  RobotCommandWrapperPosition & setTargetPosition(Point position)
  {
    return setTargetPosition(position.x(), position.y());
  }

  RobotCommandWrapperPosition & setTargetPosition(Point position, double theta)
  {
    return setTargetPosition(position.x(), position.y(), theta);
  }
};

struct RobotCommandWrapperSimpleVelocity : public RobotCommandWrapper
{
  typedef std::shared_ptr<RobotCommandWrapperSimpleVelocity> SharedPtr;

  RobotCommandWrapperSimpleVelocity(uint8_t id, WorldModelWrapper::SharedPtr world_model_wrapper)
  : RobotCommandWrapper(id, world_model_wrapper)
  {
    latest_msg.control_mode = crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE;
  }

  RobotCommandWrapperSimpleVelocity & setVelocity(Velocity velocity)
  {
    return setVelocity(velocity.x(), velocity.y());
  }

  RobotCommandWrapperSimpleVelocity & setVelocity(double x, double y)
  {
    latest_msg.control_mode = crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE;
    if (latest_msg.simple_velocity_target_mode.empty()) {
      latest_msg.simple_velocity_target_mode.emplace_back();
    }
    latest_msg.simple_velocity_target_mode.front().target_vx = x;
    latest_msg.simple_velocity_target_mode.front().target_vy = y;
    return *this;
  }
};
}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__ROBOT_COMMAND_WRAPPER_HPP_
