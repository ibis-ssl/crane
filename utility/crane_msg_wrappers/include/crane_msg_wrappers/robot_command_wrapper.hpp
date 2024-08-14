// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__ROBOT_COMMAND_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__ROBOT_COMMAND_WRAPPER_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <crane_basics/geometry_operations.hpp>
#include <crane_basics/pid_controller.hpp>
#include <crane_msgs/msg/robot_command.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "world_model_wrapper.hpp"

namespace crane
{
struct RobotCommandWrapperBase
{
  typedef std::shared_ptr<RobotCommandWrapperBase> SharedPtr;

  RobotCommandWrapperBase(
    std::string skill_name, uint8_t id, WorldModelWrapper::SharedPtr world_model_wrapper)
  : robot(world_model_wrapper->getOurRobot(id)), world_model(world_model_wrapper)
  {
    latest_msg.skill_name = skill_name;
    changeID(id);
  }

  void changeID(uint8_t id)
  {
    robot = world_model->getOurRobot(id);
    latest_msg.robot_id = id;
    latest_msg.current_pose.x = robot->pose.pos.x();
    latest_msg.current_pose.y = robot->pose.pos.y();
    latest_msg.current_pose.theta = robot->pose.theta;
  }

  uint8_t getID() const { return latest_msg.robot_id; }

  crane_msgs::msg::RobotCommand latest_msg;

  std::shared_ptr<RobotInfo> robot;

  WorldModelWrapper::SharedPtr world_model;
};

template <typename T>
class RobotCommandWrapperCommon
{
protected:
  RobotCommandWrapperBase::SharedPtr command;

public:
  explicit RobotCommandWrapperCommon(RobotCommandWrapperBase::SharedPtr command) : command(command)
  {
  }

  RobotCommandWrapperCommon(
    std::string skill_name, uint8_t id, WorldModelWrapper::SharedPtr world_model_wrapper)
  : command(std::make_shared<RobotCommandWrapperBase>(skill_name, id, world_model_wrapper))
  {
  }

  const crane_msgs::msg::RobotCommand & getMsg() const { return command->latest_msg; }

  crane_msgs::msg::RobotCommand & getEditableMsg() { return command->latest_msg; }

  const std::shared_ptr<RobotInfo> getRobot() const { return command->robot; }

  T & changeID(uint8_t id)
  {
    command->robot = command->world_model->getOurRobot(id);
    command->latest_msg.robot_id = id;
    return static_cast<T &>(*this);
  }

  T & kickWithChip(double power)
  {
    command->latest_msg.chip_enable = true;
    command->latest_msg.kick_power = power;
    return static_cast<T &>(*this);
  }

  T & kickStraight(double power)
  {
    command->latest_msg.chip_enable = false;
    command->latest_msg.kick_power = power;
    return static_cast<T &>(*this);
  }

  T & dribble(double power)
  {
    command->latest_msg.dribble_power = power;
    command->latest_msg.kick_power = 0.0;
    return static_cast<T &>(*this);
  }

  T & setTargetTheta(double theta)
  {
    command->latest_msg.target_theta = theta;
    return static_cast<T &>(*this);
  }

  virtual T & stopHere()
  {
    command->latest_msg.stop_flag = true;
    return static_cast<T &>(*this);
  }

  T & disablePlacementAvoidance()
  {
    command->latest_msg.local_planner_config.disable_placement_avoidance = true;
    return static_cast<T &>(*this);
  }

  T & enablePlacementAvoidance()
  {
    command->latest_msg.local_planner_config.disable_placement_avoidance = false;
    return static_cast<T &>(*this);
  }

  T & disableCollisionAvoidance()
  {
    command->latest_msg.local_planner_config.disable_collision_avoidance = true;
    return static_cast<T &>(*this);
  }

  T & enableCollisionAvoidance()
  {
    command->latest_msg.local_planner_config.disable_collision_avoidance = false;
    return static_cast<T &>(*this);
  }

  T & disableGoalAreaAvoidance()
  {
    command->latest_msg.local_planner_config.disable_goal_area_avoidance = true;
    return static_cast<T &>(*this);
  }

  T & enableGoalAreaAvoidance()
  {
    command->latest_msg.local_planner_config.disable_goal_area_avoidance = false;
    return static_cast<T &>(*this);
  }

  T & disableBallAvoidance()
  {
    command->latest_msg.local_planner_config.disable_ball_avoidance = true;
    return static_cast<T &>(*this);
  }

  T & enableBallAvoidance()
  {
    command->latest_msg.local_planner_config.disable_ball_avoidance = false;
    return static_cast<T &>(*this);
  }

  T & disableRuleAreaAvoidance()
  {
    command->latest_msg.local_planner_config.disable_rule_area_avoidance = true;
    return static_cast<T &>(*this);
  }

  T & enableRuleAreaAvoidance()
  {
    command->latest_msg.local_planner_config.disable_rule_area_avoidance = false;
    return static_cast<T &>(*this);
  }

  T & setGoalieDefault()
  {
    disableCollisionAvoidance();
    disableGoalAreaAvoidance();
    return static_cast<T &>(*this);
  }

  T & enableBallCenteringControl()
  {
    command->latest_msg.enable_ball_centering_control = true;
    return static_cast<T &>(*this);
  }

  T & enableLocalGoalie()
  {
    command->latest_msg.local_goalie_enable = true;
    return static_cast<T &>(*this);
  }

  T & setMaxVelocity(double max_velocity)
  {
    command->latest_msg.local_planner_config.max_velocity = max_velocity;
    return static_cast<T &>(*this);
  }

  T & setMaxAcceleration(double max_acceleration)
  {
    command->latest_msg.local_planner_config.max_acceleration = max_acceleration;
    return static_cast<T &>(*this);
  }

  T & setMaxOmega(double max_omega)
  {
    command->latest_msg.local_planner_config.max_omega = max_omega;
    return static_cast<T &>(*this);
  }

  T & setOmegaLimit(double omega_limit)
  {
    command->latest_msg.omega_limit = omega_limit;
    return static_cast<T &>(*this);
  }

  T & setTerminalVelocity(double terminal_velocity)
  {
    command->latest_msg.local_planner_config.terminal_velocity = terminal_velocity;
    return static_cast<T &>(*this);
  }

  T & stopEmergency(bool flag = true)
  {
    command->latest_msg.stop_flag = flag;
    return static_cast<T &>(*this);
  }

  T & liftUpDribbler(bool flag = true)
  {
    command->latest_msg.lift_up_dribbler_flag = flag;
    return static_cast<T &>(*this);
  }

  T & setLatencyMs(double latency_ms)
  {
    command->latest_msg.latency_ms = latency_ms;
    return static_cast<T &>(*this);
  }

  T & lookAt(Point pos) { return setTargetTheta(getAngle(pos - command->robot->pose.pos)); }

  T & lookAtBall() { return lookAt(command->world_model->ball.pos); }

  T & lookAtBallFrom(Point from) { return lookAtFrom(command->world_model->ball.pos, from); }

  T & lookAtFrom(Point at, Point from) { return setTargetTheta(getAngle(at - from)); }
};

class RobotCommandWrapperPosition : public RobotCommandWrapperCommon<RobotCommandWrapperPosition>
{
public:
  typedef std::shared_ptr<RobotCommandWrapperPosition> SharedPtr;

  explicit RobotCommandWrapperPosition(RobotCommandWrapperBase::SharedPtr & base)
  : RobotCommandWrapperCommon(base)
  {
    command->latest_msg.control_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
    command->latest_msg.local_camera_mode.clear();
    command->latest_msg.position_target_mode.clear();
    command->latest_msg.simple_velocity_target_mode.clear();
    command->latest_msg.velocity_target_mode.clear();
    command->latest_msg.position_target_mode.emplace_back();
  }

  RobotCommandWrapperPosition(
    std::string skill_name, uint8_t id, WorldModelWrapper::SharedPtr world_model_wrapper)
  : RobotCommandWrapperCommon(skill_name, id, world_model_wrapper)
  {
    command->latest_msg.control_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
    command->latest_msg.local_camera_mode.clear();
    command->latest_msg.position_target_mode.clear();
    command->latest_msg.simple_velocity_target_mode.clear();
    command->latest_msg.velocity_target_mode.clear();
    command->latest_msg.position_target_mode.emplace_back();
  }

  RobotCommandWrapperPosition & setTargetPosition(double x, double y, double theta)
  {
    command->latest_msg.control_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
    command->latest_msg.target_theta = theta;

    return setTargetPosition(x, y);
  }

  RobotCommandWrapperPosition & setTargetPosition(double x, double y)
  {
    command->latest_msg.control_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
    if (command->latest_msg.position_target_mode.empty()) {
      command->latest_msg.position_target_mode.emplace_back();
    }

    command->latest_msg.position_target_mode.front().target_x = x;
    command->latest_msg.position_target_mode.front().target_y = y;

    return *this;
  }

  RobotCommandWrapperPosition & setDribblerTargetPosition(Point position)
  {
    double theta = command->latest_msg.target_theta;
    return setDribblerTargetPosition(position, theta);
  }

  RobotCommandWrapperPosition & setDribblerTargetPosition(Point position, double theta)
  {
    return setTargetPosition(
      position + getNormVec(theta + M_PI) * getRobot()->getDribblerDistance(), theta);
  }

  RobotCommandWrapperPosition & setTargetPosition(Point position)
  {
    return setTargetPosition(position.x(), position.y());
  }

  RobotCommandWrapperPosition & setTargetPosition(Point position, double theta)
  {
    return setTargetPosition(position.x(), position.y(), theta);
  }

  RobotCommandWrapperPosition & stopHere() override
  {
    command->latest_msg.stop_flag = true;
    return setTargetPosition(command->robot->pose.pos);
  }
};

class RobotCommandWrapperSimpleVelocity
: public RobotCommandWrapperCommon<RobotCommandWrapperSimpleVelocity>
{
public:
  typedef std::shared_ptr<RobotCommandWrapperSimpleVelocity> SharedPtr;

  explicit RobotCommandWrapperSimpleVelocity(RobotCommandWrapperBase::SharedPtr & base);

  RobotCommandWrapperSimpleVelocity(
    std::string skill_name, uint8_t id, WorldModelWrapper::SharedPtr world_model_wrapper)
  : RobotCommandWrapperCommon(skill_name, id, world_model_wrapper)
  {
    command->latest_msg.control_mode = crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE;
    command->latest_msg.local_camera_mode.clear();
    command->latest_msg.position_target_mode.clear();
    command->latest_msg.simple_velocity_target_mode.clear();
    command->latest_msg.velocity_target_mode.clear();
    command->latest_msg.simple_velocity_target_mode.emplace_back();
  }

  RobotCommandWrapperSimpleVelocity & setVelocity(Velocity velocity)
  {
    return setVelocity(velocity.x(), velocity.y());
  }

  RobotCommandWrapperSimpleVelocity & setVelocity(double x, double y)
  {
    command->latest_msg.control_mode = crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE;
    if (command->latest_msg.simple_velocity_target_mode.empty()) {
      command->latest_msg.simple_velocity_target_mode.emplace_back();
    }
    command->latest_msg.simple_velocity_target_mode.front().target_vx = x;
    command->latest_msg.simple_velocity_target_mode.front().target_vy = y;
    return *this;
  }

  RobotCommandWrapperSimpleVelocity & setTargetPosition(Point target);

protected:
  PIDController x_controller, y_controller;
};
}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__ROBOT_COMMAND_WRAPPER_HPP_
