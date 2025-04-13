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
  using SharedPtr = std::shared_ptr<RobotCommandWrapperBase>;

  RobotCommandWrapperBase(
    std::string skill_name, uint8_t id, WorldModelWrapper::SharedPtr world_model_wrapper)
  : robot(world_model_wrapper->getOurRobot(id)), world_model(world_model_wrapper)
  {
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

class RobotCommandWrapper
{
public:
  using SharedPtr = std::shared_ptr<RobotCommandWrapper>;

private:
  RobotCommandWrapperBase::SharedPtr command;

  // 現在のモード
  uint8_t current_mode;

public:
  RobotCommandWrapper(
    std::string skill_name, uint8_t id, WorldModelWrapper::SharedPtr world_model_wrapper)
  : command(std::make_shared<RobotCommandWrapperBase>(skill_name, id, world_model_wrapper)),
    current_mode(crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE)
  {
    // デフォルトでは位置モードを使用
    usePositionMode();
  }

  // モード切替関数
  RobotCommandWrapper & usePositionMode()
  {
    command->latest_msg.control_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
    command->latest_msg.local_camera_mode.clear();
    command->latest_msg.position_target_mode.clear();
    command->latest_msg.simple_velocity_target_mode.clear();
    command->latest_msg.polar_velocity_target_mode.clear();
    command->latest_msg.position_target_mode.emplace_back();
    current_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
    return *this;
  }

  RobotCommandWrapper & usePolarVelocityMode()
  {
    command->latest_msg.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
    command->latest_msg.local_camera_mode.clear();
    command->latest_msg.position_target_mode.clear();
    command->latest_msg.simple_velocity_target_mode.clear();
    command->latest_msg.polar_velocity_target_mode.clear();
    command->latest_msg.polar_velocity_target_mode.emplace_back();
    current_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
    return *this;
  }

  // 現在のモードを返す
  uint8_t getCurrentMode() const { return current_mode; }

  // メッセージを取得
  const crane_msgs::msg::RobotCommand & getMsg() const { return command->latest_msg; }

  crane_msgs::msg::RobotCommand & getEditableMsg() { return command->latest_msg; }

  const std::shared_ptr<RobotInfo> getRobot() const { return command->robot; }

  auto getWorldModel() const -> WorldModelWrapper::SharedPtr { return command->world_model; }

  // ===== 位置操作関数 =====

  // ===== 共通操作関数 =====

  RobotCommandWrapper & changeID(uint8_t id)
  {
    command->changeID(id);
    return *this;
  }

  RobotCommandWrapper & kickWithChip(double power)
  {
    command->latest_msg.chip_enable = true;
    command->latest_msg.kick_power = power;
    return *this;
  }

  RobotCommandWrapper & kickStraight(double power)
  {
    command->latest_msg.chip_enable = false;
    command->latest_msg.kick_power = power;
    return *this;
  }

  RobotCommandWrapper & dribble(double power)
  {
    command->latest_msg.dribble_power = power;
    command->latest_msg.kick_power = 0.0;
    return *this;
  }

  RobotCommandWrapper & withDribble(double power)
  {
    command->latest_msg.dribble_power = power;
    return *this;
  }

  RobotCommandWrapper & setTargetTheta(double theta, double tolerance = 0.0)
  {
    command->latest_msg.target_theta = theta;
    command->latest_msg.local_planner_config.theta_tolerance = tolerance;
    return *this;
  }

  RobotCommandWrapper & setThetaTolerance(double tolerance)
  {
    command->latest_msg.local_planner_config.theta_tolerance = tolerance;
    return *this;
  }

  // 停止関数（現在のモードに応じた適切な停止を実行）
  RobotCommandWrapper & stopHere()
  {
    switch (current_mode) {
      case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE:
        return setTargetPosition(command->robot->pose.pos)
          .setTargetTheta(command->robot->pose.theta)
          .setOmegaLimit(0.);
      case crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE:
        return setVelocityNorm(0.);
      default:
        // 不明なモードの場合は位置モードで停止
        usePositionMode();
        return setTargetPosition(command->robot->pose.pos)
          .setTargetTheta(command->robot->pose.theta)
          .setOmegaLimit(0.);
    }
  }

  RobotCommandWrapper & disablePlacementAvoidance()
  {
    command->latest_msg.local_planner_config.disable_placement_avoidance = true;
    return *this;
  }

  RobotCommandWrapper & enablePlacementAvoidance()
  {
    command->latest_msg.local_planner_config.disable_placement_avoidance = false;
    return *this;
  }

  RobotCommandWrapper & disableCollisionAvoidance()
  {
    command->latest_msg.local_planner_config.disable_collision_avoidance = true;
    return *this;
  }

  RobotCommandWrapper & enableCollisionAvoidance()
  {
    command->latest_msg.local_planner_config.disable_collision_avoidance = false;
    return *this;
  }

  RobotCommandWrapper & disableGoalAreaAvoidance()
  {
    command->latest_msg.local_planner_config.disable_goal_area_avoidance = true;
    return *this;
  }

  RobotCommandWrapper & enableGoalAreaAvoidance()
  {
    command->latest_msg.local_planner_config.disable_goal_area_avoidance = false;
    return *this;
  }

  RobotCommandWrapper & disableBallAvoidance()
  {
    command->latest_msg.local_planner_config.disable_ball_avoidance = true;
    return *this;
  }

  RobotCommandWrapper & enableBallAvoidance()
  {
    command->latest_msg.local_planner_config.disable_ball_avoidance = false;
    return *this;
  }

  RobotCommandWrapper & disableRuleAreaAvoidance()
  {
    command->latest_msg.local_planner_config.disable_rule_area_avoidance = true;
    return *this;
  }

  RobotCommandWrapper & enableRuleAreaAvoidance()
  {
    command->latest_msg.local_planner_config.disable_rule_area_avoidance = false;
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
    command->latest_msg.enable_ball_centering_control = true;
    return *this;
  }

  RobotCommandWrapper & enableLocalGoalie()
  {
    command->latest_msg.local_goalie_enable = true;
    return *this;
  }

  RobotCommandWrapper & setMaxVelocity(double max_velocity)
  {
    command->latest_msg.local_planner_config.max_velocity = max_velocity;
    return *this;
  }

  RobotCommandWrapper & setMaxAcceleration(double max_acceleration)
  {
    command->latest_msg.local_planner_config.max_acceleration = max_acceleration;
    return *this;
  }

  RobotCommandWrapper & setOmegaLimit(double omega_limit)
  {
    command->latest_msg.omega_limit = omega_limit;
    return *this;
  }

  RobotCommandWrapper & setTerminalVelocity(double terminal_velocity)
  {
    command->latest_msg.local_planner_config.terminal_velocity = terminal_velocity;
    return *this;
  }

  RobotCommandWrapper & stopEmergency(bool flag = true)
  {
    command->latest_msg.stop_flag = flag;
    return *this;
  }

  RobotCommandWrapper & liftUpDribbler(bool flag = true)
  {
    command->latest_msg.lift_up_dribbler_flag = flag;
    return *this;
  }

  RobotCommandWrapper & setLatencyMs(double latency_ms)
  {
    command->latest_msg.latency_ms = latency_ms;
    return *this;
  }

  RobotCommandWrapper & lookAt(Point pos, double tolerance = 0.0)
  {
    return setTargetTheta(getAngle(pos - command->robot->pose.pos), tolerance);
  }

  RobotCommandWrapper & lookAtBall(double tolerance = 0.0)
  {
    return lookAt(command->world_model->ball.pos, tolerance);
  }

  RobotCommandWrapper & lookAtBallFrom(Point from, double tolerance = 0.0)
  {
    return lookAtFrom(command->world_model->ball.pos, from, tolerance);
  }

  RobotCommandWrapper & lookAtFrom(Point at, Point from, double tolerance = 0.0)
  {
    return setTargetTheta(getAngle(at - from), tolerance);
  }

  void addStateFactor(const std::string & name, const std::string & state)
  {
    // 同じnameのものが存在しなければ追加。存在すれば、更新
    if (auto state_factor = ranges::find_if(
          command->latest_msg.state_factors,
          [name](const auto & state_factor) { return state_factor.name == name; });
        state_factor == command->latest_msg.state_factors.end() || state_factor->state != state) {
      crane_msgs::msg::StateFactor msg;
      msg.name = name;
      msg.state = state;
      command->latest_msg.state_factors.emplace_back(msg);
    }
  }

  void clearSkillStates() { command->latest_msg.state_factors.clear(); }

  // ===== PositionTargetMode固有の関数 =====

  RobotCommandWrapper & setTargetPosition(double x, double y, double tolerance = 0.0)
  {
    // 必要に応じてモードを切り替え
    if (current_mode != crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE) {
      usePositionMode();
    }

    command->latest_msg.position_target_mode.front().target_x = x;
    command->latest_msg.position_target_mode.front().target_y = y;
    command->latest_msg.position_target_mode.front().position_tolerance = tolerance;

    return *this;
  }

  RobotCommandWrapper & setTargetPosition(Point position, double tolerance = 0.0)
  {
    return setTargetPosition(position.x(), position.y(), tolerance);
  }

  RobotCommandWrapper & setDribblerTargetPosition(Point position, double tolerance = 0.0)
  {
    // 必要に応じてモードを切り替え
    if (current_mode != crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE) {
      usePositionMode();
    }

    double theta = command->latest_msg.target_theta;
    return setTargetPosition(
      position + getNormVec(theta + M_PI) * getRobot()->getDribblerDistance(), tolerance);
  }

  double getTargetDistance()
  {
    if (current_mode != crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE) {
    return std::hypot(
      command->latest_msg.position_target_mode.front().target_x - command->robot->pose.pos.x(),
      command->latest_msg.position_target_mode.front().target_y - command->robot->pose.pos.y());
  }else{
    return 0.;
}
  }

  // ===== PolarVelocityTargetMode固有の関数 =====

  RobotCommandWrapper & setVelocity(Velocity velocity)
  {
    return setVelocityNorm(velocity.norm()).setVelocityAngle(getAngle(velocity));
  }

  RobotCommandWrapper & setVelocity(double x, double y) { return setVelocity({x, y}); }

  RobotCommandWrapper & setVelocityNorm(double r)
  {
    // 必要に応じてモードを切り替え
    if (current_mode != crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE) {
      usePolarVelocityMode();
    }

    command->latest_msg.polar_velocity_target_mode.front().target_velocity_r = r;
    return *this;
  }

  RobotCommandWrapper & setVelocityAngle(double theta)
  {
    // 必要に応じてモードを切り替え
    if (current_mode != crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE) {
      usePolarVelocityMode();
    }

    command->latest_msg.polar_velocity_target_mode.front().target_velocity_theta = theta;
    return *this;
  }
};
}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__ROBOT_COMMAND_WRAPPER_HPP_
