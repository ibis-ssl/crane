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
class RobotCommandWrapper
{
public:
  using SharedPtr = std::shared_ptr<RobotCommandWrapper>;

private:
  crane_msgs::msg::RobotCommand latest_msg;

  std::shared_ptr<RobotInfo> robot;

  WorldModelWrapper::SharedPtr world_model;

  // 現在のモード
  uint8_t current_mode;

  uint8_t getID() const { return latest_msg.robot_id; }

public:
  const std::string name;

  RobotCommandWrapper(
    std::string skill_name, uint8_t id, WorldModelWrapper::SharedPtr world_model_wrapper)
  : robot(world_model_wrapper->getOurRobot(id)),
    world_model(world_model_wrapper),
    current_mode(crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE),
    name(skill_name)
  {
    changeID(id);
    // デフォルトでは位置モードを使用
    usePositionMode();
  }

  // モード切替関数
  RobotCommandWrapper & usePositionMode()
  {
    latest_msg.control_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
    latest_msg.local_camera_mode.clear();
    latest_msg.position_target_mode.clear();
    latest_msg.simple_velocity_target_mode.clear();
    latest_msg.polar_velocity_target_mode.clear();
    latest_msg.position_target_mode.emplace_back();
    current_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
    return *this;
  }

  RobotCommandWrapper & usePolarVelocityMode()
  {
    latest_msg.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
    latest_msg.local_camera_mode.clear();
    latest_msg.position_target_mode.clear();
    latest_msg.simple_velocity_target_mode.clear();
    latest_msg.polar_velocity_target_mode.clear();
    latest_msg.polar_velocity_target_mode.emplace_back();
    current_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
    return *this;
  }

  // 現在のモードを返す
  uint8_t getCurrentMode() const { return current_mode; }

  // メッセージを取得
  const crane_msgs::msg::RobotCommand & getMsg() const { return latest_msg; }

  crane_msgs::msg::RobotCommand & getEditableMsg() { return latest_msg; }

  const std::shared_ptr<RobotInfo> getRobot() const { return robot; }

  auto getWorldModel() const -> WorldModelWrapper::SharedPtr { return world_model; }

  // ===== 位置操作関数 =====

  // ===== 共通操作関数 =====
  RobotCommandWrapper & changeID(uint8_t id)
  {
    robot = world_model->getOurRobot(id);
    latest_msg.robot_id = id;
    latest_msg.current_pose.x = robot->pose.pos.x();
    latest_msg.current_pose.y = robot->pose.pos.y();
    latest_msg.current_pose.theta = robot->pose.theta;
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

  RobotCommandWrapper & withDribble(double power)
  {
    latest_msg.dribble_power = power;
    return *this;
  }

  RobotCommandWrapper & setTargetTheta(double theta, double tolerance = 0.0)
  {
    latest_msg.target_theta = theta;
    latest_msg.local_planner_config.theta_tolerance = tolerance;
    return *this;
  }

  RobotCommandWrapper & setThetaTolerance(double tolerance)
  {
    latest_msg.local_planner_config.theta_tolerance = tolerance;
    return *this;
  }

  // 停止関数（現在のモードに応じた適切な停止を実行）
  RobotCommandWrapper & stopHere()
  {
    switch (current_mode) {
      case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE:
        return setTargetPosition(robot->pose.pos)
          .setTargetTheta(robot->pose.theta)
          .setOmegaLimit(0.);
      case crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE:
        return setVelocityNorm(0.);
      default:
        // 不明なモードの場合は位置モードで停止
        usePositionMode();
        return setTargetPosition(robot->pose.pos)
          .setTargetTheta(robot->pose.theta)
          .setOmegaLimit(0.);
    }
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

  RobotCommandWrapper & disableAnyAreaAvoidance()
  {
    return disableGoalAreaAvoidance()..disableBallAvoidance().disablePlacementAvoidance();
  }

  RobotCommandWrapper & enableAnyAreaAvoidance()
  {
    return enableGoalAreaAvoidance()..enableBallAvoidance().enablePlacementAvoidance();
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

  RobotCommandWrapper & setOmegaLimit(double omega_limit)
  {
    latest_msg.omega_limit = omega_limit;
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

  RobotCommandWrapper & lookAt(Point pos, double tolerance = 0.0)
  {
    return setTargetTheta(getAngle(pos - robot->pose.pos), tolerance);
  }

  RobotCommandWrapper & lookAtBall(double tolerance = 0.0)
  {
    return lookAt(world_model->ball.pos, tolerance);
  }

  RobotCommandWrapper & lookAtBallFrom(Point from, double tolerance = 0.0)
  {
    return lookAtFrom(world_model->ball.pos, from, tolerance);
  }

  RobotCommandWrapper & lookAtFrom(Point at, Point from, double tolerance = 0.0)
  {
    return setTargetTheta(getAngle(at - from), tolerance);
  }

  void addStateFactor(const std::string & name, const std::string & state)
  {
    // 同じnameのものが存在しなければ追加。存在すれば、更新
    if (auto state_factor = ranges::find_if(
          latest_msg.state_factors,
          [name](const auto & state_factor) { return state_factor.name == name; });
        state_factor == latest_msg.state_factors.end() || state_factor->state != state) {
      crane_msgs::msg::StateFactor msg;
      msg.name = name;
      msg.state = state;
      latest_msg.state_factors.emplace_back(msg);
    }
  }

  void clearSkillStates() { latest_msg.state_factors.clear(); }

  // ===== PositionTargetMode固有の関数 =====

  RobotCommandWrapper & setTargetPosition(double x, double y, double tolerance = 0.0)
  {
    // 必要に応じてモードを切り替え
    if (current_mode != crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE) {
      usePositionMode();
    }

    latest_msg.position_target_mode.front().target_x = x;
    latest_msg.position_target_mode.front().target_y = y;
    latest_msg.position_target_mode.front().position_tolerance = tolerance;

    return *this;
  }

  RobotCommandWrapper & setTargetPosition(Point position, double tolerance = 0.0)
  {
    return setTargetPosition(position.x(), position.y(), tolerance);
  }

  RobotCommandWrapper & setDribblerTargetPosition(Point position, double tolerance = 0.0)
  {
    double theta = latest_msg.target_theta;
    return setTargetPosition(
      position + getNormVec(theta + M_PI) * getRobot()->getDribblerDistance(), tolerance);
  }

  double getTargetDistance()
  {
    if (current_mode != crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE) {
      return std::hypot(
        latest_msg.position_target_mode.front().target_x - robot->pose.pos.x(),
        latest_msg.position_target_mode.front().target_y - robot->pose.pos.y());
    } else {
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

    latest_msg.polar_velocity_target_mode.front().target_velocity_r = r;
    return *this;
  }

  RobotCommandWrapper & setVelocityAngle(double theta)
  {
    // 必要に応じてモードを切り替え
    if (current_mode != crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE) {
      usePolarVelocityMode();
    }

    latest_msg.polar_velocity_target_mode.front().target_velocity_theta = theta;
    return *this;
  }
};
}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__ROBOT_COMMAND_WRAPPER_HPP_
