// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__ROBOT_COMMAND_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__ROBOT_COMMAND_WRAPPER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_msgs/msg/robot_command.hpp>
#include <crane_physics/kicker_model.hpp>
#include <crane_physics/pid_controller.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "delay_monitor_wrapper.hpp"
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

  // キッカーモデル（停止距離指定キック用）
  std::shared_ptr<KickerModel> kicker_model;

  // 現在のモード
  uint8_t current_mode;

  auto getID() const -> uint8_t { return latest_msg.robot_id; }

public:
  const std::string name;

  RobotCommandWrapper(
    std::string skill_name, uint8_t id, WorldModelWrapper::SharedPtr world_model_wrapper)
  : robot(world_model_wrapper->getOurRobot(id)),
    world_model(world_model_wrapper),
    kicker_model(nullptr),
    current_mode(crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE),
    name(skill_name)
  {
    changeID(id);
    // デフォルトでは位置モードを使用
    usePositionMode();
  }

  // モード切替関数
  auto usePositionMode() -> RobotCommandWrapper &
  {
    latest_msg.control_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
    latest_msg.local_camera_mode.clear();
    latest_msg.position_target_mode.clear();
    latest_msg.simple_velocity_target_mode.clear();
    latest_msg.polar_velocity_target_mode.clear();
    latest_msg.position_target_mode.emplace_back();
    latest_msg.local_planner_config.max_velocity_factors.clear();
    latest_msg.local_planner_config.max_acceleration_factors.clear();
    current_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
    return *this;
  }

  auto usePolarVelocityMode() -> RobotCommandWrapper &
  {
    latest_msg.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
    latest_msg.local_camera_mode.clear();
    latest_msg.position_target_mode.clear();
    latest_msg.simple_velocity_target_mode.clear();
    latest_msg.polar_velocity_target_mode.clear();
    latest_msg.polar_velocity_target_mode.emplace_back();
    latest_msg.local_planner_config.max_velocity_factors.clear();
    latest_msg.local_planner_config.max_acceleration_factors.clear();
    current_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
    return *this;
  }

  // 現在のモードを返す
  auto getCurrentMode() const -> uint8_t { return current_mode; }

  // メッセージを取得
  auto getMsg() const -> const crane_msgs::msg::RobotCommand & { return latest_msg; }

  auto getEditableMsg() -> crane_msgs::msg::RobotCommand & { return latest_msg; }

  auto getRobot() const -> std::shared_ptr<RobotInfo> { return robot; }

  auto getWorldModel() const -> WorldModelWrapper::SharedPtr { return world_model; }

  // ===== 位置操作関数 =====

  // ===== 共通操作関数 =====
  auto changeID(uint8_t id) -> RobotCommandWrapper &
  {
    robot = world_model->getOurRobot(id);
    latest_msg.robot_id = id;
    latest_msg.current_pose.x = robot->pose.pos.x();
    latest_msg.current_pose.y = robot->pose.pos.y();
    latest_msg.current_pose.theta = robot->pose.theta;
    return *this;
  }

  auto kickWithChip(double power) -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.kick_power_override = false;
    latest_msg.chip_enable = true;
    latest_msg.kick_power = power;
    return *this;
  }

  auto kickStraight(double power) -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.kick_power_override = false;
    latest_msg.chip_enable = false;
    latest_msg.kick_power = power;
    return *this;
  }

  auto setKickStraightTargetSpeed(double speed_mps) -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.kick_power_override = true;
    latest_msg.chip_enable = false;
    latest_msg.local_planner_config.target_kick_ball_speed = speed_mps;
    return *this;
  }

  auto setKickWithChipTargetDistance(double distance) -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.kick_power_override = true;
    latest_msg.chip_enable = true;
    latest_msg.local_planner_config.target_chip_distance = distance;
    return *this;
  }

  auto kickStraightToStopAt(double stop_distance) -> RobotCommandWrapper &
  {
    if (!kicker_model) {
      throw std::runtime_error(
        "KickerModelが設定されていません。setKickerModelを呼び出してください。");
    }

    double kick_power = kicker_model->calculateKickPowerForStopDistance(stop_distance);
    latest_msg.local_planner_config.kick_power_override = false;
    latest_msg.chip_enable = false;
    latest_msg.kick_power = kick_power;
    return *this;
  }

  auto kickStraightWithInitialSpeed(double initial_speed) -> RobotCommandWrapper &
  {
    if (!kicker_model) {
      throw std::runtime_error(
        "KickerModelが設定されていません。setKickerModelを呼び出してください。");
    }

    double kick_power = kicker_model->calculateStraightKickPower(initial_speed);
    latest_msg.local_planner_config.kick_power_override = false;
    latest_msg.chip_enable = false;
    latest_msg.kick_power = kick_power;
    return *this;
  }

  auto predictStraightKickStopDistance(double initial_speed) const -> double
  {
    if (!kicker_model) {
      throw std::runtime_error(
        "KickerModelが設定されていません。setKickerModelを呼び出してください。");
    }

    double kick_power = kicker_model->calculateStraightKickPower(initial_speed);
    return kicker_model->predictStopDistance(kick_power);
  }

  auto dribble(double power) -> RobotCommandWrapper &
  {
    latest_msg.dribble_power = power;
    latest_msg.kick_power = 0.0;
    return *this;
  }

  auto withDribble(double power) -> RobotCommandWrapper &
  {
    latest_msg.dribble_power = power;
    return *this;
  }

  auto setTargetTheta(double theta, double tolerance = 0.0) -> RobotCommandWrapper &
  {
    latest_msg.target_theta = theta;
    latest_msg.local_planner_config.theta_tolerance = tolerance;
    return *this;
  }

  auto setThetaTolerance(double tolerance) -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.theta_tolerance = tolerance;
    return *this;
  }

  // 停止関数（現在のモードに応じた適切な停止を実行）
  auto stopHere() -> RobotCommandWrapper &
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

  auto disablePlacementAvoidance() -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.disable_placement_avoidance = true;
    return *this;
  }

  auto enablePlacementAvoidance() -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.disable_placement_avoidance = false;
    return *this;
  }

  auto disableCollisionAvoidance() -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.disable_collision_avoidance = true;
    return *this;
  }

  auto enableCollisionAvoidance() -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.disable_collision_avoidance = false;
    return *this;
  }

  auto disableGoalAreaAvoidance() -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.disable_goal_area_avoidance = true;
    return *this;
  }

  auto enableGoalAreaAvoidance() -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.disable_goal_area_avoidance = false;
    return *this;
  }

  auto disableBallAvoidance() -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.disable_ball_avoidance = true;
    return *this;
  }

  auto enableBallAvoidance() -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.disable_ball_avoidance = false;
    return *this;
  }

  auto enableRotationStopOnAccel() -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.enable_rotation_stop_on_accel = true;
    return *this;
  }

  auto disableRotationStopOnAccel() -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.enable_rotation_stop_on_accel = false;
    return *this;
  }

  auto disableAnyAreaAvoidance() -> RobotCommandWrapper &
  {
    return disableGoalAreaAvoidance().disableBallAvoidance().disablePlacementAvoidance();
  }

  auto enableAnyAreaAvoidance() -> RobotCommandWrapper &
  {
    return enableGoalAreaAvoidance().enableBallAvoidance().enablePlacementAvoidance();
  }

  auto setGoalieDefault() -> RobotCommandWrapper &
  {
    disableCollisionAvoidance();
    disableGoalAreaAvoidance();
    return *this;
  }

  auto enableBallCenteringControl() -> RobotCommandWrapper &
  {
    latest_msg.enable_ball_centering_control = true;
    return *this;
  }

  auto enableLocalGoalie() -> RobotCommandWrapper &
  {
    latest_msg.local_goalie_enable = true;
    return *this;
  }

  auto setMaxVelocity(const std::string & factor_name, double max_velocity) -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.max_velocity_factors.emplace_back(
      crane_msgs::msg::NamedFloat().set__name(factor_name).set__value(max_velocity));
    return *this;
  }

  auto clearMaxVelocityFactors() -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.max_velocity_factors.clear();
    return *this;
  }

  auto setMaxAcceleration(const std::string & factor_name, double max_acceleration)
    -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.max_acceleration_factors.emplace_back(
      crane_msgs::msg::NamedFloat().set__name(factor_name).set__value(max_acceleration));
    return *this;
  }

  auto clearMaxAccelerationFactors() -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.max_acceleration_factors.clear();
    return *this;
  }

  auto setOmegaLimit(double omega_limit) -> RobotCommandWrapper &
  {
    latest_msg.omega_limit = omega_limit;
    return *this;
  }

  auto setTerminalVelocity(double terminal_velocity) -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.terminal_velocity = terminal_velocity;
    return *this;
  }

  auto stopEmergency(bool flag = true) -> RobotCommandWrapper &
  {
    latest_msg.stop_flag = flag;
    return *this;
  }

  // auto setLatencyMs(double latency_ms) -> RobotCommandWrapper &
  // {
  //   latest_msg.latency_ms = latency_ms;
  //   return *this;
  // }

  auto lookAt(Point pos, double tolerance = 0.0) -> RobotCommandWrapper &
  {
    return setTargetTheta(getAngle(pos - robot->pose.pos), tolerance);
  }

  auto lookAtBall(double tolerance = 0.0) -> RobotCommandWrapper &
  {
    return lookAt(world_model->ball().pos, tolerance);
  }

  auto lookAtBallFrom(Point from, double tolerance = 0.0) -> RobotCommandWrapper &
  {
    return lookAtFrom(world_model->ball().pos, from, tolerance);
  }

  auto lookAtFrom(Point at, Point from, double tolerance = 0.0) -> RobotCommandWrapper &
  {
    return setTargetTheta(getAngle(at - from), tolerance);
  }

  auto addStateFactor(const std::string & name, const std::string & state) -> void
  {
    // 同じnameのものが存在しなければ追加。存在すれば、更新
    if (auto state_factor = ranges::find_if(
          latest_msg.state_factors,
          [name](const auto & state_factor) { return state_factor.name == name; });
        state_factor == latest_msg.state_factors.end() || state_factor->value != state) {
      crane_msgs::msg::NamedString msg;
      msg.name = name;
      msg.value = state;
      latest_msg.state_factors.emplace_back(msg);
    }
  }

  auto clearSkillStates() -> void { latest_msg.state_factors.clear(); }

  // ===== 遅延監視関連メソッド =====

  auto addDelayCheckpoint(const std::string & name, const std::string & value = "") -> void
  {
    DelayMonitorWrapper::addDelayCheckpoint(latest_msg.delay_checkpoints, name, value);
  }

  auto clearDelayCheckpoints() -> void
  {
    DelayMonitorWrapper::clearCheckpoints(latest_msg.delay_checkpoints);
  }

  auto calculateDelayMs(const std::string & start_name, const std::string & end_name) -> double
  {
    return DelayMonitorWrapper::calculateDelayMs(
      latest_msg.delay_checkpoints, start_name, end_name);
  }

  auto calculateTotalDelayMs(const std::string & end_name) -> double
  {
    return DelayMonitorWrapper::calculateTotalDelayMs(latest_msg.delay_checkpoints, end_name);
  }

  auto getDelayCheckpointsString() -> std::string
  {
    return DelayMonitorWrapper::checkpointsToString(latest_msg.delay_checkpoints);
  }

  auto mergeDelayCheckpoints(const crane_msgs::msg::DelayCheckpoints & source_checkpoints) -> void
  {
    DelayMonitorWrapper::mergeCheckpoints(latest_msg.delay_checkpoints, source_checkpoints);
  }

  // ===== KickerModel管理メソッド =====

  auto setKickerModel(std::shared_ptr<KickerModel> kicker) -> RobotCommandWrapper &
  {
    kicker_model = kicker;
    return *this;
  }

  auto getKickerModel() const -> std::shared_ptr<KickerModel> { return kicker_model; }

  // ===== PositionTargetMode固有の関数 =====

  auto setTargetPosition(double x, double y, double tolerance = 0.0) -> RobotCommandWrapper &
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

  auto setTargetPosition(Point position, double tolerance = 0.0) -> RobotCommandWrapper &
  {
    return setTargetPosition(position.x(), position.y(), tolerance);
  }

  auto setDribblerTargetPosition(Point position, double tolerance = 0.0) -> RobotCommandWrapper &
  {
    double theta = latest_msg.target_theta;
    return setTargetPosition(
      position + getNormVec(theta + M_PI) * getRobot()->getDribblerDistance(), tolerance);
  }

  auto getTargetDistance() -> double
  {
    if (current_mode == crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE) {
      return std::hypot(
        latest_msg.position_target_mode.front().target_x - robot->pose.pos.x(),
        latest_msg.position_target_mode.front().target_y - robot->pose.pos.y());
    } else {
      return 0.;
    }
  }

  // ===== PolarVelocityTargetMode固有の関数 =====

  auto setVelocity(Velocity velocity) -> RobotCommandWrapper &
  {
    return setVelocityNorm(velocity.norm()).setVelocityAngle(getAngle(velocity));
  }

  auto setVelocity(double x, double y) -> RobotCommandWrapper & { return setVelocity({x, y}); }

  auto setVelocityNorm(double r) -> RobotCommandWrapper &
  {
    // 必要に応じてモードを切り替え
    if (current_mode != crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE) {
      usePolarVelocityMode();
    }

    latest_msg.polar_velocity_target_mode.front().target_velocity_r = r;
    return *this;
  }

  auto setVelocityAngle(double theta) -> RobotCommandWrapper &
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
