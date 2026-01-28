// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__POSITION_COMMAND_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__POSITION_COMMAND_WRAPPER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_msgs/msg/position_command.hpp>
#include <crane_physics/kicker_model.hpp>
#include <memory>
#include <range/v3/algorithm/find_if.hpp>
#include <vector>

#include "delay_monitor_wrapper.hpp"
#include "velocity_plan_tracker.hpp"
#include "world_model_wrapper.hpp"

namespace crane
{
/**
 * @brief session_planner → local_planner 用の位置指令ラッパー
 */
class PositionCommandWrapper
{
public:
  using SharedPtr = std::shared_ptr<PositionCommandWrapper>;

private:
  crane_msgs::msg::PositionCommand latest_msg;

  std::shared_ptr<RobotInfo> robot;

  WorldModelWrapper::SharedPtr world_model;

  std::shared_ptr<KickerModel> kicker_model;

  auto getID() const -> uint8_t { return latest_msg.robot_id; }

public:
  const std::string name;

  PositionCommandWrapper(
    std::string skill_name, uint8_t id, WorldModelWrapper::SharedPtr world_model_wrapper)
  : robot(world_model_wrapper->getOurRobot(id)),
    world_model(world_model_wrapper),
    kicker_model(nullptr),
    name(skill_name)
  {
    changeID(id);
  }

  // メッセージを取得
  auto getMsg() const -> const crane_msgs::msg::PositionCommand & { return latest_msg; }

  auto getEditableMsg() -> crane_msgs::msg::PositionCommand & { return latest_msg; }

  auto getRobot() const -> std::shared_ptr<RobotInfo> { return robot; }

  auto getWorldModel() const -> WorldModelWrapper::SharedPtr { return world_model; }

  // ===== 共通操作関数 =====
  auto changeID(uint8_t id) -> PositionCommandWrapper &
  {
    robot = world_model->getOurRobot(id);
    latest_msg.robot_id = id;
    latest_msg.current_pose.x = robot->pose.pos.x();
    latest_msg.current_pose.y = robot->pose.pos.y();
    latest_msg.current_pose.theta = robot->pose.theta;
    return *this;
  }

  auto kickWithChip(double power) -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.kick_power_override = false;
    latest_msg.chip_enable = true;
    latest_msg.kick_power = power;
    return *this;
  }

  auto kickStraight(double power) -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.kick_power_override = false;
    latest_msg.chip_enable = false;
    latest_msg.kick_power = power;
    return *this;
  }

  auto setKickStraightTargetSpeed(double speed_mps) -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.kick_power_override = true;
    latest_msg.chip_enable = false;
    latest_msg.local_planner_config.target_kick_ball_speed = speed_mps;
    return *this;
  }

  auto setKickWithChipTargetDistance(double distance) -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.kick_power_override = true;
    latest_msg.chip_enable = true;
    latest_msg.local_planner_config.target_chip_distance = distance;
    return *this;
  }

  auto kickStraightToStopAt(double stop_distance) -> PositionCommandWrapper &
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

  auto kickStraightWithInitialSpeed(double initial_speed) -> PositionCommandWrapper &
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

  auto dribble(double power) -> PositionCommandWrapper &
  {
    latest_msg.dribble_power = power;
    latest_msg.kick_power = 0.0;
    return *this;
  }

  auto withDribble(double power) -> PositionCommandWrapper &
  {
    latest_msg.dribble_power = power;
    return *this;
  }

  auto setTargetTheta(double theta, double tolerance = 0.0) -> PositionCommandWrapper &
  {
    latest_msg.target_theta = theta;
    latest_msg.local_planner_config.theta_tolerance = tolerance;
    return *this;
  }

  auto setThetaTolerance(double tolerance) -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.theta_tolerance = tolerance;
    return *this;
  }

  auto stopHere() -> PositionCommandWrapper &
  {
    return setTargetPosition(robot->pose.pos).setTargetTheta(robot->pose.theta).setOmegaLimit(0.);
  }

  auto disablePlacementAvoidance() -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.disable_placement_avoidance = true;
    return *this;
  }

  auto enablePlacementAvoidance() -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.disable_placement_avoidance = false;
    return *this;
  }

  auto disableCollisionAvoidance() -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.disable_collision_avoidance = true;
    return *this;
  }

  auto enableCollisionAvoidance() -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.disable_collision_avoidance = false;
    return *this;
  }

  auto disableGoalAreaAvoidance() -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.disable_goal_area_avoidance = true;
    return *this;
  }

  auto enableGoalAreaAvoidance() -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.disable_goal_area_avoidance = false;
    return *this;
  }

  auto disableBallAvoidance() -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.disable_ball_avoidance = true;
    return *this;
  }

  auto enableBallAvoidance() -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.disable_ball_avoidance = false;
    return *this;
  }

  auto enableRotationStopOnAccel() -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.enable_rotation_stop_on_accel = true;
    return *this;
  }

  auto disableRotationStopOnAccel() -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.enable_rotation_stop_on_accel = false;
    return *this;
  }

  auto disableAnyAreaAvoidance() -> PositionCommandWrapper &
  {
    return disableGoalAreaAvoidance().disableBallAvoidance().disablePlacementAvoidance();
  }

  auto enableAnyAreaAvoidance() -> PositionCommandWrapper &
  {
    return enableGoalAreaAvoidance().enableBallAvoidance().enablePlacementAvoidance();
  }

  auto disableBasicAvoidances() -> PositionCommandWrapper &
  {
    return disableCollisionAvoidance().disableBallAvoidance();
  }

  auto enableBasicAvoidances() -> PositionCommandWrapper &
  {
    return enableCollisionAvoidance().enableBallAvoidance();
  }

  auto setGoalieDefault() -> PositionCommandWrapper &
  {
    disableCollisionAvoidance();
    disableGoalAreaAvoidance();
    return *this;
  }

  auto enableBallCenteringControl() -> PositionCommandWrapper &
  {
    latest_msg.enable_ball_centering_control = true;
    return *this;
  }

  auto enableLocalGoalie() -> PositionCommandWrapper &
  {
    latest_msg.local_goalie_enable = true;
    return *this;
  }

  auto setMaxVelocity(const std::string & factor_name, double max_velocity)
    -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.max_velocity_factors.emplace_back(
      crane_msgs::msg::NamedFloat().set__name(factor_name).set__value(max_velocity));
    return *this;
  }

  auto clearMaxVelocityFactors() -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.max_velocity_factors.clear();
    return *this;
  }

  auto setMaxAcceleration(const std::string & factor_name, double max_acceleration)
    -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.max_acceleration_factors.emplace_back(
      crane_msgs::msg::NamedFloat().set__name(factor_name).set__value(max_acceleration));
    return *this;
  }

  auto clearMaxAccelerationFactors() -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.max_acceleration_factors.clear();
    return *this;
  }

  auto setOmegaLimit(double omega_limit) -> PositionCommandWrapper &
  {
    latest_msg.omega_limit = omega_limit;
    return *this;
  }

  auto setTerminalVelocity(double terminal_velocity) -> PositionCommandWrapper &
  {
    latest_msg.local_planner_config.terminal_velocity = terminal_velocity;
    return *this;
  }

  auto stopEmergency(bool flag = true) -> PositionCommandWrapper &
  {
    latest_msg.stop_flag = flag;
    return *this;
  }

  auto lookAt(Point pos, double tolerance = 0.0) -> PositionCommandWrapper &
  {
    return setTargetTheta(getAngle(pos - robot->pose.pos), tolerance);
  }

  auto lookAtBall(double tolerance = 0.0) -> PositionCommandWrapper &
  {
    return lookAt(world_model->ball().pos, tolerance);
  }

  auto lookAtBallFrom(Point from, double tolerance = 0.0) -> PositionCommandWrapper &
  {
    return lookAtFrom(world_model->ball().pos, from, tolerance);
  }

  auto lookAtFrom(Point at, Point from, double tolerance = 0.0) -> PositionCommandWrapper &
  {
    return setTargetTheta(getAngle(at - from), tolerance);
  }

  auto addPlanningFactor(const std::string & name, const std::string & state) -> void
  {
    if (auto planning_factor = ranges::find_if(
          latest_msg.planning_factors,
          [name](const auto & planning_factor) { return planning_factor.name == name; });
        planning_factor == latest_msg.planning_factors.end() || planning_factor->value != state) {
      crane_msgs::msg::NamedString msg;
      msg.name = name;
      msg.value = state;
      latest_msg.planning_factors.emplace_back(msg);
    }
  }

  auto clearPlanningFactors() -> void { latest_msg.planning_factors.clear(); }

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

  auto setKickerModel(std::shared_ptr<KickerModel> kicker) -> PositionCommandWrapper &
  {
    kicker_model = kicker;
    return *this;
  }

  auto getKickerModel() const -> std::shared_ptr<KickerModel> { return kicker_model; }

  // ===== 速度計画トレース関連メソッド =====

  /**
   * @brief 速度計画トレースを有効化（新規トレースを作成）
   */
  auto enableVelocityPlanTrace() -> PositionCommandWrapper &
  {
    if (latest_msg.velocity_plan_trace.empty()) {
      latest_msg.velocity_plan_trace.push_back(VelocityPlanTracker::createTrace());
    }
    return *this;
  }

  /**
   * @brief 計画点を追加
   */
  auto addVelocityPlanPoint(
    const std::string & source, const Eigen::Vector2d & predicted_pos,
    const Eigen::Vector2d & predicted_vel, int32_t target_time_us,
    int32_t estimated_arrival_time_us = 0) -> void
  {
    if (!latest_msg.velocity_plan_trace.empty()) {
      VelocityPlanTracker::addPlanPoint(
        latest_msg.velocity_plan_trace[0], source, predicted_pos, predicted_vel, target_time_us,
        estimated_arrival_time_us);
    }
  }

  /**
   * @brief 速度修正を記録
   */
  auto addVelocityCorrection(
    const std::string & source, const Eigen::Vector2d & before_vel,
    const Eigen::Vector2d & after_vel) -> void
  {
    if (!latest_msg.velocity_plan_trace.empty()) {
      VelocityPlanTracker::addCorrection(
        latest_msg.velocity_plan_trace[0], source, before_vel, after_vel);
    }
  }

  /**
   * @brief 速度計画トレースが有効かどうかを確認
   */
  auto hasVelocityPlanTrace() const -> bool { return !latest_msg.velocity_plan_trace.empty(); }

  /**
   * @brief 速度計画トレースをコピー（RobotCommandからPositionCommandへの伝播用）
   */
  auto setVelocityPlanTrace(const std::vector<crane_msgs::msg::VelocityPlanTrace> & trace)
    -> PositionCommandWrapper &
  {
    latest_msg.velocity_plan_trace = trace;
    return *this;
  }

  // ===== 位置指令固有の関数 =====

  auto setTargetPosition(double x, double y, double tolerance = 0.0) -> PositionCommandWrapper &
  {
    latest_msg.target_x = x;
    latest_msg.target_y = y;
    latest_msg.position_tolerance = tolerance;
    return *this;
  }

  auto setTargetPosition(Point position, double tolerance = 0.0) -> PositionCommandWrapper &
  {
    return setTargetPosition(position.x(), position.y(), tolerance);
  }

  auto setDribblerTargetPosition(Point position, double tolerance = 0.0) -> PositionCommandWrapper &
  {
    double theta = latest_msg.target_theta;
    return setTargetPosition(
      position + getNormVec(theta + M_PI) * getRobot()->getDribblerDistance(), tolerance);
  }

  auto getTargetDistance() -> double
  {
    return std::hypot(
      latest_msg.target_x - robot->pose.pos.x(), latest_msg.target_y - robot->pose.pos.y());
  }

  auto setSpeedLimitAtTarget(double speed_limit) -> PositionCommandWrapper &
  {
    latest_msg.speed_limit_at_target = speed_limit;
    return *this;
  }
};
}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__POSITION_COMMAND_WRAPPER_HPP_
