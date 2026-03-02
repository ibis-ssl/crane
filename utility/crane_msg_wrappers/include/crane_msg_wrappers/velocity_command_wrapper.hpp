// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__VELOCITY_COMMAND_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__VELOCITY_COMMAND_WRAPPER_HPP_

#include <cmath>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_msgs/msg/robot_command.hpp>
#include <limits>
#include <memory>
#include <range/v3/algorithm/find_if.hpp>
#include <vector>

#include "delay_monitor_wrapper.hpp"
#include "velocity_plan_tracker.hpp"

namespace crane
{
/**
 * @brief local_planner → sender 用の速度指令ラッパー
 */
class VelocityCommandWrapper
{
public:
  using SharedPtr = std::shared_ptr<VelocityCommandWrapper>;

private:
  crane_msgs::msg::RobotCommand latest_msg;

public:
  VelocityCommandWrapper() = default;

  explicit VelocityCommandWrapper(uint8_t robot_id) { latest_msg.robot_id = robot_id; }

  // メッセージを取得
  auto getMsg() const -> const crane_msgs::msg::RobotCommand & { return latest_msg; }

  auto getEditableMsg() -> crane_msgs::msg::RobotCommand & { return latest_msg; }

  // ===== 速度指令固有の関数 =====

  auto setVelocity(Velocity velocity) -> VelocityCommandWrapper &
  {
    return setVelocityNorm(velocity.norm()).setVelocityAngle(getAngle(velocity));
  }

  auto setVelocity(double x, double y) -> VelocityCommandWrapper & { return setVelocity({x, y}); }

  auto setVelocityNorm(double r) -> VelocityCommandWrapper &
  {
    latest_msg.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
    if (latest_msg.polar_velocity_target_mode.empty()) {
      latest_msg.polar_velocity_target_mode.emplace_back();
    }
    latest_msg.polar_velocity_target_mode.front().target_velocity_r = r;
    return *this;
  }

  auto setVelocityAngle(double theta) -> VelocityCommandWrapper &
  {
    latest_msg.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
    if (latest_msg.polar_velocity_target_mode.empty()) {
      latest_msg.polar_velocity_target_mode.emplace_back();
    }
    latest_msg.polar_velocity_target_mode.front().target_velocity_theta = theta;
    return *this;
  }

  // ===== 将来の位置指令対応用 =====

  auto setTargetPosition(double x, double y) -> VelocityCommandWrapper &
  {
    if (latest_msg.position_target_mode.empty()) {
      latest_msg.position_target_mode.emplace_back();
    }
    latest_msg.position_target_mode.front().target_x = x;
    latest_msg.position_target_mode.front().target_y = y;
    return *this;
  }

  auto setTargetPosition(Point position) -> VelocityCommandWrapper &
  {
    return setTargetPosition(position.x(), position.y());
  }

  auto clearTargetPosition() -> VelocityCommandWrapper &
  {
    if (latest_msg.position_target_mode.empty()) {
      latest_msg.position_target_mode.emplace_back();
    }
    latest_msg.position_target_mode.front().target_x = std::numeric_limits<float>::quiet_NaN();
    latest_msg.position_target_mode.front().target_y = std::numeric_limits<float>::quiet_NaN();
    return *this;
  }

  auto hasTargetPosition() const -> bool
  {
    return (
      !latest_msg.position_target_mode.empty() &&
      std::isfinite(latest_msg.position_target_mode.front().target_x) &&
      std::isfinite(latest_msg.position_target_mode.front().target_y));
  }

  // ===== 共通操作関数 =====

  auto setRobotId(uint8_t id) -> VelocityCommandWrapper &
  {
    latest_msg.robot_id = id;
    return *this;
  }

  auto kickWithChip(double power) -> VelocityCommandWrapper &
  {
    latest_msg.chip_enable = true;
    latest_msg.kick_power = power;
    return *this;
  }

  auto kickStraight(double power) -> VelocityCommandWrapper &
  {
    latest_msg.chip_enable = false;
    latest_msg.kick_power = power;
    return *this;
  }

  auto dribble(double power) -> VelocityCommandWrapper &
  {
    latest_msg.dribble_power = power;
    latest_msg.kick_power = 0.0;
    return *this;
  }

  auto withDribble(double power) -> VelocityCommandWrapper &
  {
    latest_msg.dribble_power = power;
    return *this;
  }

  auto setTargetTheta(double theta) -> VelocityCommandWrapper &
  {
    latest_msg.target_theta = theta;
    return *this;
  }

  auto setOmegaLimit(double omega_limit) -> VelocityCommandWrapper &
  {
    latest_msg.omega_limit = omega_limit;
    return *this;
  }

  auto stopEmergency(bool flag = true) -> VelocityCommandWrapper &
  {
    latest_msg.stop_flag = flag;
    return *this;
  }

  auto enableLocalFeedback(bool enable = true) -> VelocityCommandWrapper &
  {
    latest_msg.enable_local_feedback = enable;
    return *this;
  }

  auto setMaxVelocity(double max_velocity) -> VelocityCommandWrapper &
  {
    latest_msg.local_planner_config.final_planned_max_velocity.name = "velocity_wrapper";
    latest_msg.local_planner_config.final_planned_max_velocity.value = max_velocity;
    return *this;
  }

  auto setMaxAcceleration(double max_acceleration) -> VelocityCommandWrapper &
  {
    latest_msg.local_planner_config.final_planned_max_acceleration.name = "velocity_wrapper";
    latest_msg.local_planner_config.final_planned_max_acceleration.value = max_acceleration;
    return *this;
  }

  auto setCurrentPose(double x, double y, double theta) -> VelocityCommandWrapper &
  {
    latest_msg.current_pose.x = x;
    latest_msg.current_pose.y = y;
    latest_msg.current_pose.theta = theta;
    return *this;
  }

  auto setCurrentVelocity(double x, double y, double omega) -> VelocityCommandWrapper &
  {
    latest_msg.current_velocity.x = x;
    latest_msg.current_velocity.y = y;
    latest_msg.current_velocity.theta = omega;
    return *this;
  }

  auto setLatencyMs(double latency_ms) -> VelocityCommandWrapper &
  {
    latest_msg.latency_ms = latency_ms;
    return *this;
  }

  auto setElapsedTimeMsSinceLastVision(uint16_t elapsed_time_ms) -> VelocityCommandWrapper &
  {
    latest_msg.elapsed_time_ms_since_last_vision = elapsed_time_ms;
    return *this;
  }

  auto addPlanningFactor(const std::string & name, const std::string & state) -> void
  {
    if (
      auto planning_factor = ranges::find_if(
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

  auto setPlannerName(const std::string & planner_name) -> VelocityCommandWrapper &
  {
    latest_msg.planner_name = planner_name;
    return *this;
  }

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

  auto setLocalPlannerConfig(const crane_msgs::msg::LocalPlannerConfig & config)
    -> VelocityCommandWrapper &
  {
    latest_msg.local_planner_config = config;
    return *this;
  }

  // ===== 速度計画トレース関連メソッド =====

  /**
   * @brief 速度計画トレースを有効化（新規トレースを作成）
   */
  auto enableVelocityPlanTrace() -> VelocityCommandWrapper &
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
   * @brief 速度計画トレースをコピー（RobotCommandからRobotCommandへの伝播用）
   */
  auto setVelocityPlanTrace(const std::vector<crane_msgs::msg::VelocityPlanTrace> & trace)
    -> VelocityCommandWrapper &
  {
    latest_msg.velocity_plan_trace.assign(trace.begin(), trace.end());
    return *this;
  }

  // ===== RobotCommand -> RobotCommand 変換ユーティリティ =====
  static auto fromPositionCommand(
    const crane_msgs::msg::RobotCommand & pos_cmd, double velocity_r, double velocity_theta)
    -> crane_msgs::msg::RobotCommand
  {
    crane_msgs::msg::RobotCommand vel_cmd;
    vel_cmd.robot_id = pos_cmd.robot_id;
    vel_cmd.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
    vel_cmd.polar_velocity_target_mode.emplace_back();
    vel_cmd.polar_velocity_target_mode.front().target_velocity_r = velocity_r;
    vel_cmd.polar_velocity_target_mode.front().target_velocity_theta = velocity_theta;
    vel_cmd.target_theta = pos_cmd.target_theta;
    vel_cmd.omega_limit = pos_cmd.omega_limit;
    vel_cmd.chip_enable = pos_cmd.chip_enable;
    vel_cmd.kick_power = pos_cmd.kick_power;
    vel_cmd.dribble_power = pos_cmd.dribble_power;
    vel_cmd.stop_flag = pos_cmd.stop_flag;
    vel_cmd.current_pose = pos_cmd.current_pose;
    vel_cmd.current_velocity = pos_cmd.current_velocity;
    vel_cmd.planning_factors = pos_cmd.planning_factors;
    vel_cmd.planner_name = pos_cmd.planner_name;
    vel_cmd.delay_checkpoints = pos_cmd.delay_checkpoints;
    vel_cmd.local_planner_config = pos_cmd.local_planner_config;
    vel_cmd.velocity_plan_trace = pos_cmd.velocity_plan_trace;
    return vel_cmd;
  }
};
}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__VELOCITY_COMMAND_WRAPPER_HPP_
