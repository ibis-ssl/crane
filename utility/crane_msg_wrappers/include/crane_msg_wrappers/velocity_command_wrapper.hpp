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
#include <limits>
#include <memory>
#include <vector>

#include "command_wrapper_base.hpp"
#include "delay_monitor_wrapper.hpp"
#include "velocity_plan_tracker.hpp"

namespace crane
{
/**
 * @brief local_planner → sender 用の速度指令ラッパー
 */
class VelocityCommandWrapper : public CommandWrapperBase<VelocityCommandWrapper>,
                               public DelayMonitorMixin<VelocityCommandWrapper>,
                               public VelocityPlanTraceMixin<VelocityCommandWrapper>
{
  friend class CommandWrapperBase<VelocityCommandWrapper>;
  friend class DelayMonitorMixin<VelocityCommandWrapper>;
  friend class VelocityPlanTraceMixin<VelocityCommandWrapper>;

public:
  using SharedPtr = std::shared_ptr<VelocityCommandWrapper>;

private:
  crane_msgs::msg::RobotCommand latest_msg;

  auto getLatestMsg() -> crane_msgs::msg::RobotCommand & { return latest_msg; }
  auto getLatestMsg() const -> const crane_msgs::msg::RobotCommand & { return latest_msg; }
  auto getDelayCheckpoints() -> crane_msgs::msg::DelayCheckpoints &
  {
    return latest_msg.delay_checkpoints;
  }
  auto getDelayCheckpoints() const -> const crane_msgs::msg::DelayCheckpoints &
  {
    return latest_msg.delay_checkpoints;
  }
  auto getVelocityPlanTrace() -> decltype(latest_msg.velocity_plan_trace) &
  {
    return latest_msg.velocity_plan_trace;
  }
  auto getVelocityPlanTrace() const -> const decltype(latest_msg.velocity_plan_trace) &
  {
    return latest_msg.velocity_plan_trace;
  }

public:
  VelocityCommandWrapper() = default;

  explicit VelocityCommandWrapper(uint8_t robot_id) { latest_msg.robot_id = robot_id; }

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

  auto setPlannerName(const std::string & planner_name) -> VelocityCommandWrapper &
  {
    latest_msg.planner_name = planner_name;
    return *this;
  }

  auto setLocalPlannerConfig(const crane_msgs::msg::LocalPlannerConfig & config)
    -> VelocityCommandWrapper &
  {
    latest_msg.local_planner_config = config;
    return *this;
  }

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
