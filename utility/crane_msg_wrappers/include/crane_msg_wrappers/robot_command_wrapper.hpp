// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__ROBOT_COMMAND_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__ROBOT_COMMAND_WRAPPER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_physics/kicker_model.hpp>
#include <memory>
#include <vector>

#include "command_wrapper_base.hpp"
#include "delay_monitor_wrapper.hpp"
#include "velocity_plan_tracker.hpp"
#include "world_model_wrapper.hpp"

namespace crane
{
class RobotCommandWrapper : public CommandWrapperBase<RobotCommandWrapper>,
                            public DelayMonitorMixin<RobotCommandWrapper>,
                            public VelocityPlanTraceMixin<RobotCommandWrapper>
{
  friend class CommandWrapperBase<RobotCommandWrapper>;
  friend class DelayMonitorMixin<RobotCommandWrapper>;
  friend class VelocityPlanTraceMixin<RobotCommandWrapper>;

public:
  using SharedPtr = std::shared_ptr<RobotCommandWrapper>;

private:
  crane_msgs::msg::RobotCommand latest_msg;

  std::shared_ptr<RobotInfo> robot;

  WorldModelWrapper::SharedPtr world_model;

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

  // キッカーモデル（停止距離指定キック用）
  std::shared_ptr<KickerModel> kicker_model;

  // 現在のモード
  uint8_t current_mode;

  auto getID() const -> uint8_t { return latest_msg.robot_id; }

public:
  std::string name;

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
    if (latest_msg.position_target_mode.empty()) {
      latest_msg.position_target_mode.emplace_back();
    }
    latest_msg.local_planner_config.max_velocity_factors.clear();
    latest_msg.local_planner_config.max_acceleration_factors.clear();
    current_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
    return *this;
  }

  auto usePolarVelocityMode() -> RobotCommandWrapper &
  {
    latest_msg.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
    if (latest_msg.polar_velocity_target_mode.empty()) {
      latest_msg.polar_velocity_target_mode.emplace_back();
    }
    latest_msg.local_planner_config.max_velocity_factors.clear();
    latest_msg.local_planner_config.max_acceleration_factors.clear();
    current_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
    return *this;
  }

  // 現在のモードを返す
  auto getCurrentMode() const -> uint8_t { return current_mode; }

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
    if (world_model->isPracticeKickProhibited()) {
      latest_msg.kick_power = 0.0;
      return *this;
    }
    latest_msg.local_planner_config.kick_power_override = false;
    latest_msg.chip_enable = true;
    latest_msg.kick_power = power;
    return *this;
  }

  auto kickStraight(double power) -> RobotCommandWrapper &
  {
    if (world_model->isPracticeKickProhibited()) {
      latest_msg.kick_power = 0.0;
      return *this;
    }
    latest_msg.local_planner_config.kick_power_override = false;
    latest_msg.chip_enable = false;
    latest_msg.kick_power = power;
    return *this;
  }

  auto setKickStraightTargetSpeed(double speed_mps) -> RobotCommandWrapper &
  {
    if (world_model->isPracticeKickProhibited()) {
      latest_msg.kick_power = 0.0;
      return *this;
    }
    latest_msg.local_planner_config.kick_power_override = true;
    latest_msg.chip_enable = false;
    latest_msg.local_planner_config.target_kick_ball_speed = speed_mps;
    return *this;
  }

  auto setKickWithChipTargetDistance(double distance) -> RobotCommandWrapper &
  {
    if (world_model->isPracticeKickProhibited()) {
      latest_msg.kick_power = 0.0;
      return *this;
    }
    latest_msg.local_planner_config.kick_power_override = true;
    latest_msg.chip_enable = true;
    latest_msg.local_planner_config.target_chip_distance = distance;
    return *this;
  }

  auto kickStraightToStopAt(double stop_distance) -> RobotCommandWrapper &
  {
    if (world_model->isPracticeKickProhibited()) {
      latest_msg.kick_power = 0.0;
      return *this;
    }
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
    if (world_model->isPracticeKickProhibited()) {
      latest_msg.kick_power = 0.0;
      return *this;
    }
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
    addPlanningFactor("CommandAction", "STOP_HERE");
    addPlanningFactor("CommandSource", name);
    switch (current_mode) {
      case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE:
        return setTargetPosition(robot->pose.pos, 0.001).setOmegaLimit(0.);
      case crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE:
        return setVelocityNorm(0.);
      default:
        // 不明なモードの場合は位置モードで停止
        usePositionMode();
        return setTargetPosition(robot->pose.pos, 0.001).setOmegaLimit(0.);
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

  auto disableFieldBoundary() -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.disable_field_boundary = true;
    return *this;
  }

  auto enableFieldBoundary() -> RobotCommandWrapper &
  {
    latest_msg.local_planner_config.disable_field_boundary = false;
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
    return disableGoalAreaAvoidance()
      .disableBallAvoidance()
      .disablePlacementAvoidance()
      .disableFieldBoundary();
  }

  auto enableAnyAreaAvoidance() -> RobotCommandWrapper &
  {
    return enableGoalAreaAvoidance()
      .enableBallAvoidance()
      .enablePlacementAvoidance()
      .enableFieldBoundary();
  }

  auto disableBasicAvoidances() -> RobotCommandWrapper &
  {
    return disableCollisionAvoidance().disableBallAvoidance();
  }

  auto enableBasicAvoidances() -> RobotCommandWrapper &
  {
    return enableCollisionAvoidance().enableBallAvoidance();
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

  // ===== KickerModel管理メソッド =====

  auto setKickerModel(std::shared_ptr<KickerModel> kicker) -> RobotCommandWrapper &
  {
    kicker_model = kicker;
    return *this;
  }

  auto getKickerModel() const -> std::shared_ptr<KickerModel> { return kicker_model; }

  // ===== PositionTargetMode固有の関数 =====

  auto setTargetPosition(double x, double y, double tolerance = 0.01) -> RobotCommandWrapper &
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

  auto setTargetPosition(Point position, double tolerance = 0.01) -> RobotCommandWrapper &
  {
    return setTargetPosition(position.x(), position.y(), tolerance);
  }

  auto setDribblerTargetPosition(Point position, double tolerance = 0.01) -> RobotCommandWrapper &
  {
    double theta = latest_msg.target_theta;
    return setTargetPosition(
      position + getNormVec(theta + M_PI) * getRobot()->getDribblerDistance(), tolerance);
  }

  auto setSpeedLimitAtTarget(double speed_limit) -> RobotCommandWrapper &
  {
    if (!latest_msg.position_target_mode.empty()) {
      latest_msg.position_target_mode.front().speed_limit_at_target = speed_limit;
    }
    return *this;
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
