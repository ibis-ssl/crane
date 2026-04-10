// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/rvo2_planner.hpp"

#include <algorithm>
#include <boost/stacktrace.hpp>
#include <crane_msg_wrappers/command_wrapper_base.hpp>
#include <crane_visualization_interfaces/crane_visualizer_wrapper.hpp>
#include <cstdint>
#include <iomanip>
#include <range/v3/algorithm/find_if.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/referee.hpp>
#include <sstream>

// cspell: ignore OBST

namespace crane
{
namespace
{
auto pointChanged(const Point & before, const Point & after, double epsilon = 1e-4) -> bool
{
  return (before - after).norm() > epsilon;
}

void drawRobotRadiusWithSpeed(
  const std::shared_ptr<VisualizerMessageBuilder> & visualizer, Point center, double radius,
  double speed, const std::string & color = "yellow", double circle_opacity = 0.2,
  double text_opacity = 0.5, double stroke_width = 10.0, double font_size = 50.0)
{
  visualizer->circle()
    .center(center)
    .radius(radius)
    .stroke(color, circle_opacity)
    .strokeWidth(stroke_width)
    .build();

  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << speed << "m/s";
  visualizer->text()
    .position(center + Vector2(0, radius + 0.07))
    .text(ss.str())
    .fontSize(font_size)
    .textAnchor("middle")
    .fill(color, text_opacity)
    .build();
}
}  // namespace

RVO2Planner::RVO2Planner(rclcpp::Node & node)
: LocalPlannerBase("rvo2_local_planner", node),
  acceleration_factor("acceleration_factor", node, 1.5)
{
  node.declare_parameter("rvo_time_step", RVO_TIME_STEP);
  RVO_TIME_STEP = node.get_parameter("rvo_time_step").as_double();
  node.declare_parameter("rvo_neighbor_dist", RVO_NEIGHBOR_DIST);
  RVO_NEIGHBOR_DIST = node.get_parameter("rvo_neighbor_dist").as_double();
  node.declare_parameter("rvo_max_neighbors", RVO_MAX_NEIGHBORS);
  RVO_MAX_NEIGHBORS = node.get_parameter("rvo_max_neighbors").as_int();
  node.declare_parameter("rvo_time_horizon", RVO_TIME_HORIZON);
  RVO_TIME_HORIZON = node.get_parameter("rvo_time_horizon").as_double();
  node.declare_parameter("rvo_time_horizon_obst", RVO_TIME_HORIZON_OBST);
  RVO_TIME_HORIZON_OBST = node.get_parameter("rvo_time_horizon_obst").as_double();
  node.declare_parameter("rvo_radius", RVO_RADIUS);
  RVO_RADIUS = node.get_parameter("rvo_radius").as_double();
  node.declare_parameter("rvo_max_speed", RVO_MAX_SPEED);
  RVO_MAX_SPEED = node.get_parameter("rvo_max_speed").as_double();

  node.declare_parameter("max_vel", MAX_VEL);
  MAX_VEL = node.get_parameter("max_vel").as_double();

  node.declare_parameter("stop_state_max_velocity", STOP_STATE_MAX_VELOCITY);
  STOP_STATE_MAX_VELOCITY = node.get_parameter("stop_state_max_velocity").as_double();

  node.declare_parameter("field_boundary_offset", FIELD_BOUNDARY_OFFSET);
  FIELD_BOUNDARY_OFFSET = node.get_parameter("field_boundary_offset").as_double();

  node.declare_parameter("crash_speed_limit", CRASH_SPEED_LIMIT);
  CRASH_SPEED_LIMIT = node.get_parameter("crash_speed_limit").as_double();
  node.declare_parameter("crash_safety_margin", CRASH_SAFETY_MARGIN);
  CRASH_SAFETY_MARGIN = node.get_parameter("crash_safety_margin").as_double();
  node.declare_parameter("crash_avoidance_distance", CRASH_AVOIDANCE_DISTANCE);
  CRASH_AVOIDANCE_DISTANCE = node.get_parameter("crash_avoidance_distance").as_double();
  node.declare_parameter("crash_avoidance_decel_distance", CRASH_AVOIDANCE_DECEL_DISTANCE);
  CRASH_AVOIDANCE_DECEL_DISTANCE = node.get_parameter("crash_avoidance_decel_distance").as_double();
  if (CRASH_AVOIDANCE_DISTANCE <= CRASH_AVOIDANCE_DECEL_DISTANCE) {
    RCLCPP_ERROR(
      node.get_logger(),
      "crash_avoidance_distance (%.2f) must be > crash_avoidance_decel_distance (%.2f). "
      "Resetting to defaults.",
      CRASH_AVOIDANCE_DISTANCE, CRASH_AVOIDANCE_DECEL_DISTANCE);
    CRASH_AVOIDANCE_DISTANCE = 1.0;
    CRASH_AVOIDANCE_DECEL_DISTANCE = 0.5;
  }

  node.declare_parameter("penalty_area_offset", PENALTY_AREA_OFFSET);
  PENALTY_AREA_OFFSET = node.get_parameter("penalty_area_offset").as_double();
  node.declare_parameter("penalty_area_surrounding_offset", PENALTY_AREA_SURROUNDING_OFFSET);
  PENALTY_AREA_SURROUNDING_OFFSET =
    node.get_parameter("penalty_area_surrounding_offset").as_double();
  node.declare_parameter(
    "penalty_area_force_waypoint_on_crossing", PENALTY_AREA_FORCE_WAYPOINT_ON_CROSSING);
  PENALTY_AREA_FORCE_WAYPOINT_ON_CROSSING =
    node.get_parameter("penalty_area_force_waypoint_on_crossing").as_bool();
  node.declare_parameter("penalty_area_time_horizon_obst", PENALTY_AREA_TIME_HORIZON_OBST);
  PENALTY_AREA_TIME_HORIZON_OBST =
    static_cast<float>(node.get_parameter("penalty_area_time_horizon_obst").as_double());

  node.declare_parameter("enable_velocity_plan_trace", false);
  enable_velocity_plan_trace = node.get_parameter("enable_velocity_plan_trace").as_bool();

  node.declare_parameter("velocity_damping_gain", velocity_damping_gain);
  velocity_damping_gain = node.get_parameter("velocity_damping_gain").as_double();

  rvo_sim = std::make_unique<RVO::RVOSimulator>(
    RVO_TIME_STEP, RVO_NEIGHBOR_DIST, RVO_MAX_NEIGHBORS, RVO_TIME_HORIZON, RVO_TIME_HORIZON_OBST,
    RVO_RADIUS, RVO_MAX_SPEED);

  // friend robots -> 0~19
  // enemy robots -> 20~39
  for (int i = 0; i < 40; i++) {
    rvo_sim->addAgent(RVO::Vector2(20.0f, 20.0f));
  }

  sub_feedback_array = node.create_subscription<crane_msgs::msg::RobotFeedbackArray>(
    "/robot_feedback", 1,
    [this](const crane_msgs::msg::RobotFeedbackArray & msg) { latest_feedback = msg; });
}

auto RVO2Planner::initializePenaltyAreaObstacles() -> void
{
  // ペナルティエリアをRVO2のポリゴン障害物として登録する。
  // processObstacles()呼び出し後は変更不可なので、フィールド情報確定後に一度だけ呼ぶ。
  // 頂点は反時計回り（RVO2の仕様）で指定する。
  const Box our_area = world_model->getOurPenaltyArea();
  const Box their_area = world_model->getTheirPenaltyArea();
  auto addBoxObstacle = [&](const Box & box) {
    const float xmin = static_cast<float>(box.min_corner().x());
    const float xmax = static_cast<float>(box.max_corner().x());
    const float ymin = static_cast<float>(box.min_corner().y());
    const float ymax = static_cast<float>(box.max_corner().y());
    // 反時計回り: 左下 → 右下 → 右上 → 左上
    rvo_sim->addObstacle({{xmin, ymin}, {xmax, ymin}, {xmax, ymax}, {xmin, ymax}});
  };
  addBoxObstacle(our_area);
  addBoxObstacle(their_area);
  rvo_sim->processObstacles();
  penalty_area_obstacles_initialized = true;
  RCLCPP_INFO(
    rclcpp::get_logger("rvo2_local_planner"),
    "Penalty area obstacles initialized (our: [%.2f,%.2f]x[%.2f,%.2f], "
    "their: [%.2f,%.2f]x[%.2f,%.2f])",
    our_area.min_corner().x(), our_area.max_corner().x(), our_area.min_corner().y(),
    our_area.max_corner().y(), their_area.min_corner().x(), their_area.max_corner().x(),
    their_area.min_corner().y(), their_area.max_corner().y());
}

auto RVO2Planner::getCurrentEstimatedPosition(uint8_t robot_id, const Point & fallback) const
  -> Point
{
  if (
    auto feedback = ranges::find_if(
      latest_feedback.feedback, [&](const auto & f) { return f.robot_id == robot_id; });
    feedback != latest_feedback.feedback.end()) {
    return Point(feedback->odom[0], feedback->odom[1]);
  }
  return fallback;
}

auto RVO2Planner::initializePlanningFactors(crane_msgs::msg::RobotCommand & command) const -> void
{
  setPlanningStage(command, "INIT");
  addOrUpdatePlanningFactor(command, "RVO2AdjustFieldBoundary", "0");
  addOrUpdatePlanningFactor(command, "RVO2AdjustPenaltyArea", "0");
  addOrUpdatePlanningFactor(command, "RVO2PenaltyCrossingDetected", "0");
  addOrUpdatePlanningFactor(command, "RVO2PenaltyBypassSide", "NONE");
  addOrUpdatePlanningFactor(command, "RVO2AdjustBallAvoidance", "0");
  addOrUpdatePlanningFactor(command, "RVO2AdjustPlacementAvoidance", "0");
  addOrUpdatePlanningFactor(command, "RVO2TargetFallback", "NONE");
}

auto RVO2Planner::setPlanningStage(
  crane_msgs::msg::RobotCommand & command, const std::string & stage) const -> void
{
  addOrUpdatePlanningFactor(command, "RVO2Stage", stage);
}

auto RVO2Planner::addMaxVelocityFactor(
  crane_msgs::msg::RobotCommand & command, const std::string & name, double value) const -> void
{
  command.local_planner_config.max_velocity_factors.emplace_back(
    crane_msgs::msg::NamedFloat().set__name(name).set__value(value));
}

auto RVO2Planner::createPreprocessContext(const crane_msgs::msg::RobotCommand & command) const
  -> PreprocessContext
{
  PreprocessContext ctx;
  ctx.robot_id = command.robot_id;
  ctx.current_pose_position = Point(command.current_pose.x, command.current_pose.y);
  ctx.current_estimated_position =
    getCurrentEstimatedPosition(command.robot_id, ctx.current_pose_position);
  if (!command.position_target_mode.empty()) {
    const auto & pos_mode = command.position_target_mode.front();
    ctx.target_pos = Point(pos_mode.target_x, pos_mode.target_y);
    ctx.original_target_pos = ctx.target_pos;
  }
  return ctx;
}

auto RVO2Planner::applyInputValidation(
  PreprocessContext & ctx, crane_msgs::msg::RobotCommand & command) const -> void
{
  if (command.position_target_mode.empty()) {
    ctx.is_valid = false;
    setPlanningStage(command, "INPUT_INVALID");
    addOrUpdatePlanningFactor(command, "RVO2TargetFallback", "NO_POSITION_TARGET_MODE");
    return;
  }

  auto & pos_mode = command.position_target_mode.front();

  setPlanningStage(command, "INPUT_VALIDATION");
  if (std::isnan(ctx.target_pos.x()) || std::isnan(ctx.target_pos.y())) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("rvo2_local_planner"),
      "[RVO2Planner] NaN detected in target_pos for robot "
        << static_cast<int>(command.robot_id) << ": target_pos(" << ctx.target_pos.x() << ", "
        << ctx.target_pos.y() << "), using current position as fallback\n"
        << crane_msgs::msg::to_yaml(command));
    ctx.target_pos = ctx.current_pose_position;
    ctx.run_target_adjustments = false;
    addOrUpdatePlanningFactor(command, "RVO2TargetFallback", "NAN_TARGET");
  }

  if (std::isnan(ctx.current_pose_position.x()) || std::isnan(ctx.current_pose_position.y())) {
    RCLCPP_WARN(
      rclcpp::get_logger("rvo2_local_planner"),
      "[RVO2Planner] NaN detected in current_pos for robot %d: current_pos(%f, %f), stopping "
      "planner output for this robot",
      static_cast<int>(command.robot_id), ctx.current_pose_position.x(),
      ctx.current_pose_position.y());
    ctx.is_valid = false;
    ctx.current_pose_position = Point(20.0, 20.0);
    ctx.current_estimated_position = ctx.current_pose_position;
    ctx.target_vel = Velocity::Zero();
    ctx.max_vel = 0.0;
    setPlanningStage(command, "INPUT_INVALID");
    addOrUpdatePlanningFactor(command, "RVO2TargetFallback", "NAN_CURRENT");
    return;
  }

  if (
    std::isnan(ctx.current_estimated_position.x()) ||
    std::isnan(ctx.current_estimated_position.y())) {
    ctx.current_estimated_position = ctx.current_pose_position;
    addOrUpdatePlanningFactor(command, "RVO2TargetFallback", "NAN_FEEDBACK_FALLBACK_POSE");
  }

  pos_mode.target_x = ctx.target_pos.x();
  pos_mode.target_y = ctx.target_pos.y();
}

auto RVO2Planner::applyTargetAdjustmentPipeline(
  PreprocessContext & ctx, crane_msgs::msg::RobotCommand & command) -> void
{
  if (command.position_target_mode.empty()) {
    return;
  }

  setPlanningStage(command, "TARGET_ADJUSTMENT");
  Point before = ctx.target_pos;
  adjustForFieldBoundary(ctx.target_pos, ctx.current_pose_position, command);
  if (pointChanged(before, ctx.target_pos)) {
    addOrUpdatePlanningFactor(command, "RVO2AdjustFieldBoundary", "1");
  }

  before = ctx.target_pos;
  adjustForPenaltyAreaAvoidance(ctx.target_pos, ctx.current_pose_position, command);
  if (pointChanged(before, ctx.target_pos)) {
    addOrUpdatePlanningFactor(command, "RVO2AdjustPenaltyArea", "1");
  }

  before = ctx.target_pos;
  adjustForBallAvoidance(ctx.target_pos, ctx.current_pose_position, command);
  if (pointChanged(before, ctx.target_pos)) {
    addOrUpdatePlanningFactor(command, "RVO2AdjustBallAvoidance", "1");
  }

  before = ctx.target_pos;
  adjustForPlacementAvoidance(ctx.target_pos, ctx.current_pose_position, command);
  if (pointChanged(before, ctx.target_pos)) {
    addOrUpdatePlanningFactor(command, "RVO2AdjustPlacementAvoidance", "1");
  }

  auto & pos_mode = command.position_target_mode.front();
  pos_mode.target_x = ctx.target_pos.x();
  pos_mode.target_y = ctx.target_pos.y();
  addOrUpdatePlanningFactor(
    command, "RVO2TargetAdjustedDistance",
    formatPlanningDouble((ctx.target_pos - ctx.original_target_pos).norm()));
}

auto RVO2Planner::computePreferredVelocityStage(
  PreprocessContext & ctx, crane_msgs::msg::RobotCommand & command, uint8_t referee_command) const
  -> void
{
  setPlanningStage(command, "PREF_VELOCITY");
  Vector2 position_diff;
  position_diff << ctx.target_pos.x() - ctx.current_estimated_position.x(),
    ctx.target_pos.y() - ctx.current_estimated_position.y();

  const double max_brk = planning_deceleration;
  addMaxVelocityFactor(command, "RVO2Planner::max_vel from parameter", MAX_VEL);
  if (referee_command == robocup_ssl_msgs::msg::RefereeCommand::STOP) {
    addMaxVelocityFactor(command, "RVO2Planner STOP制限", STOP_STATE_MAX_VELOCITY);
  }
  ctx.max_vel = resolveMaxVelocityFactors(command, MAX_VEL);

  auto brk_vel = [max_brk](double d) -> double {
    const double abs_d = std::abs(d);
    return (abs_d > 1e-9) ? std::copysign(std::sqrt(2.0 * max_brk * abs_d), d) : 0.0;
  };

  ctx.target_vel << brk_vel(position_diff.x()), brk_vel(position_diff.y());

  const Vector2 current_vel(command.current_velocity.x, command.current_velocity.y);
  ctx.target_vel -= velocity_damping_gain * current_vel;

  if (position_diff.norm() > 0.01 && ctx.target_vel.dot(position_diff) < 0.0) {
    ctx.target_vel.setZero();
  }

  const double target_vel_norm = ctx.target_vel.norm();
  if (target_vel_norm > ctx.max_vel) {
    ctx.target_vel *= ctx.max_vel / target_vel_norm;
  }
}

auto RVO2Planner::applyCrashAvoidanceConstraint(
  PreprocessContext & ctx, crane_msgs::msg::RobotCommand & command) const -> void
{
  if (command.local_planner_config.disable_crash_avoidance) {
    return;
  }

  const Point our_pos = ctx.current_pose_position;
  for (const auto & enemy_robot : world_model->theirs().robotsWhere().available().get()) {
    const Vector2 to_enemy = enemy_robot->pose.pos - our_pos;
    const double dist = to_enemy.norm();

    if (dist < CRASH_AVOIDANCE_DISTANCE && dist > 0.01) {
      const Vector2 dir = to_enemy.normalized();
      const double enemy_approach = -enemy_robot->vel.linear.dot(dir);
      double safe_approach =
        CRASH_SPEED_LIMIT - std::max(0.0, enemy_approach) - CRASH_SAFETY_MARGIN;
      safe_approach = std::max(safe_approach, 0.0);

      const double factor = 1.0 - std::clamp(
                                    (dist - CRASH_AVOIDANCE_DECEL_DISTANCE) /
                                      (CRASH_AVOIDANCE_DISTANCE - CRASH_AVOIDANCE_DECEL_DISTANCE),
                                    0.0, 1.0);

      const double approach_component = ctx.target_vel.dot(dir);
      const double max_approach = approach_component * (1.0 - factor) + safe_approach * factor;
      if (approach_component > max_approach && approach_component > 0.0) {
        const Vector2 lateral = ctx.target_vel - approach_component * dir;
        ctx.target_vel = lateral + max_approach * dir;
        addOrUpdatePlanningFactor(
          command, "CrashAvoidance",
          "robot" + std::to_string(enemy_robot->id) + ":" + formatPlanningDouble(max_approach));
      }
    }
  }
}

auto RVO2Planner::applyPenaltyAreaBrakingConstraint(
  PreprocessContext & ctx, const Box & area, const crane_msgs::msg::RobotCommand & command) const
  -> void
{
  const double penalty_area_offset =
    needsExpandedPenaltyAreaOffset(world_model->getMsg().play_situation.command.value)
      ? PENALTY_AREA_OFFSET_STOP
      : PENALTY_AREA_OFFSET;

  const double xmin = area.min_corner().x() - penalty_area_offset;
  const double xmax = area.max_corner().x() + penalty_area_offset;
  const double ymin = area.min_corner().y() - penalty_area_offset;
  const double ymax = area.max_corner().y() + penalty_area_offset;

  if (
    ctx.current_estimated_position.x() >= xmin && ctx.current_estimated_position.x() <= xmax &&
    ctx.current_estimated_position.y() >= ymin && ctx.current_estimated_position.y() <= ymax) {
    const double d_left = ctx.current_estimated_position.x() - xmin;
    const double d_right = xmax - ctx.current_estimated_position.x();
    const double d_bottom = ctx.current_estimated_position.y() - ymin;
    const double d_top = ymax - ctx.current_estimated_position.y();
    const double min_d = std::min({d_left, d_right, d_bottom, d_top});
    Eigen::Vector2d escape_dir(0, 0);
    if (min_d == d_left) {
      escape_dir = Eigen::Vector2d(-1, 0);
    } else if (min_d == d_right) {
      escape_dir = Eigen::Vector2d(1, 0);
    } else if (min_d == d_bottom) {
      escape_dir = Eigen::Vector2d(0, -1);
    } else {
      escape_dir = Eigen::Vector2d(0, 1);
    }

    const double escape_component = ctx.target_vel.dot(escape_dir);
    if (escape_component <= 0.0) {
      ctx.target_vel -= escape_component * escape_dir;
      ctx.target_vel += 0.5 * escape_dir;
    }
    return;
  }

  const double dx_left = xmin - ctx.current_estimated_position.x();
  const double dx_right = ctx.current_estimated_position.x() - xmax;
  const double dy_below = ymin - ctx.current_estimated_position.y();
  const double dy_above = ctx.current_estimated_position.y() - ymax;

  constexpr double BRAKING_SAFETY_MARGIN = 0.05;
  if ((dx_left > 0.0 || dx_right > 0.0) && (dy_below > 0.0 || dy_above > 0.0)) {
    const double corner_x = (dx_left > 0.0) ? xmin : xmax;
    const double corner_y = (dy_below > 0.0) ? ymin : ymax;
    const double cdx = corner_x - ctx.current_estimated_position.x();
    const double cdy = corner_y - ctx.current_estimated_position.y();
    const double dist_to_corner = std::hypot(cdx, cdy);
    if (dist_to_corner > 1e-9) {
      const Eigen::Vector2d corner_dir(cdx / dist_to_corner, cdy / dist_to_corner);
      const double approach = ctx.target_vel.dot(corner_dir);
      if (approach > 0.0) {
        const double effective_dist = std::max(dist_to_corner - BRAKING_SAFETY_MARGIN, 0.0);
        const double v_max = std::sqrt(2.0 * planning_deceleration * effective_dist);
        if (approach > v_max) {
          ctx.target_vel -= (approach - v_max) * corner_dir;
        }
      }
    }
    return;
  }

  if (dx_left > 0.0 && ctx.target_vel.x() > 0.0) {
    const double v_max =
      std::sqrt(2.0 * planning_deceleration * std::max(dx_left - BRAKING_SAFETY_MARGIN, 0.0));
    ctx.target_vel.x() = std::min(ctx.target_vel.x(), v_max);
  }
  if (dx_right > 0.0 && ctx.target_vel.x() < 0.0) {
    const double v_max =
      std::sqrt(2.0 * planning_deceleration * std::max(dx_right - BRAKING_SAFETY_MARGIN, 0.0));
    ctx.target_vel.x() = std::max(ctx.target_vel.x(), -v_max);
  }
  if (dy_below > 0.0 && ctx.target_vel.y() > 0.0) {
    const double v_max =
      std::sqrt(2.0 * planning_deceleration * std::max(dy_below - BRAKING_SAFETY_MARGIN, 0.0));
    ctx.target_vel.y() = std::min(ctx.target_vel.y(), v_max);
  }
  if (dy_above > 0.0 && ctx.target_vel.y() < 0.0) {
    const double v_max =
      std::sqrt(2.0 * planning_deceleration * std::max(dy_above - BRAKING_SAFETY_MARGIN, 0.0));
    ctx.target_vel.y() = std::max(ctx.target_vel.y(), -v_max);
  }
}

auto RVO2Planner::applyPreConstraintStage(
  PreprocessContext & ctx, crane_msgs::msg::RobotCommand & command) const -> void
{
  setPlanningStage(command, "PRE_CONSTRAINT");
  applyCrashAvoidanceConstraint(ctx, command);
  if (!command.local_planner_config.disable_goal_area_avoidance) {
    applyPenaltyAreaBrakingConstraint(ctx, world_model->getOurPenaltyArea(), command);
    applyPenaltyAreaBrakingConstraint(ctx, world_model->getTheirPenaltyArea(), command);
  }
}

auto RVO2Planner::applyRVOInputStage(
  const PreprocessContext & ctx, const crane_msgs::msg::RobotCommand & command) -> void
{
  rvo_sim->setAgentPosition(command.robot_id, toRVO(ctx.current_pose_position));
  rvo_sim->setAgentPrefVelocity(command.robot_id, toRVO(ctx.target_vel));
  rvo_sim->setAgentMaxSpeed(command.robot_id, ctx.max_vel);
  rvo_sim->setAgentTimeHorizonObst(
    command.robot_id, command.local_planner_config.disable_goal_area_avoidance
                        ? 0.0f
                        : PENALTY_AREA_TIME_HORIZON_OBST);
}

auto RVO2Planner::reflectWorldToRVOSim(crane_msgs::msg::RobotCommands & msg) -> void
{
  // ペナルティエリアObstacleの遅延初期化（フィールド情報が確定してから一度だけ）
  if (!penalty_area_obstacles_initialized && world_model->fieldSize().squaredNorm() > 0.01) {
    initializePenaltyAreaObstacles();
  }

  const auto referee_command = world_model->getMsg().play_situation.referee_raw.command.value;
  if (referee_command == robocup_ssl_msgs::msg::RefereeCommand::STOP) {
    for (int i = 0; i < 40; i++) {
      rvo_sim->setAgentMaxSpeed(i, STOP_STATE_MAX_VELOCITY);
    }
  } else {
    for (int i = 0; i < 40; i++) {
      rvo_sim->setAgentMaxSpeed(i, RVO_MAX_SPEED);
    }
  }
  // 味方ロボット：RVO内の位置・速度（＝進みたい方向）の更新
  for (auto & command : msg.robot_commands) {
    if (command.position_target_mode.empty()) {
      RCLCPP_WARN(
        rclcpp::get_logger("rvo2_local_planner"),
        "robot_id=%d has no position_target_mode. skipping.", static_cast<int>(command.robot_id));
      continue;
    }
    auto ctx = createPreprocessContext(command);
    initializePlanningFactors(command);
    applyInputValidation(ctx, command);
    if (!ctx.is_valid) {
      applyRVOInputStage(ctx, command);
      continue;
    }

    setPlanningStage(command, "RVO_INPUT");
    auto vel = std::hypot(command.current_velocity.x, command.current_velocity.y);
    double radius = 0.05f + vel * 0.1f;
    rvo_sim->setAgentRadius(command.robot_id, radius);

    auto robot = world_model->getOurRobot(command.robot_id);
    drawRobotRadiusWithSpeed(visualizer, robot->pose.pos, radius, vel, "yellow");

    if (ctx.run_target_adjustments) {
      applyTargetAdjustmentPipeline(ctx, command);
    } else if (!command.position_target_mode.empty()) {
      auto & pos_mode = command.position_target_mode.front();
      pos_mode.target_x = ctx.target_pos.x();
      pos_mode.target_y = ctx.target_pos.y();
      addOrUpdatePlanningFactor(command, "RVO2TargetAdjustedDistance", "0.000");
    }
    computePreferredVelocityStage(ctx, command, referee_command);
    applyPreConstraintStage(ctx, command);
    setPlanningStage(command, "RVO_INPUT");
    applyRVOInputStage(ctx, command);
  }

  for (const auto & enemy_robot : world_model->theirs().robots) {
    if (enemy_robot->available()) {
      const auto & pos = enemy_robot->pose.pos;
      const auto & vel = enemy_robot->vel.linear;
      rvo_sim->setAgentPosition(enemy_robot->id + 20, toRVO(pos));
      rvo_sim->setAgentPrefVelocity(enemy_robot->id + 20, toRVO(vel));
    } else {
      rvo_sim->setAgentPosition(enemy_robot->id + 20, RVO::Vector2(20.f, 20.f));
      rvo_sim->setAgentPrefVelocity(enemy_robot->id + 20, RVO::Vector2(0.f, 0.f));
    }
  }
}

auto RVO2Planner::extractVelocityCommandsFromRVOSim(
  const crane_msgs::msg::RobotCommands & msg, double theta_offset) -> crane_msgs::msg::RobotCommands
{
  crane_msgs::msg::RobotCommands commands;
  for (const auto & original_command : msg.robot_commands) {
    const auto & robot = world_model->getOurRobot(original_command.robot_id);
    if (original_command.position_target_mode.empty()) {
      continue;
    }
    const auto & original_pos_mode = original_command.position_target_mode.front();

    // 元コマンドを保持したまま下流向け情報を追記する
    crane_msgs::msg::RobotCommand command = original_command;
    command.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
    if (command.polar_velocity_target_mode.empty()) {
      command.polar_velocity_target_mode.emplace_back();
    }

    auto pref_vel = toPoint(rvo_sim->getAgentPrefVelocity(original_command.robot_id));
    auto vel = toPoint(rvo_sim->getAgentVelocity(original_command.robot_id));
    addOrUpdatePlanningFactor(command, "RVO2PrefSpeed", formatPlanningDouble(pref_vel.norm()));

    // 速度修正をトレースに記録（RVO2による修正）
    if (enable_velocity_plan_trace && !command.velocity_plan_trace.empty()) {
      // RVO2による修正を記録
      if ((vel - pref_vel).norm() > 0.01) {  // 1cm/s以上の差がある場合のみ記録
        VelocityPlanTracker::addCorrection(command.velocity_plan_trace[0], "rvo2", pref_vel, vel);
      }
    }

    // 障害物回避を無効にする場合、目標速度をそのまま使う
    if (command.local_planner_config.disable_collision_avoidance) {
      vel = pref_vel;
      if (vel.norm() > rvo_sim->getAgentMaxSpeed(original_command.robot_id)) {
        vel = vel.normalized() * rvo_sim->getAgentMaxSpeed(original_command.robot_id);
      }
      addOrUpdatePlanningFactor(command, "RVO2CollisionAvoidance", "DISABLED");
    } else {
      addOrUpdatePlanningFactor(command, "RVO2CollisionAvoidance", "ENABLED");
    }

    // 位置目標が許容誤差以下の場合、速度目標を0にする
    double distance = std::hypot(
      original_pos_mode.target_x - robot->pose.pos.x(),
      original_pos_mode.target_y - robot->pose.pos.y());
    addOrUpdatePlanningFactor(command, "RVO2DistanceToTarget", formatPlanningDouble(distance));
    addOrUpdatePlanningFactor(
      command, "RVO2PositionTolerance", formatPlanningDouble(original_pos_mode.position_tolerance));
    ZeroVelocityReason zero_velocity_reason = ZeroVelocityReason::NONE;
    if (distance < original_pos_mode.position_tolerance) {
      vel = Velocity::Zero();
      zero_velocity_reason = ZeroVelocityReason::POSITION_TOLERANCE;
    }
    if (zero_velocity_reason == ZeroVelocityReason::NONE && vel.norm() < 1e-4) {
      zero_velocity_reason = (pref_vel.norm() > 1e-3)
                               ? ZeroVelocityReason::RVO_COLLISION_OR_CONSTRAINT
                               : ZeroVelocityReason::PREF_VELOCITY_ZERO;
    }
    addOrUpdatePlanningFactor(command, "RVO2OutputSpeed", formatPlanningDouble(vel.norm()));
    addOrUpdatePlanningFactor(command, "RVO2ZeroVelocityReason", toString(zero_velocity_reason));

    // 座標系の設計について：
    // - target_velocity_thetaはtheta_offsetを含む（half_court_practice_mode対応）
    // - vel.x/y（RVOの出力）はフィールド座標系のまま（theta_offset未適用）
    // - sim_senderで velocity_theta = target_velocity_theta - current_theta により
    //   ロボットローカル座標系に変換される
    command.polar_velocity_target_mode.front().target_velocity_r = vel.norm();
    command.polar_velocity_target_mode.front().target_velocity_theta =
      std::atan2(vel.y(), vel.x()) + theta_offset;

    // 効率的な加速のための回転制御
    if (command.local_planner_config.enable_rotation_stop_on_accel) {
      double move_angle = std::atan2(vel.y(), vel.x());
      double angle_diff = getAngleDiff(robot->pose.theta, move_angle);

      constexpr double ANGLE_THRESHOLD = 15.0 * M_PI / 180.0;  // 15度
      bool is_forward_or_backward =
        (std::abs(angle_diff) <= ANGLE_THRESHOLD) ||                 // 前方
        (std::abs(std::abs(angle_diff) - M_PI) <= ANGLE_THRESHOLD);  // 後方

      double current_speed = robot->vel.linear.norm();
      double target_speed = vel.norm();
      double max_speed = rvo_sim->getAgentMaxSpeed(original_command.robot_id);

      bool is_accelerating = current_speed < target_speed;
      bool is_low_speed = current_speed <= max_speed * 0.5;

      // 加速初期段階かつ前後方向に向いている場合、回転を停止
      if (is_forward_or_backward && is_low_speed && is_accelerating && target_speed > 0.01) {
        command.omega_limit = 0.0;
      }
    }

    commands.robot_commands.emplace_back(command);
  }

  pre_commands = msg;
  return commands;
}

auto RVO2Planner::calculateRobotCommand(
  const crane_msgs::msg::RobotCommands & msg, double theta_offset) -> crane_msgs::msg::RobotCommands
{
  crane_msgs::msg::RobotCommands commands = msg;
  reflectWorldToRVOSim(commands);
  // RVOシミュレータ更新
  rvo_sim->doStep();
  return extractVelocityCommandsFromRVOSim(commands, theta_offset);
}

auto RVO2Planner::overrideTargetPosition(crane_msgs::msg::RobotCommands & msg) -> void
{
  const auto referee_command = world_model->getMsg().play_situation.referee_raw.command.value;
  for (auto & command : msg.robot_commands) {
    if (command.position_target_mode.empty()) {
      continue;
    }

    initializePlanningFactors(command);
    auto ctx = createPreprocessContext(command);
    applyInputValidation(ctx, command);
    if (!ctx.is_valid) {
      continue;
    }

    if (ctx.run_target_adjustments) {
      applyTargetAdjustmentPipeline(ctx, command);
    } else if (!command.position_target_mode.empty()) {
      auto & pos_mode = command.position_target_mode.front();
      pos_mode.target_x = ctx.target_pos.x();
      pos_mode.target_y = ctx.target_pos.y();
      addOrUpdatePlanningFactor(command, "RVO2TargetAdjustedDistance", "0.000");
    }
  }
}

auto RVO2Planner::adjustForFieldBoundary(
  Point & target_pos, const Point & current_pos,
  const crane_msgs::msg::RobotCommand & command) const -> void
{
  if (command.local_planner_config.disable_field_boundary) {
    return;
  }
  const double max_x = world_model->fieldSize().x() / 2.0 + FIELD_BOUNDARY_OFFSET;
  const double max_y = world_model->fieldSize().y() / 2.0 + FIELD_BOUNDARY_OFFSET;

  // フィールド境界のBox
  Box field_box;
  field_box.min_corner() << -max_x, -max_y;
  field_box.max_corner() << max_x, max_y;

  // 目標位置がフィールド内ならそのまま
  if (isInBox(field_box, target_pos)) {
    return;
  }

  // 現在位置から目標位置への線分
  Segment move_line(current_pos, target_pos);

  // フィールド境界の4辺
  Segment top_edge(Point(-max_x, max_y), Point(max_x, max_y));
  Segment bottom_edge(Point(-max_x, -max_y), Point(max_x, -max_y));
  Segment right_edge(Point(max_x, -max_y), Point(max_x, max_y));
  Segment left_edge(Point(-max_x, -max_y), Point(-max_x, max_y));

  // 各辺との交点を計算
  std::vector<Point> all_intersections;
  for (const auto & edge : {top_edge, bottom_edge, right_edge, left_edge}) {
    auto intersections = getIntersections(move_line, edge);
    all_intersections.insert(all_intersections.end(), intersections.begin(), intersections.end());
  }

  // 現在位置に最も近い交点を選択（最初に交差する点）
  if (!all_intersections.empty()) {
    auto closest = std::min_element(
      all_intersections.begin(), all_intersections.end(),
      [&current_pos](const Point & a, const Point & b) {
        return bg::distance(a, current_pos) < bg::distance(b, current_pos);
      });
    target_pos = *closest;
  } else {
    // 交点がない場合（現在位置がフィールド外など）、単純なクランプにフォールバック
    target_pos.x() = std::clamp(target_pos.x(), -max_x, max_x);
    target_pos.y() = std::clamp(target_pos.y(), -max_y, max_y);
  }
}

auto RVO2Planner::adjustForPenaltyAreaAvoidance(
  Point & target_pos, const Point & current_pos, crane_msgs::msg::RobotCommand & command) -> void
{
  if (not command.local_planner_config.disable_goal_area_avoidance) {
    const double penalty_area_offset =
      needsExpandedPenaltyAreaOffset(world_model->getMsg().play_situation.command.value)
        ? PENALTY_AREA_OFFSET_STOP
        : PENALTY_AREA_OFFSET;

    auto avoidPenaltyArea = [&](const Box & penalty_area, const Point & goal_pos) {
      constexpr int MAX_ITERATIONS = 100;
      if (isInBox(penalty_area, current_pos, penalty_area_offset)) {
        if (std::abs(current_pos.x()) > world_model->fieldSize().x() / 2.0) {
          target_pos.x() = std::copysign(world_model->fieldSize().x() / 2.0, target_pos.x());
        }
        // stopHere()等でtarget_pos == current_posの場合（ペナルティエリア内で停止指示）、
        // ゴールから離れる方向に初期目標を設定することで脱出ループを有効化する
        if ((target_pos - current_pos).norm() < 1e-6) {
          target_pos = current_pos + (current_pos - goal_pos).normalized() * 0.05;
        }
        // 目標点をペナルティエリアの外に出るようにする (反復上限で無限ループ防止)
        for (int iter = 0;
             iter < MAX_ITERATIONS && isInBox(penalty_area, target_pos, penalty_area_offset);
             ++iter) {
          target_pos += (target_pos - current_pos).normalized() * 0.05;  // 5cmずつ離れていく
        }
      } else if (isInBox(penalty_area, target_pos, penalty_area_offset)) {
        // ペナルティエリア内にいる場合は、ペナルティエリアの外に出るようにする
        if (std::abs(target_pos.x()) > world_model->fieldSize().x() / 2.0) {
          target_pos.x() = std::copysign(world_model->fieldSize().x() / 2.0, target_pos.x());
        }
        // 目標点をペナルティエリアの外に出るようにする
        for (int iter = 0;
             iter < MAX_ITERATIONS && isInBox(penalty_area, target_pos, penalty_area_offset) &&
             target_pos != goal_pos;
             ++iter) {
          target_pos += (target_pos - goal_pos).normalized() * 0.05;
        }
      }

      const auto decision = computePenaltyBypassDecision(
        current_pos, target_pos, penalty_area, goal_pos, world_model->penaltyAreaSize(),
        penalty_area_offset, PENALTY_AREA_SURROUNDING_OFFSET,
        PENALTY_AREA_FORCE_WAYPOINT_ON_CROSSING);
      if (!decision.crossing_detected) {
        return;
      }

      addOrUpdatePlanningFactor(command, "RVO2PenaltyCrossingDetected", "1");
      addOrUpdatePlanningFactor(command, "RVO2PenaltyBypassSide", toString(decision.selected_side));

      if (decision.target_overridden) {
        target_pos = decision.waypoint;
      }
    };

    // 自陣と敵陣の両方のペナルティエリアを回避
    avoidPenaltyArea(world_model->getOurPenaltyArea(), world_model->getOurGoalCenter());
    avoidPenaltyArea(world_model->getTheirPenaltyArea(), world_model->getTheirGoalCenter());
  }
}

auto RVO2Planner::adjustForBallAvoidance(
  Point & target_pos, const Point & current_pos,
  const crane_msgs::msg::RobotCommand & command) const -> void
{
  if (not command.local_planner_config.disable_ball_avoidance) {
    const auto & ball_pos = world_model->ball().pos;
    const double MIN_BALL_DISTANCE = [&]() {
      switch (world_model->getMsg().play_situation.command.value) {
        case crane_msgs::msg::PlaySituation::THEIR_DIRECT_FREE:
          return 0.7;
        case crane_msgs::msg::PlaySituation::STOP:
          return 0.5;
        default:
          return 0.2;
      }
    }();
    if ((target_pos - ball_pos).norm() < MIN_BALL_DISTANCE) {
      target_pos = ball_pos + (target_pos - ball_pos).normalized() * MIN_BALL_DISTANCE;
    }
    if ((current_pos - ball_pos).norm() < MIN_BALL_DISTANCE) {
      // 現在位置が近い場合は、最優先で離れる
      target_pos = ball_pos + (current_pos - ball_pos).normalized() * (MIN_BALL_DISTANCE + 0.05);
    } else {
      Segment move_line(current_pos, target_pos);
      auto [distance, closest_point] = getClosestPointAndDistance(ball_pos, move_line);
      if (
        closest_point != ball_pos && closest_point != target_pos && distance < MIN_BALL_DISTANCE) {
        // 少しずらす
        target_pos = ball_pos + (closest_point - ball_pos).normalized() * MIN_BALL_DISTANCE;
      }
    }
  }
}

auto RVO2Planner::adjustForPlacementAvoidance(
  Point & target_pos, const Point & current_pos,
  const crane_msgs::msg::RobotCommand & command) const -> void
{
  if (
    not command.local_planner_config.disable_placement_avoidance &&
    world_model->getBallPlacementTarget().has_value()) {
    const auto placement_area_opt = world_model->getBallPlacementArea();
    if (!placement_area_opt) return;
    const auto & placement_area = placement_area_opt.value();

    auto isInPlacementArea = [&placement_area](const Point & point, double offset) {
      return bg::distance(point, placement_area) <= placement_area.radius + offset;
    };

    if (isInPlacementArea(current_pos, 0.2)) {
      auto [distance, closest_point] =
        getClosestPointAndDistance(placement_area.segment, current_pos);
      // 0.6m離れる
      Point target_position = closest_point + (current_pos - closest_point).normalized() * 0.8;
      if (not world_model->point_checker.isFieldInside(target_position, 0.2)) {
        // 一番近いフィールド外のポイントがだめなので逆方向に0.6m離れる
        target_position = closest_point + (closest_point - current_pos).normalized() * 0.8;

        if (
          const auto & segment = placement_area.segment;
          (closest_point == segment.first || closest_point == segment.second)) {
          // 一番近い点が端点の場合は単純に反対側の点を選択するだけではだめなので、
          // 垂直方向に0.6m離れた点を複数選択して、フィールド内かつ配置エリア外の点を選択する
          Vector2 vertical_vec =
            getVerticalVec((segment.second - segment.first).normalized()) * 0.8;
          std::array<Point, 2> target_candidates = {
            closest_point + vertical_vec, closest_point - vertical_vec};

          if (
            auto target = std::ranges::find_if(
              target_candidates,
              [&](const auto & target_candidate) {
                return (
                  world_model->point_checker.isFieldInside(target_candidate, 0.2) &&
                  not isInPlacementArea(target_candidate, 0.1));
              });
            target != target_candidates.end()) {
            target_pos = *target;
          } else {
            // 垂直方向の2候補もだめな場合は放射状8方向で有効点を探索する
            std::array<Point, 8> radial_candidates;
            for (int i = 0; i < 8; i++) {
              double angle = i * M_PI / 4.0;
              radial_candidates[i] = closest_point + Point(std::cos(angle), std::sin(angle)) * 0.8;
            }
            auto valid = std::ranges::find_if(radial_candidates, [&](const auto & c) {
              return world_model->point_checker.isFieldInside(c, 0.2) &&
                     not isInPlacementArea(c, 0.1);
            });
            if (valid != radial_candidates.end()) {
              target_pos = *valid;
            } else {
              // 最終フォールバック: 移動しない
              target_pos = current_pos;
            }
          }
        } else {
          target_pos = target_position;
        }
      } else {
        target_pos = target_position;
      }
      // 安全チェック: 結果がフィールド外にならないようにクランプ
      if (not world_model->point_checker.isFieldInside(target_pos, 0.2)) {
        const double max_x = world_model->fieldSize().x() / 2.0 + FIELD_BOUNDARY_OFFSET;
        const double max_y = world_model->fieldSize().y() / 2.0 + FIELD_BOUNDARY_OFFSET;
        target_pos.x() = std::clamp(target_pos.x(), -max_x, max_x);
        target_pos.y() = std::clamp(target_pos.y(), -max_y, max_y);
      }
    }
  }
}
}  // namespace crane
