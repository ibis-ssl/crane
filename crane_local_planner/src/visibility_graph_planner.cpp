// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/visibility_graph_planner.hpp"

#include <algorithm>
#include <cmath>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_msg_wrappers/command_wrapper_base.hpp>
#include <crane_msgs/msg/play_situation.hpp>
#include <crane_utils/parameter.hpp>
#include <limits>
#include <robocup_ssl_msgs/msg/referee.hpp>

namespace crane
{
namespace
{
auto expandedPenaltyAreaForAvoidance(const Box & source, const Point & goal_center, double offset)
  -> Box
{
  constexpr double FAR = 20.0;
  Box result = source;
  result.min_corner().y() -= offset;
  result.max_corner().y() += offset;
  if (goal_center.x() < 0.0) {
    // 負側のゴール: フィールド外側（ゴール裏）は -x 方向。
    // ゴール裏側を -FAR まで拡張し、ゴールの後ろを通り抜ける迂回経路を塞ぐ。
    result.min_corner().x() = -FAR;
    result.max_corner().x() += offset;
  } else {
    // 正側のゴール: フィールド外側（ゴール裏）は +x 方向。
    result.min_corner().x() -= offset;
    result.max_corner().x() = FAR;
  }
  return result;
}

auto closestPointOnSegment(const Point & point, const Point & from, const Point & to) -> Point
{
  const Vector2 segment = to - from;
  if (segment.squaredNorm() < 1e-9) {
    return from;
  }
  const double ratio = std::clamp((point - from).dot(segment) / segment.squaredNorm(), 0.0, 1.0);
  return from + ratio * segment;
}
}  // namespace

VisibilityGraphPlanner::VisibilityGraphPlanner(rclcpp::Node & node)
: LocalPlannerBase("visibility_graph_planner", node)
{
  crane::get_or_declare_parameter(node, "max_vel", max_velocity_);
  crane::get_or_declare_parameter(node, "stop_state_max_velocity", stop_state_max_velocity_);
  crane::get_or_declare_parameter(node, "field_boundary_offset", field_boundary_offset_);
  crane::get_or_declare_parameter(node, "penalty_area_offset", penalty_area_offset_);
  crane::get_or_declare_parameter(node, "penalty_area_offset_stop", penalty_area_offset_stop_);

  crane::get_or_declare_parameter(node, "visibility_graph.prediction_horizon", prediction_horizon_);
  crane::get_or_declare_parameter(node, "visibility_graph.safety_margin", safety_margin_);
  crane::get_or_declare_parameter(node, "visibility_graph.lookahead_distance", lookahead_distance_);
  crane::get_or_declare_parameter(
    node, "visibility_graph.replan_cross_track_distance", replan_cross_track_distance_);
  crane::get_or_declare_parameter(
    node, "visibility_graph.route_switch_improvement_ratio", route_switch_improvement_ratio_);
  crane::get_or_declare_parameter(
    node, "visibility_graph.goal_change_threshold", goal_change_threshold_);
  crane::get_or_declare_parameter(
    node, "visibility_graph.full_replan_interval", full_replan_interval_);

  int circle_samples = crane::get_or_declare_parameter(node, "visibility_graph.circle_samples", 12);
  int capsule_end_samples =
    crane::get_or_declare_parameter(node, "visibility_graph.capsule_end_samples", 6);
  visibility_graph_.configure({circle_samples, capsule_end_samples, 1e-3});
}

auto VisibilityGraphPlanner::buildObstacles(
  uint8_t robot_id, const crane_msgs::msg::RobotCommand & command) const
  -> std::vector<visibility_graph::Obstacle>
{
  // 味方と敵のロボットの障害物設定 移動速度に応じて
  std::vector<visibility_graph::Obstacle> obstacles;
  const auto ego = world_model->getOurRobot(robot_id);
  const Vector2 ego_velocity(command.current_velocity.x, command.current_velocity.y);
  if (!command.local_planner_config.disable_collision_avoidance) {
    auto append_robot = [&](const auto & robot) {
      if (!robot->available() || robot == ego) {
        return;
      }
      obstacles.push_back(
        visibility_graph::makePredictedRobotObstacle(
          ego_velocity, ego->geometry().radius, robot->pose.pos, robot->vel.linear,
          robot->geometry().radius, prediction_horizon_, safety_margin_));
    };
    for (const auto & robot : world_model->ours().robots) {
      append_robot(robot);
    }
    for (const auto & robot : world_model->theirs().robots) {
      append_robot(robot);
    }
  }

  // ペナルティエリアの障害物設定 STOPのときはマージンを変える。
  if (
    !command.local_planner_config.disable_goal_area_avoidance &&
    world_model->getMsg().play_situation.command.value != crane_msgs::msg::PlaySituation::HALT) {
    const bool stop = world_model->getMsg().play_situation.referee_raw.command.value ==
                      robocup_ssl_msgs::msg::RefereeCommand::STOP;
    const double offset = stop ? penalty_area_offset_stop_ : penalty_area_offset_;
    obstacles.push_back(
      visibility_graph::Obstacle::makeBox(expandedPenaltyAreaForAvoidance(
        world_model->getOurPenaltyArea(), world_model->getOurGoalCenter(), offset)));
    obstacles.push_back(
      visibility_graph::Obstacle::makeBox(expandedPenaltyAreaForAvoidance(
        world_model->getTheirPenaltyArea(), world_model->getTheirGoalCenter(), offset)));
  }

  // シチュエーションに応じたボールの障害物設定
  if (!command.local_planner_config.disable_ball_avoidance) {
    double radius = 0.2;
    switch (world_model->getMsg().play_situation.command.value) {
      case crane_msgs::msg::PlaySituation::THEIR_DIRECT_FREE:
        radius = 0.7;
        break;
      case crane_msgs::msg::PlaySituation::STOP:
      case crane_msgs::msg::PlaySituation::STOP_PRE_OUR_KICKOFF_PREPARATION:
      case crane_msgs::msg::PlaySituation::STOP_PRE_THEIR_KICKOFF_PREPARATION:
      case crane_msgs::msg::PlaySituation::STOP_PRE_OUR_PENALTY_PREPARATION:
      case crane_msgs::msg::PlaySituation::STOP_PRE_THEIR_PENALTY_PREPARATION:
      case crane_msgs::msg::PlaySituation::STOP_PRE_OUR_DIRECT_FREE:
      case crane_msgs::msg::PlaySituation::STOP_PRE_THEIR_DIRECT_FREE:
      case crane_msgs::msg::PlaySituation::STOP_PRE_FORCE_START:
      case crane_msgs::msg::PlaySituation::OUR_KICKOFF_PREPARATION:
      case crane_msgs::msg::PlaySituation::THEIR_KICKOFF_PREPARATION:
        radius = 0.5;
        break;
      default:
        break;
    }
    obstacles.push_back(visibility_graph::Obstacle::makeCircle(world_model->ball().pos, radius));
  }

  // ボールと、配置先の間の障害物設定（ボール配置時のみ）
  if (
    !command.local_planner_config.disable_placement_avoidance &&
    world_model->getBallPlacementTarget().has_value()) {
    if (const auto placement = world_model->getBallPlacementArea(); placement.has_value()) {
      obstacles.push_back(visibility_graph::Obstacle::makeCapsule(*placement));
    }
  }

  // フィールドの外を障害物として設定
  if (!command.local_planner_config.disable_field_boundary) {
    const double half_width = world_model->fieldSize().x() / 2.0 + field_boundary_offset_;
    const double half_height = world_model->fieldSize().y() / 2.0 + field_boundary_offset_;
    constexpr double FAR = 20.0;
    obstacles.push_back(
      visibility_graph::Obstacle::makeBox(Box(Point(-FAR, half_height), Point(FAR, FAR))));
    obstacles.push_back(
      visibility_graph::Obstacle::makeBox(Box(Point(-FAR, -FAR), Point(FAR, -half_height))));
    obstacles.push_back(
      visibility_graph::Obstacle::makeBox(Box(Point(half_width, -FAR), Point(FAR, FAR))));
    obstacles.push_back(
      visibility_graph::Obstacle::makeBox(Box(Point(-FAR, -FAR), Point(-half_width, FAR))));
  }
  return obstacles;
}

auto VisibilityGraphPlanner::trimPathFromCurrent(
  const Point & current, const std::vector<Point> & path) -> std::vector<Point>
{
  if (path.size() < 2) {
    return {};
  }
  size_t best_segment = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  Point best_projection = current;
  for (size_t i = 1; i < path.size(); ++i) {
    const Point projection = closestPointOnSegment(current, path[i - 1], path[i]);
    const double distance = (current - projection).norm();
    if (distance < best_distance) {
      best_distance = distance;
      best_segment = i;
      best_projection = projection;
    }
  }
  std::vector<Point> result{current};
  if ((best_projection - current).norm() > 1e-4) {
    result.push_back(best_projection);
  }
  result.insert(result.end(), path.begin() + static_cast<std::ptrdiff_t>(best_segment), path.end());
  return result;
}

auto VisibilityGraphPlanner::selectPath(
  uint8_t robot_id, const Point & current, const Point & goal,
  const std::vector<visibility_graph::Obstacle> & obstacles) -> std::vector<Point>
{
  // 移動ロボットの障害物回避を最優先
  auto & state = path_states_.at(robot_id);
  if (const auto escape = visibility_graph_.nearestDynamicEscape(current, obstacles)) {
    state.path = {current, *escape};
    state.goal = goal;
    state.valid = false;
    return state.path;
  }

  // 前回経路が、回避動作ではない(valid == true) & 目標位置が変わっていない場合 保持経路再利用
  std::vector<Point> retained;
  if (state.valid && (goal - state.goal).norm() <= goal_change_threshold_) {
    // 一番近い経路上の点を求めて、目標位置までの経路を切り出す。
    double cross_track_distance = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < state.path.size(); ++i) {
      cross_track_distance = std::min(
        cross_track_distance,
        (current - closestPointOnSegment(current, state.path[i - 1], state.path[i])).norm());
    }
    retained = trimPathFromCurrent(current, state.path);
    if (!retained.empty()) {
      retained.back() = goal;
    }

    // 経路が2点未満、または次の移動先と現在位置が離れているときは経路を破棄
    if (
      retained.size() < 2 || cross_track_distance > replan_cross_track_distance_ ||
      !visibility_graph_.isPathVisible(retained, obstacles)) {
      retained.clear();
    }
  }

  // 保持経路が安全な間は高コストな全グラフ再計算を周期的に限定する。ただし、
  // 障害物が直線経路から退いた場合は即座に最短の直線へ戻す。

  // 直線経路の干渉を確認する
  const std::vector<Point> direct_path{current, goal};
  const bool direct_path_visible = visibility_graph_.isPathVisible(direct_path, obstacles);
  // 経路再生成をする時刻になったか
  const auto now = std::chrono::steady_clock::now();
  const bool full_replan_due = now >= state.next_full_replan;
  // 経路再計画の戦略を決定
  const auto action =
    visibility_graph::decideReplanAction(!retained.empty(), direct_path_visible, full_replan_due);

  // 直線経路を採用
  if (action == visibility_graph::ReplanAction::USE_DIRECT_PATH) {
    state.path = direct_path;
    state.goal = goal;
    state.valid = true;
    return state.path;
  }

  // 経路を再利用
  if (action == visibility_graph::ReplanAction::REUSE_RETAINED_PATH) {
    state.path = retained;
    state.goal = goal;
    state.valid = true;
    return state.path;
  }

  // 経路を再計算
  const auto new_path = visibility_graph_.plan(current, goal, obstacles);
  const double stagger = 0.02 * static_cast<double>(robot_id);
  state.next_full_replan =
    now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(std::max(0.0, full_replan_interval_) + stagger));

  std::vector<Point> selected;
  if (!retained.empty() && new_path.has_value()) {
    // 経路の切り替えは、保持経路よりも新しい経路が十分に短い場合のみ行う。
    const double retained_length = visibility_graph::pathLength(retained);
    const double new_length = visibility_graph::pathLength(*new_path);
    selected =
      new_length < retained_length * (1.0 - route_switch_improvement_ratio_) ? *new_path : retained;
  } else if (!retained.empty()) {
    selected = retained;
  } else if (new_path.has_value()) {
    selected = *new_path;
  } else {
    selected = {current, current};
  }

  state.path = selected;
  state.goal = goal;
  state.valid = selected.size() >= 2;
  return selected;
}

auto VisibilityGraphPlanner::pointAtDistance(const std::vector<Point> & path, double distance)
  -> Point
{
  if (path.empty()) {
    return Point::Zero();
  }
  double remaining = std::max(0.0, distance);
  for (size_t i = 1; i < path.size(); ++i) {
    const Vector2 segment = path[i] - path[i - 1];
    const double length = segment.norm();
    if (length > 1e-9 && remaining <= length) {
      return path[i - 1] + segment * (remaining / length);
    }
    remaining -= length;
  }
  return path.back();
}

auto VisibilityGraphPlanner::planSingleRobot(
  const crane_msgs::msg::RobotCommand & command, double theta_offset)
  -> crane_msgs::msg::RobotCommand
{
  crane_msgs::msg::RobotCommand result = command;
  result.control_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
  if (result.position_target_mode.empty()) {
    result.position_target_mode.emplace_back();
  }
  if (command.position_target_mode.empty() || command.robot_id >= path_states_.size()) {
    auto & output = result.position_target_mode.front();
    output.target_x = command.current_pose.x;
    output.target_y = command.current_pose.y;
    output.terminal_velocity_x = 0.0;
    output.terminal_velocity_y = 0.0;
    output.speed_limit_at_target = 0.0;
    return result;
  }

  const Point current(command.current_pose.x, command.current_pose.y);
  const auto & input = command.position_target_mode.front();
  Point goal(input.target_x, input.target_y);
  if (!command.local_planner_config.disable_field_boundary) {
    const double half_width = world_model->fieldSize().x() / 2.0 + field_boundary_offset_;
    const double half_height = world_model->fieldSize().y() / 2.0 + field_boundary_offset_;
    goal = clampPoint(goal, half_width, half_height);
  }
  auto obstacles = buildObstacles(command.robot_id, command);
  const auto path = selectPath(command.robot_id, current, goal, obstacles);
  const double remaining_distance = visibility_graph::pathLength(path);

  // 次の移動先をサブゴールとして設定する。サブゴールまでの経路が鑑賞する場合は、経路上の次の点をサブゴールとする。
  Point subgoal = pointAtDistance(path, lookahead_distance_);
  // pointAtDistance は経路長を超える距離を渡すと終点を返すので、subgoal までの弧長は
  // min(lookahead, 全長) になる。迂回時のフォールバックでは path[1] までの直線長。
  double subgoal_arc_length = std::min(lookahead_distance_, remaining_distance);
  if (!visibility_graph_.isPathVisible({current, subgoal}, obstacles) && path.size() >= 2) {
    subgoal = path[1];
    subgoal_arc_length = (path[1] - path[0]).norm();
  }

  const bool final_target = (subgoal - path.back()).norm() < 1e-4;

  result.local_planner_config.max_velocity_factors.emplace_back(
    crane_msgs::msg::NamedFloat()
      .set__name("VisibilityGraphPlanner::max_vel")
      .set__value(max_velocity_));
  const auto referee_command = world_model->getMsg().play_situation.referee_raw.command.value;
  if (
    referee_command == robocup_ssl_msgs::msg::RefereeCommand::STOP &&
    !world_model->isPracticeNormalSpeed()) {
    result.local_planner_config.max_velocity_factors.emplace_back(
      crane_msgs::msg::NamedFloat()
        .set__name("VisibilityGraphPlanner STOP制限")
        .set__value(stop_state_max_velocity_));
  }
  const double max_velocity = resolveMaxVelocityFactors(result, max_velocity_);
  resolveMaxAccelerationFactors(result, planning_acceleration);

  auto & output = result.position_target_mode.front();
  output.target_x = subgoal.x();
  output.target_y = subgoal.y();

  // スキルが宣言した「最終目標に到達した瞬間の速度」。
  // 指定経路が 2 つある: setSpeedLimitAtTarget() は position_target_mode に、
  // setTerminalVelocity() は local_planner_config に書く。後者はこれまで
  // どこからも読まれておらず、スキルの意図が黙って捨てられていた。
  // どちらも既定 0（= 目標で止まる）なので、大きい方を採って両方を活かす。
  const double goal_terminal_speed = std::max(
    {0.0, static_cast<double>(input.speed_limit_at_target),
     static_cast<double>(command.local_planner_config.terminal_velocity)});

  // subgoal を通過してよい速度は「subgoal から先に残っている距離」で決まる。
  // 全残距離で計算すると、subgoal が最終目標のすぐ手前にあるときに過大な通過速度を
  // 許してしまい、final_target に切り替わった瞬間に終端速度が goal_terminal_speed へ
  // 段差で落ちる。lookahead_distance 付近で指令速度が数倍跳ぶのはこれが原因で、
  // 機体は急制動したまま目標手前で止まる。ここを連続にしておくこと。
  const double distance_after_subgoal =
    final_target ? 0.0 : std::max(0.0, remaining_distance - subgoal_arc_length);
  const double terminal_speed = std::min(
    max_velocity, std::sqrt(
                    goal_terminal_speed * goal_terminal_speed +
                    2.0 * planning_deceleration * distance_after_subgoal));

  // 終端速度ベクトルの向き: 中継点では次の経路区間、最終目標では現在位置からの接近方向。
  Vector2 direction = subgoal - current;
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    if ((subgoal - path[i]).norm() < 1e-4) {
      direction = path[i + 1] - path[i];
      break;
    }
  }
  if (direction.norm() > 1e-6) {
    direction.normalize();
  } else {
    direction.setZero();
  }

  output.position_tolerance =
    final_target ? input.position_tolerance : std::min(input.position_tolerance, 0.02f);
  output.speed_limit_at_target = terminal_speed;
  output.terminal_velocity_x = direction.x() * terminal_speed;
  output.terminal_velocity_y = direction.y() * terminal_speed;

  addOrUpdatePlanningFactor(
    result, "VisibilityGraphStatus", remaining_distance < 1e-6 ? "HOLD" : "OK");
  addOrUpdatePlanningFactor(
    result, "VisibilityGraphPathLength", formatPlanningDouble(remaining_distance));
  addOrUpdatePlanningFactor(result, "VisibilityGraphNodes", std::to_string(path.size()));
  // 「残距離いくつのとき終端速度をいくつで指令したか」を bag だけで追えるようにする。
  // これが無いと、機体が目標手前で止まったとき指令が原因か機体側が原因か切り分けられない。
  addOrUpdatePlanningFactor(result, "VisibilityGraphFinalTarget", final_target ? "1" : "0");
  addOrUpdatePlanningFactor(
    result, "VisibilityGraphSubgoalArc", formatPlanningDouble(subgoal_arc_length));
  addOrUpdatePlanningFactor(
    result, "VisibilityGraphTerminalSpeed", formatPlanningDouble(terminal_speed));
  addOrUpdatePlanningFactor(
    result, "VisibilityGraphGoalTerminalSpeed", formatPlanningDouble(goal_terminal_speed));

  for (const auto & obstacle : obstacles) {
    if (obstacle.type == visibility_graph::Obstacle::Type::CAPSULE && obstacle.is_dynamic_robot) {
      visualizer->drawLine(
        obstacle.capsule.segment.first, obstacle.capsule.segment.second, "red", 12, 0.35);
      visualizer->drawCircle(
        obstacle.capsule.segment.first, obstacle.capsule.radius, "red", 6, 0.15);
      visualizer->drawCircle(
        obstacle.capsule.segment.second, obstacle.capsule.radius, "red", 6, 0.15);
    }
  }
  visualizer->drawPolyline(path, "cyan", 0.8, 12.0);
  visualizer->drawFilledCircle(subgoal, 0.04, "orange", 0.8);
  (void)theta_offset;
  return result;
}

auto VisibilityGraphPlanner::calculateRobotCommand(
  const crane_msgs::msg::RobotCommands & msg, double theta_offset) -> crane_msgs::msg::RobotCommands
{
  crane_msgs::msg::RobotCommands result;
  result.header = msg.header;
  result.on_positive_half = msg.on_positive_half;
  result.is_yellow = msg.is_yellow;
  result.robot_commands.reserve(msg.robot_commands.size());
  for (const auto & command : msg.robot_commands) {
    result.robot_commands.push_back(planSingleRobot(command, theta_offset));
  }
  return result;
}

}  // namespace crane
