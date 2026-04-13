// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <cmath>
#include <crane_geometry/ddps.hpp>
#include <crane_physics/position_assignments.hpp>
#include <crane_sessions/forward_session.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/transform.hpp>

namespace crane
{

auto ForwardSession::computeCandidatePoints() const -> std::vector<Point>
{
  const double goal_line_x = world_model->fieldSize().x() * 0.5;
  const double field_half_width = world_model->fieldSize().y() * 0.5;
  constexpr double PA_MARGIN = 0.3;  // ペナルティエリアとの距離マージン

  // 攻撃サイドの符号（1 or -1）
  const double side_sign = -world_model->getAttackSideSign();

  // 攻撃半面のx範囲（センターライン手前0.5mからゴール前1.0m）
  // side_sign > 0: +x方向が敵陣 → [0.5, goal_line_x-1.0]
  // side_sign < 0: -x方向が敵陣 → [-(goal_line_x-1.0), -0.5]
  const double x_near = side_sign * 0.5;
  const double x_far = side_sign * (goal_line_x - 1.0);
  const double x_min = std::min(x_near, x_far);
  const double x_max = std::max(x_near, x_far);

  // グリッドの中心点（攻撃エリア中央）
  const double center_x = (x_min + x_max) * 0.5;
  const double center_y = 0.0;

  // x, y 方向の格子数を算出
  constexpr float GRID_STEP = 0.5f;
  const int nx = static_cast<int>(std::ceil((x_max - x_min) / GRID_STEP));
  const int ny = static_cast<int>(std::ceil(2.0 * field_half_width / GRID_STEP));

  std::vector<Point> raw = getPoints(Point(center_x, center_y), GRID_STEP, GRID_STEP, nx, ny);

  // フィールド外・PAの中（マージン込み）を除外し、x が攻撃半面内のみ残す
  std::vector<Point> candidates;
  candidates.reserve(raw.size());
  for (const auto & p : raw) {
    if (!world_model->point_checker.isFieldInside(p, 0.2)) continue;
    if (world_model->point_checker.isPenaltyArea(p, PA_MARGIN)) continue;
    // 攻撃半面（センターラインより敵陣側）のみ
    if (side_sign * p.x() < 0.3) continue;
    candidates.push_back(p);
  }
  return candidates;
}

auto ForwardSession::scorePoint(
  const Point & p, const Point & robot_pos, const std::vector<Point> & forward_positions,
  const RobotList & available_enemies) const -> double
{
  double score = 1.0;
  const auto & ball = world_model->ball();
  const auto their_goal_center = world_model->getAttackGoalCenter();

  // ---- 1. パス受取スコア（パスラインの敵距離） ----
  Segment pass_line{ball.pos, p};
  auto nearest_enemy =
    world_model->getNearestRobotWithDistanceFromSegment(pass_line, available_enemies);
  if (nearest_enemy) {
    score *= std::clamp(nearest_enemy->distance, 0.2, 2.0) / 2.0;
  }

  // ---- 2. ボール距離（近すぎ/遠すぎ回避） ----
  const double dist_to_ball = (p - ball.pos).norm();
  constexpr double MIN_PASS_DIST = 1.5;
  constexpr double MAX_PASS_DIST = 9.0;
  if (dist_to_ball < MIN_PASS_DIST) {
    score *= dist_to_ball / MIN_PASS_DIST;
  }
  score *= (std::clamp(1.0 - dist_to_ball / MAX_PASS_DIST, 0.0, 1.0) * 0.5 + 0.5);

  // ---- 3. ゴール可視角 ----
  auto [goal_angle, goal_width] = world_model->getLargestAttackGoalAngleRangeFromPoint(p);
  score *= (std::clamp(goal_width / 0.6, 0.0, 1.0) * 0.5 + 0.5);

  // ---- 4. ボール後方回避（ゴール方向の後ろ側を減点） ----
  const double dot = (their_goal_center - ball.pos).normalized().dot((p - ball.pos).normalized());
  score *= std::max((dot + 0.5), 0.0);

  // ---- 5. 斥力（味方スペーシング） ----
  constexpr double MIN_TEAM_SPACING = 1.8;
  for (const auto & fwd_pos : forward_positions) {
    if ((fwd_pos - robot_pos).norm() < 0.01) continue;  // 自分自身をスキップ
    const double d = (p - fwd_pos).norm();
    if (d < MIN_TEAM_SPACING) {
      score *= d / MIN_TEAM_SPACING;
    }
  }

  // ---- 6. 近似ボロノイ（縄張り評価） ----
  // 「自分が最近傍」なほどスコアが上がり、他ロボットに近い点は減点される
  const double own_dist = (p - robot_pos).norm() + 1e-6;
  double others_min = 1e9;
  for (const auto & fwd_pos : forward_positions) {
    if ((fwd_pos - robot_pos).norm() < 0.01) continue;
    others_min = std::min(others_min, (p - fwd_pos).norm());
  }
  for (const auto & enemy : available_enemies) {
    others_min = std::min(others_min, (p - enemy->pose.pos).norm());
  }
  if (others_min < 1e9) {
    // own_dist < others_min → 自分が最近傍 → factor > 1 (最大1.5倍)
    // own_dist > others_min → 他ロボットが近い → factor < 1 (最小0.2)
    const double voronoi_factor = std::clamp(others_min / own_dist, 0.2, 1.5);
    score *= voronoi_factor;
  }

  return score;
}

auto ForwardSession::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
  -> std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>>
{
  if (robots.empty()) {
    forward_skills.clear();
    return {SessionBase::Status::RUNNING, {}};
  }

  // スキル数がロボット数と異なる場合に再生成
  if (forward_skills.size() != robots.size()) {
    forward_skills.clear();
    visualizer->layer = "skill/forward";
    for (const auto & robot_id : robots) {
      auto skill = std::make_shared<skills::Forward>(robot_id.id, world_model);
      skill->setParameter("max_vel", 1.5);
      skill->planner_visualizer = visualizer;
      forward_skills.emplace_back(skill);
    }
  }

  // ---- 候補点生成 ----
  const auto candidates = computeCandidatePoints();
  if (candidates.empty()) {
    // フォールバック: 現在位置を維持
    std::vector<crane_msgs::msg::RobotCommand> robot_commands;
    for (auto & skill : forward_skills) {
      skill->run();
      robot_commands.emplace_back(skill->getRobotCommand());
    }
    return {SessionBase::Status::RUNNING, robot_commands};
  }

  const size_t n_robots = robots.size();
  const size_t n_cands = candidates.size();

  // ---- 全フォワードロボットの現在位置 ----
  std::vector<Point> forward_positions =
    robots | ranges::views::transform([this](const RobotIdentifier & id) -> Point {
      return world_model->getRobot(id)->pose.pos;
    }) |
    ranges::to<std::vector>;

  const auto available_enemies = world_model->theirs().robotsWhere().available().get();

  // ---- スコア行列計算 ----
  // score_matrix[robot_idx][cand_idx]
  std::vector<std::vector<double>> score_matrix(n_robots, std::vector<double>(n_cands, 0.0));

  for (size_t ri = 0; ri < n_robots; ++ri) {
    const Point & robot_pos = forward_positions[ri];
    const uint8_t robot_id = robots[ri].id;

    for (size_t ci = 0; ci < n_cands; ++ci) {
      double s = scorePoint(candidates[ci], robot_pos, forward_positions, available_enemies);

      // ---- ヒステリシス引力（前回目標付近のスコアを引き上げ） ----
      // BONUS * exp(-dist²/(2σ²)) で前回目標付近を優遇
      if (last_targets_.count(robot_id)) {
        const double dist = (candidates[ci] - last_targets_.at(robot_id)).norm();
        constexpr double HYSTERESIS_BONUS = 0.3;
        constexpr double HYSTERESIS_SIGMA = 1.0;
        s *= 1.0 + HYSTERESIS_BONUS *
                     std::exp(-dist * dist / (2.0 * HYSTERESIS_SIGMA * HYSTERESIS_SIGMA));
      }

      score_matrix[ri][ci] = s;

      // 可視化（スコアを色付き円で表示）
      if (ri == 0 && visualizer) {
        visualizer->drawCircle(candidates[ci], s * 0.2, "lime", 3, 0.4);
      }
    }
  }

  // ---- スコアの正規化（最大スコアで割る） ----
  double global_max = 1e-9;
  for (const auto & row : score_matrix) {
    for (double v : row) global_max = std::max(global_max, v);
  }

  // ---- 最適割当（コスト = 1 - normalized_score） ----
  auto solution = getOptimalAssignmentsWithCost(
    n_robots, n_cands, [&](int ri, int ci) { return 1.0 - score_matrix[ri][ci] / global_max; });

  // ---- 目標点の適用とヒステリシス更新 ----
  for (size_t i = 0; i < n_robots; ++i) {
    const int ci = solution[i];
    const uint8_t robot_id = robots[i].id;

    if (ci < 0 || static_cast<size_t>(ci) >= n_cands) {
      RCLCPP_WARN(
        rclcpp::get_logger("ForwardSession"),
        "Invalid assignment index (%d) for robot %d. Skipping.", ci, robot_id);
      continue;
    }

    const Point & target = candidates[ci];

    // ヒステリシス: 新目標が旧目標より15%以上スコア改善した場合のみ更新
    bool should_update = true;
    if (last_targets_.count(robot_id)) {
      // 現在の旧目標の非ヒステリシス補正済みスコア（元スコアで比較）
      const Point & old_target = last_targets_.at(robot_id);
      const double old_score =
        scorePoint(old_target, forward_positions[i], forward_positions, available_enemies);
      const double new_score =
        scorePoint(target, forward_positions[i], forward_positions, available_enemies);
      constexpr double MIN_IMPROVEMENT = 0.15;
      // 旧目標よりも大幅に改善した場合のみ切り替え
      // （ヒステリシスなしの生スコアで比較してチラつきを防ぐ）
      if (new_score < old_score * (1.0 + MIN_IMPROVEMENT)) {
        should_update = false;
      }
    }

    const Point & final_target = should_update ? target : last_targets_.at(robot_id);
    if (should_update) {
      last_targets_[robot_id] = target;
    }

    forward_skills[i]->setParameter("front_point", final_target);
    forward_skills[i]->setParameter("back_point", final_target);

    // 選択された目標点を強調表示
    if (visualizer) {
      visualizer->drawCircle(final_target, 0.25, "magenta", 8, 0.9);
      if (last_targets_.count(robot_id) && !should_update) {
        // ヒステリシスで旧目標を保持中の場合は旧目標→新候補へ線を引く
        visualizer->drawLine(final_target, target, "orange", 3);
      }
    }
  }

  // ---- スキル実行 ----
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  for (auto & skill : forward_skills) {
    skill->run();
    robot_commands.emplace_back(skill->getRobotCommand());
  }
  return {SessionBase::Status::RUNNING, robot_commands};
}
}  // namespace crane
