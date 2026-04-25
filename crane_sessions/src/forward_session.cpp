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
#include <limits>
#include <numeric>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/transform.hpp>

namespace crane
{

auto ForwardSession::computeCandidatePoints() const -> std::vector<Point>
{
  const double goal_line_x = world_model->fieldSize().x() * 0.5;
  const double field_half_width = world_model->fieldSize().y() * 0.5;
  const double PA_MARGIN =
    needsExpandedPenaltyAreaOffset(world_model->getMsg().play_situation.command.value) ? 0.4 : 0.3;
  constexpr double FIELD_MARGIN = 0.2;
  constexpr double ATTACK_HALFLINE_THRESHOLD = 0.3;
  constexpr double GRID_STEP = 0.5;

  const double side_sign = -world_model->getAttackSideSign();
  const double x_near = side_sign * 0.5;
  const double x_far = side_sign * (goal_line_x - 1.0);
  const double x_min = std::min(x_near, x_far);
  const double x_max = std::max(x_near, x_far);
  const double center_x = (x_min + x_max) * 0.5;

  const int nx = static_cast<int>(std::ceil((x_max - x_min) / GRID_STEP));
  const int ny = static_cast<int>(std::ceil(2.0 * field_half_width / GRID_STEP));

  std::vector<Point> raw = getPoints(
    Point(center_x, 0.0), static_cast<float>(GRID_STEP), static_cast<float>(GRID_STEP), nx, ny);

  std::vector<Point> candidates;
  candidates.reserve(raw.size());
  for (const auto & p : raw) {
    if (!world_model->point_checker.isFieldInside(p, FIELD_MARGIN)) continue;
    if (world_model->point_checker.isPenaltyArea(p, PA_MARGIN)) continue;
    if (side_sign * p.x() < ATTACK_HALFLINE_THRESHOLD) continue;
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

  // 1. パス受取スコア（チップキック考慮済みパスライン上の敵距離）
  //    ボール近傍 CHIP_KICK_CLEAR_RADIUS 以内の敵はチップで越せるため線分評価から除外する。
  //    これによりコーナー等の長いパス線が、ボール直近の敵を理由に不当に減点されない。
  constexpr double CHIP_KICK_CLEAR_RADIUS = 1.5;
  RobotList chippable_filtered;
  chippable_filtered.reserve(available_enemies.size());
  for (const auto & enemy : available_enemies) {
    if ((enemy->pose.pos - ball.pos).norm() > CHIP_KICK_CLEAR_RADIUS) {
      chippable_filtered.push_back(enemy);
    }
  }
  auto nearest_enemy =
    world_model->getNearestRobotWithDistanceFromSegment(Segment{ball.pos, p}, chippable_filtered);
  if (nearest_enemy) {
    score *= std::clamp(nearest_enemy->distance, 0.2, 2.0) / 2.0;
  }

  // 2. ボール距離（近すぎ/遠すぎ回避）
  const double dist_to_ball = (p - ball.pos).norm();
  constexpr double MIN_PASS_DIST = 1.5;
  constexpr double MAX_PASS_DIST = 9.0;
  if (dist_to_ball < MIN_PASS_DIST) {
    score *= dist_to_ball / MIN_PASS_DIST;
  }
  score *= (std::clamp(1.0 - dist_to_ball / MAX_PASS_DIST, 0.0, 1.0) * 0.5 + 0.5);

  // 3. ゴール可視角
  score *=
    (std::clamp(
       world_model->getLargestAttackGoalAngleRangeFromPoint(p).angle_width / 0.6, 0.0, 1.0) *
       0.5 +
     0.5);

  // 4. ボール後方回避（ゴール方向の後ろ側を減点）
  const double dot = (their_goal_center - ball.pos).normalized().dot((p - ball.pos).normalized());
  score *= std::max((dot + 0.5), 0.0);

  // 5&6. 斥力（味方スペーシング）+ 近似ボロノイ（縄張り評価）を1ループで計算
  constexpr double MIN_TEAM_SPACING = 1.8;
  const double own_dist = (p - robot_pos).norm() + 1e-6;
  double others_min = std::numeric_limits<double>::max();
  for (const auto & fwd_pos : forward_positions) {
    if ((fwd_pos - robot_pos).norm() < 0.01) continue;
    const double d = (p - fwd_pos).norm();
    if (d < MIN_TEAM_SPACING) score *= d / MIN_TEAM_SPACING;
    others_min = std::min(others_min, d);
  }
  for (const auto & enemy : available_enemies) {
    others_min = std::min(others_min, (p - enemy->pose.pos).norm());
  }
  if (others_min < std::numeric_limits<double>::max()) {
    // own_dist < others_min → 自分が最近傍 → factor > 1（最大1.5倍）
    // own_dist > others_min → 他ロボットが近い → factor < 1（最小0.2）
    score *= std::clamp(others_min / own_dist, 0.2, 1.5);
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

  const auto candidates = computeCandidatePoints();
  if (candidates.empty()) {
    std::vector<crane_msgs::msg::RobotCommand> robot_commands;
    for (auto & skill : forward_skills) {
      skill->run();
      robot_commands.emplace_back(skill->getRobotCommand());
    }
    return {SessionBase::Status::RUNNING, robot_commands};
  }

  const size_t n_robots = robots.size();
  const size_t n_cands = candidates.size();

  std::vector<Point> forward_positions =
    robots | ranges::views::transform([this](const RobotIdentifier & id) -> Point {
      return world_model->getRobot(id)->pose.pos;
    }) |
    ranges::to<std::vector>;

  const auto available_enemies = world_model->theirs().robotsWhere().available().get();

  // ---- パス1: 全候補点の生スコアを計算 ----
  std::vector<std::vector<double>> raw_scores(n_robots, std::vector<double>(n_cands));
  for (size_t ri = 0; ri < n_robots; ++ri) {
    for (size_t ci = 0; ci < n_cands; ++ci) {
      raw_scores[ri][ci] =
        scorePoint(candidates[ci], forward_positions[ri], forward_positions, available_enemies);
    }
  }

  // ---- 上位K点に絞ってハンガリアン行列を縮小 ----
  // ロボット間でmax取りしてK点を選ぶことで全ロボットの選好を担保する
  constexpr size_t MIN_TOP_K = 20;
  const size_t top_k = std::min(n_cands, std::max(MIN_TOP_K, 4 * n_robots));

  std::vector<size_t> top_indices(n_cands);
  std::iota(top_indices.begin(), top_indices.end(), 0u);
  if (n_cands > top_k) {
    std::partial_sort(
      top_indices.begin(), top_indices.begin() + static_cast<ptrdiff_t>(top_k), top_indices.end(),
      [&](size_t a, size_t b) {
        double max_a = 0.0, max_b = 0.0;
        for (size_t ri = 0; ri < n_robots; ++ri) {
          max_a = std::max(max_a, raw_scores[ri][a]);
          max_b = std::max(max_b, raw_scores[ri][b]);
        }
        return max_a > max_b;
      });
    top_indices.resize(top_k);
  }

  // ---- パス2: 上位K点にヒステリシスボーナスを適用してスコア行列を構築 ----
  const size_t n_top = top_indices.size();
  std::vector<std::vector<double>> score_matrix(n_robots, std::vector<double>(n_top));

  for (size_t ri = 0; ri < n_robots; ++ri) {
    const uint8_t robot_id = robots[ri].id;
    const auto last_it = last_targets_.find(robot_id);

    for (size_t ki = 0; ki < n_top; ++ki) {
      const size_t ci = top_indices[ki];
      double s = raw_scores[ri][ci];

      // ヒステリシス引力: BONUS * exp(-dist²/(2σ²)) で前回目標付近を優遇
      if (last_it != last_targets_.end()) {
        const double dist = (candidates[ci] - last_it->second).norm();
        constexpr double HYSTERESIS_BONUS = 0.3;
        constexpr double HYSTERESIS_SIGMA = 1.0;
        s *= 1.0 + HYSTERESIS_BONUS *
                     std::exp(-dist * dist / (2.0 * HYSTERESIS_SIGMA * HYSTERESIS_SIGMA));
      }
      score_matrix[ri][ki] = s;

      if (ri == 0 && visualizer) {
        visualizer->drawCircle(candidates[ci], s * 0.2, "lime", 3, 0.4);
      }
    }
  }

  double global_max = 1e-9;
  for (const auto & row : score_matrix) {
    for (double v : row) global_max = std::max(global_max, v);
  }

  auto solution = getOptimalAssignmentsWithCost(
    n_robots, n_top, [&](int ri, int ki) { return 1.0 - score_matrix[ri][ki] / global_max; });

  // ---- 目標点の適用とヒステリシス更新 ----
  for (size_t i = 0; i < n_robots; ++i) {
    const int ki = solution[i];
    const uint8_t robot_id = robots[i].id;

    if (ki < 0 || static_cast<size_t>(ki) >= n_top) {
      RCLCPP_WARN(
        rclcpp::get_logger("ForwardSession"),
        "Invalid assignment index (%d) for robot %d. Skipping.", ki, robot_id);
      continue;
    }

    const size_t ci = top_indices[ki];
    const Point & target = candidates[ci];

    // ヒステリシス: 生スコアで15%以上改善しない限り目標を切り替えない
    bool should_update = true;
    auto last_it = last_targets_.find(robot_id);
    if (last_it != last_targets_.end()) {
      auto score_it = last_scores_.find(robot_id);
      const double old_raw_score = (score_it != last_scores_.end()) ? score_it->second : 0.0;
      constexpr double MIN_IMPROVEMENT = 0.15;
      if (raw_scores[i][ci] < old_raw_score * (1.0 + MIN_IMPROVEMENT)) {
        should_update = false;
      }
    }

    Point final_target = target;
    if (!should_update) {
      final_target = last_it->second;  // last_it有効: should_update=falseはlast_it!=end()の場合のみ
    }

    if (should_update) {
      last_targets_[robot_id] = target;
      last_scores_[robot_id] = raw_scores[i][ci];
    }

    forward_skills[i]->setParameter("front_point", final_target);
    forward_skills[i]->setParameter("back_point", final_target);

    if (visualizer) {
      visualizer->drawCircle(final_target, 0.25, "magenta", 8, 0.9);
      if (!should_update) {
        visualizer->drawLine(final_target, target, "orange", 3);
      }
    }
  }

  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  for (auto & skill : forward_skills) {
    skill->run();
    robot_commands.emplace_back(skill->getRobotCommand());
  }
  return {SessionBase::Status::RUNNING, robot_commands};
}
}  // namespace crane
