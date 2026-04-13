// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <cmath>
#include <crane_robot_skills/forward.hpp>
#include <range/v3/algorithm/max_element.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>

namespace crane::skills
{
auto Forward::update() -> Status
{
  Point front_point = getParameter<Point>("front_point");
  Point back_point = getParameter<Point>("back_point");
  auto max_ball_distance = getParameter<double>("max_ball_distance");
  auto max_vel = getParameter<double>("max_vel");
  auto & ball = world_model()->ball();

  const double line_length = (back_point - front_point).norm();
  if (!std::isfinite(line_length)) {
    // stopHereの代わりに現在位置を維持してボールを注視
    // (ForwardSessionのライン生成が一時的に不安定な場合のフォールバック)
    command->setTargetPosition(robot()->pose.pos).lookAtBall();
    return Status::RUNNING;
  }

  // front_point == back_point（点指定モード）: 1Dサンプリングをスキップして直接追従
  if (line_length < 1e-4) {
    command->setTargetPosition(front_point)
      .lookAtBall()
      .setMaxVelocity("Forward::max_vel", max_vel);
    return Status::RUNNING;
  }

  max_ball_distance = std::max(max_ball_distance, 0.1);

  // front_point -> back_pointの0.1mごとのポイントを生成
  constexpr int MAX_SAMPLING_POINTS = 200;
  int num_points =
    std::clamp(static_cast<int>(std::ceil(line_length / 0.1)), 1, MAX_SAMPLING_POINTS);

  // ループ外で定数を取得（サンプル点ごとの重複呼び出しを回避）
  const auto their_goal_center = world_model()->getAttackGoalCenter();
  const auto available_our_robots = world_model()->ours().robotsWhere().available().get();
  const auto available_enemy_robots = world_model()->theirs().robotsWhere().available().get();

  std::vector<std::pair<Point, double>> points_with_score =
    ranges::views::iota(0, num_points) | ranges::views::transform([&](int i) -> Point {
      return front_point + (back_point - front_point) * static_cast<double>(i) / num_points;
    }) |
    ranges::views::transform([&](const Point & p) {
      double score = 1.0;
      // ボールの受取やすさ
      Segment segment{
        p, ball.pos + (p - ball.vel).normalized() * 1.5};  // ボール近くはチップで無視可能
      auto nearest_enemy =
        world_model()->getNearestRobotWithDistanceFromSegment(segment, available_enemy_robots);
      if (nearest_enemy) {
        // 0.0~1.0 : 遠いほど高スコア
        score *= std::clamp(nearest_enemy->distance, 0.2, 2.0) / 2.0;
      }
      // else: 敵がいない = 安全なので score は 1.0 のまま

      double distance = (p - ball.pos).norm();
      // 最小距離制約: 1.5m未満は大幅減点
      constexpr double MIN_PASS_DISTANCE = 1.5;
      if (distance < MIN_PASS_DISTANCE) {
        score *= distance / MIN_PASS_DISTANCE;  // 1.0mなら0.67倍、0.5mなら0.33倍
      }
      // ボールとの距離評価（近すぎず遠すぎず）
      score *= (std::clamp(1.0 - distance / max_ball_distance, 0.0, 1.0) * 0.5 + 0.5);

      // 1-A: ゴールシュート角度評価
      auto [goal_angle, goal_width] = world_model()->getLargestAttackGoalAngleRangeFromPoint(p);
      score *= (std::clamp(goal_width / 0.6, 0.0, 1.0) * 0.5 + 0.5);

      // 1-B: ボール後方回避（攻撃方向の後ろ側はスコアを下げる）
      double dot = (their_goal_center - ball.pos).normalized().dot((p - ball.pos).normalized());
      score *= std::max((dot + 0.5), 0.0);

      // 1-C: 味方との分散（1.5m以内の味方ロボットがいる場合は減点）
      constexpr double MIN_TEAM_SPACING = 1.5;
      for (const auto & our_robot : available_our_robots) {
        if (our_robot->id == robot()->id) continue;
        double d = (p - our_robot->pose.pos).norm();
        if (d < MIN_TEAM_SPACING) score *= d / MIN_TEAM_SPACING;
      }

      if (planner_visualizer) {
        planner_visualizer->drawCircle(p, score * 0.25, "lime", 5, 0.5);
      }

      return std::make_pair(p, score);
    }) |
    ranges::to<std::vector>();

  auto best_point = ranges::max_element(
    points_with_score, [](const auto & a, const auto & b) { return a.second < b.second; });

  if (best_point != points_with_score.end()) {
    command->setTargetPosition(best_point->first)
      .lookAtBall()
      .setMaxVelocity("Forward::max_vel", max_vel);
  } else {
    // stopHereの代わりにfront/back中点にフォールバック
    Point midpoint = (front_point + back_point) * 0.5;
    command->setTargetPosition(midpoint).lookAtBall().setMaxVelocity("Forward::fallback", max_vel);
  }

  if (planner_visualizer) {
    planner_visualizer->drawLine(front_point, back_point, "green", 10);
  }
  return Status::RUNNING;
}
}  // namespace crane::skills
