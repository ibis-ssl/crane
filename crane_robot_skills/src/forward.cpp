// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/forward.hpp>

namespace crane::skills
{
auto Forward::update() -> Status
{
  auto their_robots = world_model()->theirs().getAvailableRobots();
  Point front_point = getParameter<Point>("front_point");
  Point back_point = getParameter<Point>("back_point");
  auto max_ball_distance = getParameter<double>("max_ball_distance");
  auto max_vel = getParameter<double>("max_vel");
  auto & ball = world_model()->ball();
  // front_point -> back_pointの0.1mごとのポイントを生成
  int num_points = static_cast<int>(std::ceil((back_point - front_point).norm() / 0.1));

  std::vector<std::pair<Point, double>> points_with_score =
    ranges::views::iota(0, num_points) | ranges::views::transform([&](int i) -> Point {
      return front_point + (back_point - front_point) * static_cast<double>(i) / num_points;
    }) |
    ranges::views::transform([&](const Point & p) {
      double score = 1.0;
      // ボールの受取やすさ
      Segment segment{
        p, ball.pos + (p - ball.vel).normalized() * 1.5};  // ボール近くはチップで無視可能
      auto nearest_enemy = world_model()->getNearestRobotWithDistanceFromSegment(
        segment, world_model()->theirs().getAvailableRobots());
      if (nearest_enemy) {
        // 0.0~1.0 : 遠いほど高スコア
        score *= std::clamp(nearest_enemy->distance, 0.2, 2.0) / 2.0;
      } else {
        score = 0.0;
      }

      double distance = (p - ball.pos).norm();
      score *= (std::clamp(1.0 - distance / max_ball_distance, 0.0, 1.0) * 0.5 + 0.5);

      if (planner_visualizer) {
        planner_visualizer->circle()
          .center(p)
          .radius(score * 0.25)
          .stroke("lime", 0.5)
          .strokeWidth(5)
          .build();
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
    command->stopHere();
  }

  if (planner_visualizer) {
    planner_visualizer->drawLine(front_point, back_point, "green", 10);
  }
  return Status::RUNNING;
}
}  // namespace crane::skills
