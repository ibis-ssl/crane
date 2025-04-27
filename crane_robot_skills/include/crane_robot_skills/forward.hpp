// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__FORWARD_HPP_
#define CRANE_ROBOT_SKILLS__FORWARD_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

#include "skill_base.hpp"

namespace crane::skills
{
class Forward : public SkillBase
{
public:
  template <typename... Args>
  explicit Forward(Args &&... args) : SkillBase("Forward", std::forward<Args>(args)...)
  {
    // 敵陣側
    setParameter("front_point", Point(0, 0));
    // 自陣側
    setParameter("back_point", Point(0, 0));
    setParameter("max_ball_distance", 5.0);
  }

  auto update() -> Status override
  {
    auto their_robots = world_model()->theirs.getAvailableRobots();
    Point front_point = getParameter<Point>("front_point");
    Point back_point = getParameter<Point>("back_point");
    auto max_ball_distance = getParameter<double>("max_ball_distance");
    auto & ball = world_model()->ball;
    // front_point -> back_pointの0.1mごとのポイントを生成
    std::vector<Point> points = ranges::views::iota(0, (back_point - front_point).norm() / 0.1) |
                                ranges::views::transform([&](double t) -> Point {
                                  return front_point + (back_point - front_point) * t;
                                }) |
                                ranges::to<std::vector>();

    std::vector<std::pair<Point, double>> points_with_score =
      points | ranges::views::transform([&](const Point & p) {
        double score = 1.0;
        // ボールの受取やすさ
        Segment segment{
          p, ball.pos + (p - ball.vel).normalized() * 1.5};  // ボール近くはチップで無視可能
        auto nearest_enemy = world_model()->getNearestRobotWithDistanceFromSegment(
          segment, world_model()->theirs.getAvailableRobots());
        if (nearest_enemy) {
          // 0.0~1.0 : 遠いほど高スコア
          score *= std::clamp(nearest_enemy->distance, 0.2, 2.0) / 2.0;
        } else {
          score = 0.0;
        }
        visualizer->circle().center(p).radius(score).stroke("green").strokeWidth(10).build();

        return std::make_pair(p, score);
      }) |
      ranges::to<std::vector>();

    auto best_point = ranges::max_element(
      points_with_score, [](const auto & a, const auto & b) { return a.second < b.second; });

    command->setTargetPosition(best_point->first).lookAtBall();

    visualizer->line().start(front_point).end(back_point).stroke("green").strokeWidth(10).build();
    return Status::RUNNING;
  }

  void print(std::ostream & os) const override { os << "[Forward]"; }
};

}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__FORWARD_HPP_
