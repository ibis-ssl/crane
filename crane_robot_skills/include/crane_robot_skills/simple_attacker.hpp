// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SIMPLE_ATTACKER_HPP_
#define CRANE_ROBOT_SKILLS__SIMPLE_ATTACKER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_robot_skills/skill_base.hpp>

namespace crane::skills
{
class SimpleAttacker : public SkillBase<>
{
public:
  explicit SimpleAttacker(uint8_t id, const std::shared_ptr<WorldModelWrapper> & world_model);

  void print(std::ostream & os) const override
  {
    os << "[Idle] stop_by_position: " << getParameter<bool>("stop_by_position") ? "true" : "false";
  }

  auto getBestShootTargetWithWidth() -> std::pair<Point, double>
  {
    const auto & ball = world_model->ball.pos;

    Interval goal_range;

    auto goal_posts = world_model->getTheirGoalPosts();
    goal_range.append(getAngle(goal_posts.first - ball), getAngle(goal_posts.second - ball));

    for (auto & enemy : world_model->theirs.robots) {
      double distance = (ball - enemy->pose.pos).norm();
      constexpr double MACHINE_RADIUS = 0.1;

      double center_angle = getAngle(enemy->pose.pos - ball);
      double diff_angle =
        atan(MACHINE_RADIUS / std::sqrt(distance * distance - MACHINE_RADIUS * MACHINE_RADIUS));

      goal_range.erase(center_angle - diff_angle, center_angle + diff_angle);
    }

    auto largest_interval = goal_range.getLargestInterval();

    double target_angle = (largest_interval.first + largest_interval.second) / 2.0;

    return {
      ball + getNormVec(target_angle) * 0.5, largest_interval.second - largest_interval.first};
  }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__SIMPLE_ATTACKER_HPP_
