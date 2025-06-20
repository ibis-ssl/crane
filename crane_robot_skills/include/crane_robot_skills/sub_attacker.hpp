// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SUB_ATTACKER_HPP_
#define CRANE_ROBOT_SKILLS__SUB_ATTACKER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_game_analyzer/evaluations/evaluations.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <utility>
#include <vector>

namespace crane::skills
{
class SubAttacker : public SkillBase
{
public:
  template <typename... Args>
  explicit SubAttacker(Args &&... args) : SkillBase("SubAttacker", std::forward<Args>(args)...)
  {
    initialize();
  }

  void initialize();

  Status update() override;

  void print(std::ostream & os) const override { os << "[SubAttacker]"; }

  static std::vector<std::pair<double, Point>> getPositionsWithScore(
    const Segment & ball_line, const Point & next_target,
    const WorldModelWrapper::SharedPtr & world_model);

  static double getPointScore(
    const Point & p, const Point & next_target, const WorldModelWrapper::SharedPtr & world_model);
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__SUB_ATTACKER_HPP_
