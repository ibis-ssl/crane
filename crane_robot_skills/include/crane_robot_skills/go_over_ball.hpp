// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__GO_OVER_BALL_HPP_
#define CRANE_ROBOT_SKILLS__GO_OVER_BALL_HPP_

#include <crane_basics/vector2d_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <utility>
#include <vector>

namespace crane::skills
{
class GoOverBall : public SkillBase
{
public:
  template <typename... Args>
  explicit GoOverBall(Args &&... args)
  : SkillBase("GoOverBall", std::forward<Args>(args)...),
    has_started(getContextReference<bool>("has_started", false)),
    has_passed_intermediate_target(
      getContextReference<bool>("has_passed_intermediate_target", false)),
    final_target_pos(getContextReference<Point>("final_target_pos")),
    intermediate_target_pos(
      {getContextReference<Point>("intermediate_target_pos1"),
       getContextReference<Point>("intermediate_target_pos2")})
  {
    setParameter("next_target_x", 0.0);
    setParameter("next_target_y", 0.0);
    setParameter("margin", 0.5);
    setParameter("reach_threshold", 0.05);
  }

  Status update() override;

  void print(std::ostream & out) const override;

private:
  bool & has_started;

  bool & has_passed_intermediate_target;
  //  bool has_
  Point & final_target_pos;

  std::pair<Point &, Point &> intermediate_target_pos;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__GO_OVER_BALL_HPP_
