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
#include <memory>
#include <utility>

namespace crane::skills
{
enum class SimpleAttackerState {
  ENTRY_POINT,
  NORMAL_APPROACH,
  RECEIVE_APPROACH,
  THROUGH,
  STOP,
};
class SimpleAttacker : public SkillBase<SimpleAttackerState>
{
public:
  explicit SimpleAttacker(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm);

  void print(std::ostream & os) const override
  {
    os << "[Idle] stop_by_position: " << getParameter<bool>("stop_by_position") ? "true" : "false";
  }

  bool isBallComingFromBack(double ball_vel_threshold = 0.5) const;

  double getSlackTime(double t_ball);

  std::optional<Point> getMinimumTimeInterceptPoint();

  std::optional<Point> getMaximumSlackInterceptPoint();

  Point kick_target;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__SIMPLE_ATTACKER_HPP_
