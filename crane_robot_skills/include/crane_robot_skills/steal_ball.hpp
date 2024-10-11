// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__STEAL_BALL_HPP_
#define CRANE_ROBOT_SKILLS__STEAL_BALL_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <string>
#include <vector>

namespace crane::skills
{
enum class StealBallState {
  MOVE_TO_FRONT,
  STEAL,
};
class StealBall : public SkillBaseWithState<StealBallState, RobotCommandWrapperPosition>
{
public:
  explicit StealBall(RobotCommandWrapperBase::SharedPtr & base);

  void print(std::ostream & os) const override { os << "[StealBall]"; }

  Status skill_state = Status::RUNNING;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__STEAL_BALL_HPP_
