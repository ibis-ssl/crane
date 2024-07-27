// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__KICKOFF_ATTACK_HPP_
#define CRANE_ROBOT_SKILLS__KICKOFF_ATTACK_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/go_over_ball.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
enum class KickoffAttackState {
  PREPARE_KICKOFF,
  KICKOFF,
};
class KickoffAttack : public SkillBaseWithState<KickoffAttackState, RobotCommandWrapperPosition>
{
public:
  explicit KickoffAttack(RobotCommandWrapperBase::SharedPtr & base);

  void print(std::ostream & os) const override { os << "[KickoffAttack]"; }

private:
  std::shared_ptr<GoOverBall> go_over_ball = nullptr;

  Status go_over_ball_status = Status::RUNNING;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__KICKOFF_ATTACK_HPP_
