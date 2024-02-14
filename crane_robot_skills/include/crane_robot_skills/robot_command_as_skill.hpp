// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__ROBOT_COMMAND_AS_SKILL_HPP_
#define CRANE_ROBOT_SKILLS__ROBOT_COMMAND_AS_SKILL_HPP_

#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>

namespace crane::skills
{

#define DEFINE_SKILL_COMMAND(name)                                                    \
  class Cmd##name : public SkillBase<>                                                \
  {                                                                                   \
  public:                                                                             \
    explicit Cmd##name(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model); \
    void print(std::ostream & os) const override;                                     \
  }

DEFINE_SKILL_COMMAND(KickWithChip);
DEFINE_SKILL_COMMAND(KickStraight);
DEFINE_SKILL_COMMAND(Dribble);
DEFINE_SKILL_COMMAND(SetVelocity);
DEFINE_SKILL_COMMAND(SetTargetPosition);
DEFINE_SKILL_COMMAND(SetDribblerTargetPosition);
DEFINE_SKILL_COMMAND(SetTargetTheta);
DEFINE_SKILL_COMMAND(StopHere);
DEFINE_SKILL_COMMAND(DisablePlacementAvoidance);
DEFINE_SKILL_COMMAND(EnablePlacementAvoidance);
DEFINE_SKILL_COMMAND(DisableBallAvoidance);
DEFINE_SKILL_COMMAND(EnableBallAvoidance);
DEFINE_SKILL_COMMAND(DisableCollisionAvoidance);
DEFINE_SKILL_COMMAND(EnableCollisionAvoidance);
DEFINE_SKILL_COMMAND(DisableGoalAreaAvoidance);
DEFINE_SKILL_COMMAND(EnableGoalAreaAvoidance);
DEFINE_SKILL_COMMAND(SetGoalieDefault);
DEFINE_SKILL_COMMAND(EnableBallCenteringControl);
DEFINE_SKILL_COMMAND(EnableLocalGoalie);
DEFINE_SKILL_COMMAND(SetMaxVelocity);
DEFINE_SKILL_COMMAND(SetMaxAcceleration);
DEFINE_SKILL_COMMAND(SetMaxOmega);
DEFINE_SKILL_COMMAND(SetTerminalVelocity);
DEFINE_SKILL_COMMAND(EnableStopFlag);
DEFINE_SKILL_COMMAND(DisableStopFlag);
DEFINE_SKILL_COMMAND(LiftUpDribbler);
DEFINE_SKILL_COMMAND(LookAt);
DEFINE_SKILL_COMMAND(LookAtBall);
DEFINE_SKILL_COMMAND(LookAtBallFrom);

}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__ROBOT_COMMAND_AS_SKILL_HPP_
