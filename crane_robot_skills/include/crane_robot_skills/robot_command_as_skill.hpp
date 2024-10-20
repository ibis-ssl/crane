// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__ROBOT_COMMAND_AS_SKILL_HPP_
#define CRANE_ROBOT_SKILLS__ROBOT_COMMAND_AS_SKILL_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{

#define DEFINE_SKILL_COMMAND(name, type)                                           \
  class Cmd##name : public SkillBase<RobotCommandWrapper##type>                    \
  {                                                                                \
  public:                                                                          \
    explicit Cmd##name(RobotCommandWrapperBase::SharedPtr & base);                 \
    Status update(const ConsaiVisualizerWrapper::SharedPtr & visualizer) override; \
    void print(std::ostream & os) const override;                                  \
  }

DEFINE_SKILL_COMMAND(KickWithChip, Position);
DEFINE_SKILL_COMMAND(KickStraight, Position);
DEFINE_SKILL_COMMAND(Dribble, Position);
DEFINE_SKILL_COMMAND(SetVelocity, SimpleVelocity);
DEFINE_SKILL_COMMAND(SetTargetPosition, Position);
DEFINE_SKILL_COMMAND(SetDribblerTargetPosition, Position);
DEFINE_SKILL_COMMAND(SetTargetTheta, Position);
DEFINE_SKILL_COMMAND(StopHere, Position);
DEFINE_SKILL_COMMAND(DisablePlacementAvoidance, Position);
DEFINE_SKILL_COMMAND(EnablePlacementAvoidance, Position);
DEFINE_SKILL_COMMAND(DisableBallAvoidance, Position);
DEFINE_SKILL_COMMAND(EnableBallAvoidance, Position);
DEFINE_SKILL_COMMAND(DisableCollisionAvoidance, Position);
DEFINE_SKILL_COMMAND(EnableCollisionAvoidance, Position);
DEFINE_SKILL_COMMAND(DisableGoalAreaAvoidance, Position);
DEFINE_SKILL_COMMAND(EnableGoalAreaAvoidance, Position);
DEFINE_SKILL_COMMAND(SetGoalieDefault, Position);
DEFINE_SKILL_COMMAND(EnableBallCenteringControl, Position);
DEFINE_SKILL_COMMAND(EnableLocalGoalie, Position);
DEFINE_SKILL_COMMAND(SetMaxVelocity, Position);
DEFINE_SKILL_COMMAND(SetMaxAcceleration, Position);
DEFINE_SKILL_COMMAND(SetTerminalVelocity, Position);
DEFINE_SKILL_COMMAND(EnableStopFlag, Position);
DEFINE_SKILL_COMMAND(DisableStopFlag, Position);
DEFINE_SKILL_COMMAND(LiftUpDribbler, Position);
DEFINE_SKILL_COMMAND(LookAt, Position);
DEFINE_SKILL_COMMAND(LookAtBall, Position);
DEFINE_SKILL_COMMAND(LookAtBallFrom, Position);

}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__ROBOT_COMMAND_AS_SKILL_HPP_
