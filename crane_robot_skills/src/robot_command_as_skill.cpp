// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/robot_command_as_skill.hpp>

namespace crane
{

#define ONE_FRAME_IMPLEMENTATION(name, method)                                       \
  Cmd##name::Cmd##name(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model) \
  : SkillBase<>("Cmd" #name, id, world_model, DefaultStates::DEFAULT)                \
  {                                                                                  \
    addStateFunction(                                                                \
      DefaultStates::DEFAULT,                                                        \
      [this](                                                                        \
        const std::shared_ptr<WorldModelWrapper> & world_model,                      \
        const std::shared_ptr<RobotInfo> & robot,                                    \
        crane::RobotCommandWrapper & command) -> SkillBase::Status {                 \
        command.method;                                                              \
        return SkillBase::Status::SUCCESS;                                           \
      });                                                                            \
  }

#define ETERNAL_IMPLEMENTATION(name, method)                                         \
  Cmd##name::Cmd##name(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model) \
  : SkillBase<>("Cmd" #name, id, world_model, DefaultStates::DEFAULT)                \
  {                                                                                  \
    addStateFunction(                                                                \
      DefaultStates::DEFAULT,                                                        \
      [this](                                                                        \
        const std::shared_ptr<WorldModelWrapper> & world_model,                      \
        const std::shared_ptr<RobotInfo> & robot,                                    \
        crane::RobotCommandWrapper & command) -> SkillBase::Status {                 \
        command.method;                                                              \
        return SkillBase::Status::RUNNING;                                           \
      });                                                                            \
  }

ONE_FRAME_IMPLEMENTATION(KickWithChip, kickWithChip(0.5))
ONE_FRAME_IMPLEMENTATION(KickStraight, kickStraight(0.5))
ONE_FRAME_IMPLEMENTATION(Dribble, dribble(0.5))
ONE_FRAME_IMPLEMENTATION(SetVelocity, setVelocity(0.0, 0.0))
ONE_FRAME_IMPLEMENTATION(SetTargetPosition, setTargetPosition(Point(0.0, 0.0)))
ONE_FRAME_IMPLEMENTATION(SetDribblerTargetPosition, setDribblerTargetPosition(Point(0.0, 0.0)))
ONE_FRAME_IMPLEMENTATION(SetTargetTheta, setTargetTheta(0.0))
ONE_FRAME_IMPLEMENTATION(StopHere, stopHere())
ONE_FRAME_IMPLEMENTATION(DisablePlacementAvoidance, disablePlacementAvoidance())
ONE_FRAME_IMPLEMENTATION(EnablePlacementAvoidance, enablePlacementAvoidance())
ONE_FRAME_IMPLEMENTATION(DisableBallAvoidance, disableBallAvoidance())
ONE_FRAME_IMPLEMENTATION(EnableBallAvoidance, enableBallAvoidance())
ONE_FRAME_IMPLEMENTATION(DisableCollisionAvoidance, disableCollisionAvoidance())
ONE_FRAME_IMPLEMENTATION(EnableCollisionAvoidance, enableCollisionAvoidance())
ONE_FRAME_IMPLEMENTATION(DisableGoalAreaAvoidance, disableGoalAreaAvoidance())
ONE_FRAME_IMPLEMENTATION(EnableGoalAreaAvoidance, enableGoalAreaAvoidance())
ONE_FRAME_IMPLEMENTATION(SetGoalieDefault, setGoalieDefault())
ONE_FRAME_IMPLEMENTATION(EnableBallCenteringControl, enableBallCenteringControl())
ONE_FRAME_IMPLEMENTATION(EnableLocalGoalie, enableLocalGoalie())
ONE_FRAME_IMPLEMENTATION(SetMaxVelocity, setMaxVelocity(0.0))
ONE_FRAME_IMPLEMENTATION(SetMaxAcceleration, setMaxAcceleration(0.0))
ONE_FRAME_IMPLEMENTATION(SetMaxOmega, setMaxOmega(0.0))
ONE_FRAME_IMPLEMENTATION(SetTerminalVelocity, setTerminalVelocity(0.0))

}  // namespace crane
