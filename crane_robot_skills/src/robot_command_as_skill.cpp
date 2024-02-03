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
  : SkillBase<>("Cmd"##name, id, world_model, DefaultStates::DEFAULT)                \
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

CmdKickWithChip::CmdKickWithChip(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
: SkillBase<>("CmdKickWithChip", id, world_model, DefaultStates::DEFAULT)
{
  setParameter("power", 0.5);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot,
      crane::RobotCommandWrapper & command) -> SkillBase::Status {
      command.kickWithChip(getParameter<double>("power"));
      return SkillBase::Status::SUCCESS;
    });
}

CmdKickStraight::CmdKickStraight(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
: SkillBase<>("CmdKickStraight", id, world_model, DefaultStates::DEFAULT)
{
  setParameter("power", 0.5);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot,
      crane::RobotCommandWrapper & command) -> SkillBase::Status {
      command.kickWithChip(getParameter<double>("power"));
      return SkillBase::Status::SUCCESS;
    });
}

CmdDribble::CmdDribble(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
: SkillBase<>("CmdDribble", id, world_model, DefaultStates::DEFAULT)
{
  setParameter("power", 0.5);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot,
      crane::RobotCommandWrapper & command) -> SkillBase::Status {
      command.dribble(getParameter<double>("power"));
      return SkillBase::Status::SUCCESS;
    });
}

CmdSetTerminalTargetVelocity::CmdSetTerminalTargetVelocity(
  uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
: SkillBase<>("CmdSetTerminalTargetVelocity", id, world_model, DefaultStates::DEFAULT)
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot,
      crane::RobotCommandWrapper & command) -> SkillBase::Status {
      command.setTerminalTargetVelocity(getParameter<double>("x"), getParameter<double>("y"));
      return SkillBase::Status::SUCCESS;
    });
}

CmdSetTargetPosition::CmdSetTargetPosition(
  uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
: SkillBase<>("CmdSetTargetPosition", id, world_model, DefaultStates::DEFAULT)
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
  setParameter("reach_threshold", 0.1);
  setParameter("exit_immediately", false);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot,
      crane::RobotCommandWrapper & command) -> SkillBase::Status {
      Point target{getParameter<double>("x"), getParameter<double>("y")};
      command.setTargetPosition(target);
      if (getParameter<bool>("exit_immediately")) {
        return SkillBase::Status::SUCCESS;
      } else {
        if ((robot->pose.pos - target).norm() <= getParameter<double>("reach_threshold")) {
          return SkillBase::Status::SUCCESS;
        } else {
          return SkillBase::Status::RUNNING;
        }
      }
    });
}

CmdSetDribblerTargetPosition::CmdSetDribblerTargetPosition(
  uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
: SkillBase<>("CmdSetDribblerTargetPosition", id, world_model, DefaultStates::DEFAULT)
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
  setParameter("reach_threshold", 0.1);
  setParameter("exit_immediately", false);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot,
      crane::RobotCommandWrapper & command) -> SkillBase::Status {
      Point target{getParameter<double>("x"), getParameter<double>("y")};
      command.setDribblerTargetPosition(target);
      if (getParameter<bool>("exit_immediately")) {
        return SkillBase::Status::SUCCESS;
      } else {
        if ((robot->kicker_center() - target).norm() <= getParameter<double>("reach_threshold")) {
          return SkillBase::Status::SUCCESS;
        } else {
          return SkillBase::Status::RUNNING;
        }
      }
    });
}

CmdSetTargetTheta::CmdSetTargetTheta(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
: SkillBase<>("CmdSetTargetTheta", id, world_model, DefaultStates::DEFAULT)
{
  setParameter("theta", 0.0);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot,
      crane::RobotCommandWrapper & command) -> SkillBase::Status {
      command.setTargetTheta(getParameter<double>("theta"));
      return SkillBase::Status::SUCCESS;
    });
}

CmdStopHere::CmdStopHere(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
: SkillBase<>("CmdStopHere", id, world_model, DefaultStates::DEFAULT)
{
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot,
      crane::RobotCommandWrapper & command) -> SkillBase::Status {
      command.stopHere();
      return SkillBase::Status::SUCCESS;
    });
}

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

CmdSetMaxVelocity::CmdSetMaxVelocity(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
: SkillBase<>("CmdSetMaxVelocity", id, world_model, DefaultStates::DEFAULT)
{
  setParameter("max_velocity", 0.5);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot,
      crane::RobotCommandWrapper & command) -> SkillBase::Status {
      command.setMaxVelocity(getParameter<double>("max_velocity"));
      return SkillBase::Status::SUCCESS;
    });
}

CmdSetMaxAcceleration::CmdSetMaxAcceleration(
  uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
: SkillBase<>("CmdSetMaxAcceleration", id, world_model, DefaultStates::DEFAULT)
{
  setParameter("max_acceleration", 0.5);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot,
      crane::RobotCommandWrapper & command) -> SkillBase::Status {
      command.setMaxAcceleration(getParameter<double>("max_acceleration"));
      return SkillBase::Status::SUCCESS;
    });
}

CmdSetMaxOmega::CmdSetMaxOmega(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
: SkillBase<>("CmdSetMaxOmega", id, world_model, DefaultStates::DEFAULT)
{
  setParameter("max_omega", 0.5);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot,
      crane::RobotCommandWrapper & command) -> SkillBase::Status {
      command.setMaxOmega(getParameter<double>("max_omega"));
      return SkillBase::Status::SUCCESS;
    });
}

CmdSetTerminalVelocity::CmdSetTerminalVelocity(
  uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
: SkillBase<>("CmdSetTerminalVelocity", id, world_model, DefaultStates::DEFAULT)
{
  setParameter("terminal_velocity", 0.5);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot,
      crane::RobotCommandWrapper & command) -> SkillBase::Status {
      command.setTerminalVelocity(getParameter<double>("terminal_velocity"));
      return SkillBase::Status::SUCCESS;
    });
}

CmdLookAt::CmdLookAt(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
: SkillBase<>("CmdLookAt", id, world_model, DefaultStates::DEFAULT)
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot,
      crane::RobotCommandWrapper & command) -> SkillBase::Status {
      Point target{getParameter<double>("x"), getParameter<double>("y")};
      command.lookAt(target);
      return SkillBase::Status::SUCCESS;
    });
}

CmdLookAtBall::CmdLookAtBall(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
: SkillBase<>("CmdLookAtBall", id, world_model, DefaultStates::DEFAULT)
{
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot,
      crane::RobotCommandWrapper & command) -> SkillBase::Status {
      command.lookAtBall();
      return SkillBase::Status::SUCCESS;
    });
}

CmdLookAtBallFrom::CmdLookAtBallFrom(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
: SkillBase<>("CmdLookAtBallFrom", id, world_model, DefaultStates::DEFAULT)
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot,
      crane::RobotCommandWrapper & command) -> SkillBase::Status {
      Point target{getParameter<double>("x"), getParameter<double>("y")};
      command.lookAtBallFrom(target);
      return SkillBase::Status::SUCCESS;
    });
}

}  // namespace crane
