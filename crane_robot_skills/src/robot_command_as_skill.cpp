// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/robot_command_as_skill.hpp>

namespace crane::skills
{

#define ONE_FRAME_IMPLEMENTATION(name, method)                                    \
  Cmd##name::Cmd##name(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm) \
  : SkillBase<>("Cmd" #name, id, world_model, DefaultStates::DEFAULT)             \
  {                                                                               \
    addStateFunction(                                                             \
      DefaultStates::DEFAULT,                                                     \
      [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {   \
        command->method;                                                          \
        return Status::SUCCESS;                                                   \
      });                                                                         \
  }                                                                               \
  void Cmd##name::print(std::ostream & os) const {}

CmdKickWithChip::CmdKickWithChip(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("CmdKickWithChip", id, wm, DefaultStates::DEFAULT)
{
  setParameter("power", 0.5);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command->kickWithChip(getParameter<double>("power"));
      return Status::SUCCESS;
    });
}

void CmdKickWithChip::print(std::ostream & os) const
{
  os << "[CmdKickWithChip] power: " << getParameter<double>("power");
}

CmdKickStraight::CmdKickStraight(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("CmdKickStraight", id, wm, DefaultStates::DEFAULT)
{
  setParameter("power", 0.5);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command->kickStraight(getParameter<double>("power"));
      return Status::SUCCESS;
    });
}

void CmdKickStraight::print(std::ostream & os) const
{
  os << "[CmdKickStraight] power: " << getParameter<double>("power");
}

CmdDribble::CmdDribble(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("CmdDribble", id, wm, DefaultStates::DEFAULT)
{
  setParameter("power", 0.5);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command->dribble(getParameter<double>("power"));
      return Status::SUCCESS;
    });
}

void CmdDribble::print(std::ostream & os) const
{
  os << "[CmdDribble] power: " << getParameter<double>("power");
}

CmdSetVelocity::CmdSetVelocity(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("CmdSetVelocity", id, wm, DefaultStates::DEFAULT)
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command->setVelocity(getParameter<double>("x"), getParameter<double>("y"));
      return Status::SUCCESS;
    });
}

void CmdSetVelocity::print(std::ostream & os) const
{
  os << "[CmdSetVelocity] x: " << getParameter<double>("x") << " y: " << getParameter<double>("y");
}

CmdSetTargetPosition::CmdSetTargetPosition(
  uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("CmdSetTargetPosition", id, wm, DefaultStates::DEFAULT)
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
  setParameter("reach_threshold", 0.1);
  setParameter("exit_immediately", false);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      Point target{getParameter<double>("x"), getParameter<double>("y")};
      command->setTargetPosition(target);
      if (getParameter<bool>("exit_immediately")) {
        return Status::SUCCESS;
      } else {
        if (robot->getDistance(target) <= getParameter<double>("reach_threshold")) {
          return Status::SUCCESS;
        } else {
          return Status::RUNNING;
        }
      }
    });
}

void CmdSetTargetPosition::print(std::ostream & os) const
{
  os << "[CmdSetTargetPosition] distance: "
     << (robot->pose.pos - Point{getParameter<double>("x"), getParameter<double>("y")}).norm();
}

CmdSetDribblerTargetPosition::CmdSetDribblerTargetPosition(
  uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("CmdSetDribblerTargetPosition", id, wm, DefaultStates::DEFAULT)
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
  setParameter("reach_threshold", 0.1);
  setParameter("exit_immediately", false);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      Point target{getParameter<double>("x"), getParameter<double>("y")};
      command->setDribblerTargetPosition(target);
      if (getParameter<bool>("exit_immediately")) {
        return Status::SUCCESS;
      } else {
        if ((robot->kicker_center() - target).norm() <= getParameter<double>("reach_threshold")) {
          return Status::SUCCESS;
        } else {
          return Status::RUNNING;
        }
      }
    });
}

void CmdSetDribblerTargetPosition::print(std::ostream & os) const
{
  os << "[CmdSetDribblerTargetPosition] distance: "
     << (robot->kicker_center() - Point{getParameter<double>("x"), getParameter<double>("y")})
          .norm();
}

CmdSetTargetTheta::CmdSetTargetTheta(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("CmdSetTargetTheta", id, wm, DefaultStates::DEFAULT)
{
  setParameter("theta", 0.0);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command->setTargetTheta(getParameter<double>("theta"));
      return Status::SUCCESS;
    });
}

void CmdSetTargetTheta::print(std::ostream & os) const
{
  os << "[CmdSetTargetTheta] theta: " << getParameter<double>("theta");
}

CmdStopHere::CmdStopHere(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("CmdStopHere", id, wm, DefaultStates::DEFAULT)
{
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command->stopHere();
      return Status::SUCCESS;
    });
}

void CmdStopHere::print(std::ostream & os) const { os << "[CmdStopHere]"; }

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

CmdSetMaxVelocity::CmdSetMaxVelocity(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("CmdSetMaxVelocity", id, wm, DefaultStates::DEFAULT)
{
  setParameter("max_velocity", 0.5);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command->setMaxVelocity(getParameter<double>("max_velocity"));
      return Status::SUCCESS;
    });
}

void CmdSetMaxVelocity::print(std::ostream & os) const
{
  os << "[CmdSetMaxVelocity] max_velocity: " << getParameter<double>("max_velocity");
}

CmdSetMaxAcceleration::CmdSetMaxAcceleration(
  uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("CmdSetMaxAcceleration", id, wm, DefaultStates::DEFAULT)
{
  setParameter("max_acceleration", 0.5);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command->setMaxAcceleration(getParameter<double>("max_acceleration"));
      return Status::SUCCESS;
    });
}

void CmdSetMaxAcceleration::print(std::ostream & os) const
{
  os << "[CmdSetMaxAcceleration] max_acceleration: " << getParameter<double>("max_acceleration");
}

CmdSetMaxOmega::CmdSetMaxOmega(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("CmdSetMaxOmega", id, wm, DefaultStates::DEFAULT)
{
  setParameter("max_omega", 0.5);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command->setMaxOmega(getParameter<double>("max_omega"));
      return Status::SUCCESS;
    });
}

void CmdSetMaxOmega::print(std::ostream & os) const
{
  os << "[CmdSetMaxOmega] max_omega: " << getParameter<double>("max_omega");
}

CmdSetTerminalVelocity::CmdSetTerminalVelocity(
  uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("CmdSetTerminalVelocity", id, wm, DefaultStates::DEFAULT)
{
  setParameter("terminal_velocity", 0.5);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command->setTerminalVelocity(getParameter<double>("terminal_velocity"));
      return Status::SUCCESS;
    });
}

void CmdSetTerminalVelocity::print(std::ostream & os) const
{
  os << "[CmdSetTerminalVelocity] terminal_velocity: " << getParameter<double>("terminal_velocity");
}

CmdEnableStopFlag::CmdEnableStopFlag(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("CmdEnableStopFlag  ", id, wm, DefaultStates::DEFAULT)
{
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command->stopEmergency(true);
      return Status::SUCCESS;
    });
}

void CmdEnableStopFlag::print(std::ostream & os) const { os << "[CmdEnableStopFlag]"; }

CmdDisableStopFlag::CmdDisableStopFlag(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("CmdDisableStopFlag", id, wm, DefaultStates::DEFAULT)
{
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command->stopEmergency(false);
      return Status::SUCCESS;
    });
}

void CmdDisableStopFlag::print(std::ostream & os) const { os << "[CmdDisableStopFlag]"; }

CmdLiftUpDribbler::CmdLiftUpDribbler(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("CmdLiftUpDribbler", id, wm, DefaultStates::DEFAULT)
{
  setParameter("enable", true);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command->liftUpDribbler(getParameter<bool>("enable"));
      return Status::SUCCESS;
    });
}

void CmdLiftUpDribbler::print(std::ostream & os) const
{
  os << "[CmdLiftUpDribbler] enable: " << getParameter<bool>("enable");
}

CmdLookAt::CmdLookAt(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("CmdLookAt", id, wm, DefaultStates::DEFAULT)
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      Point target{getParameter<double>("x"), getParameter<double>("y")};
      command->lookAt(target);
      return Status::SUCCESS;
    });
}

void CmdLookAt::print(std::ostream & os) const
{
  os << "[CmdLookAt] x: " << getParameter<double>("x") << " y: " << getParameter<double>("y");
}

CmdLookAtBall::CmdLookAtBall(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("CmdLookAtBall", id, wm, DefaultStates::DEFAULT)
{
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command->lookAtBall();
      return Status::SUCCESS;
    });
}

void CmdLookAtBall::print(std::ostream & os) const { os << "[CmdLookAtBall]"; }

CmdLookAtBallFrom::CmdLookAtBallFrom(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("CmdLookAtBallFrom", id, wm, DefaultStates::DEFAULT)
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      Point target{getParameter<double>("x"), getParameter<double>("y")};
      command->lookAtBallFrom(target);
      return Status::SUCCESS;
    });
}

void CmdLookAtBallFrom::print(std::ostream & os) const
{
  os << "[CmdLookAtBallFrom] x: " << getParameter<double>("x")
     << " y: " << getParameter<double>("y");
}

}  // namespace crane::skills
