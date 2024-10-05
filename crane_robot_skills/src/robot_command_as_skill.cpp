// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/robot_command_as_skill.hpp>
#include <memory>

namespace crane::skills
{

#define ONE_FRAME_IMPLEMENTATION(name, method)                                                     \
  Cmd##name::Cmd##name(RobotCommandWrapperBase::SharedPtr & base)                                  \
  : SkillBase("Cmd" #name, base) {}                                                                \
  Status Cmd##name::update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) \
  {                                                                                                \
    command.method;                                                                                \
    return Status::SUCCESS;                                                                        \
  }                                                                                                \
  void Cmd##name::print([[maybe_unused]] std::ostream & os) const {}

CmdKickWithChip::CmdKickWithChip(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("CmdKickWithChip", base)
{
  setParameter("power", 0.5);
}

Status CmdKickWithChip::update(
  [[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  command.kickWithChip(getParameter<double>("power"));
  return Status::SUCCESS;
}

void CmdKickWithChip::print(std::ostream & os) const
{
  os << "[CmdKickWithChip] power: " << getParameter<double>("power");
}

CmdKickStraight::CmdKickStraight(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("CmdKickStraight", base)
{
  setParameter("power", 0.5);
}

Status CmdKickStraight::update(
  [[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  command.kickStraight(getParameter<double>("power"));
  return Status::SUCCESS;
}

void CmdKickStraight::print(std::ostream & os) const
{
  os << "[CmdKickStraight] power: " << getParameter<double>("power");
}

CmdDribble::CmdDribble(RobotCommandWrapperBase::SharedPtr & base) : SkillBase("CmdDribble", base)
{
  setParameter("power", 0.5);
}

Status CmdDribble::update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  command.dribble(getParameter<double>("power"));
  return Status::SUCCESS;
}

void CmdDribble::print(std::ostream & os) const
{
  os << "[CmdDribble] power: " << getParameter<double>("power");
}

CmdSetVelocity::CmdSetVelocity(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("CmdSetVelocity", base)
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
}

Status CmdSetVelocity::update(
  [[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  command.setVelocity(getParameter<double>("x"), getParameter<double>("y"));
  return Status::SUCCESS;
}

void CmdSetVelocity::print(std::ostream & os) const
{
  os << "[CmdSetVelocity] x: " << getParameter<double>("x") << " y: " << getParameter<double>("y");
}

CmdSetTargetPosition::CmdSetTargetPosition(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("CmdSetTargetPosition", base)
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
  setParameter("tolerance", 0.0);
  setParameter("reach_threshold", 0.1);
  setParameter("exit_immediately", false);
}

Status CmdSetTargetPosition::update(
  [[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  Point target{getParameter<double>("x"), getParameter<double>("y")};
  command.setTargetPosition(target, getParameter<double>("tolerance"));
  if (getParameter<bool>("exit_immediately")) {
    return Status::SUCCESS;
  } else {
    if (robot()->getDistance(target) <= getParameter<double>("reach_threshold")) {
      return Status::SUCCESS;
    } else {
      return Status::RUNNING;
    }
  }
}

void CmdSetTargetPosition::print(std::ostream & os) const
{
  os << "[CmdSetTargetPosition] distance: "
     << (robot()->pose.pos - Point{getParameter<double>("x"), getParameter<double>("y")}).norm();
}

CmdSetDribblerTargetPosition::CmdSetDribblerTargetPosition(
  RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("CmdSetDribblerTargetPosition", base)
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
  setParameter("theta", 0.0);
  setParameter("position_tolerance", 0.0);
  setParameter("angle_tolerance", 0.0);
  setParameter("reach_threshold", 0.1);
  setParameter("exit_immediately", false);
}

Status CmdSetDribblerTargetPosition::update(
  [[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  Point target{getParameter<double>("x"), getParameter<double>("y")};
  command.setTargetTheta(getParameter<double>("theta"), getParameter<double>("angle_tolerance"));
  command.setDribblerTargetPosition(target, getParameter<double>("position_tolerance"));
  if (getParameter<bool>("exit_immediately")) {
    return Status::SUCCESS;
  } else {
    if ((robot()->kicker_center() - target).norm() <= getParameter<double>("reach_threshold")) {
      return Status::SUCCESS;
    } else {
      return Status::RUNNING;
    }
  }
}

void CmdSetDribblerTargetPosition::print(std::ostream & os) const
{
  os << "[CmdSetDribblerTargetPosition] distance: "
     << (robot()->kicker_center() - Point{getParameter<double>("x"), getParameter<double>("y")})
          .norm();
}

CmdSetTargetTheta::CmdSetTargetTheta(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("CmdSetTargetTheta", base)
{
  setParameter("theta", 0.0);
  setParameter("tolerance", 0.0);
  setParameter("omega_limit", 10.0);
}

Status CmdSetTargetTheta::update(
  [[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  command.setTargetTheta(getParameter<double>("theta"), getParameter<double>("tolerance"))
    .setOmegaLimit(getParameter<double>("omega_limit"));
  return Status::SUCCESS;
}

void CmdSetTargetTheta::print(std::ostream & os) const
{
  os << "[CmdSetTargetTheta] theta: " << getParameter<double>("theta");
}

CmdStopHere::CmdStopHere(RobotCommandWrapperBase::SharedPtr & base) : SkillBase("CmdStopHere", base)
{
}

Status CmdStopHere::update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  command.stopHere();
  return Status::SUCCESS;
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

CmdSetMaxVelocity::CmdSetMaxVelocity(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("CmdSetMaxVelocity", base)
{
  setParameter("max_velocity", 0.5);
}

Status CmdSetMaxVelocity::update(
  [[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  command.setMaxVelocity(getParameter<double>("max_velocity"));
  return Status::SUCCESS;
}

void CmdSetMaxVelocity::print(std::ostream & os) const
{
  os << "[CmdSetMaxVelocity] max_velocity: " << getParameter<double>("max_velocity");
}

CmdSetMaxAcceleration::CmdSetMaxAcceleration(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("CmdSetMaxAcceleration", base)
{
  setParameter("max_acceleration", 0.5);
}

Status CmdSetMaxAcceleration::update(
  [[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  command.setMaxAcceleration(getParameter<double>("max_acceleration"));
  return Status::SUCCESS;
}

void CmdSetMaxAcceleration::print(std::ostream & os) const
{
  os << "[CmdSetMaxAcceleration] max_acceleration: " << getParameter<double>("max_acceleration");
}

CmdSetMaxOmega::CmdSetMaxOmega(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("CmdSetMaxOmega", base)
{
  setParameter("max_omega", 0.5);
}

Status CmdSetMaxOmega::update(
  [[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  command.setMaxOmega(getParameter<double>("max_omega"));
  return Status::SUCCESS;
}

void CmdSetMaxOmega::print(std::ostream & os) const
{
  os << "[CmdSetMaxOmega] max_omega: " << getParameter<double>("max_omega");
}

CmdSetTerminalVelocity::CmdSetTerminalVelocity(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("CmdSetTerminalVelocity", base)
{
  setParameter("terminal_velocity", 0.5);
}

Status CmdSetTerminalVelocity::update(
  [[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  command.setTerminalVelocity(getParameter<double>("terminal_velocity"));
  return Status::SUCCESS;
}

void CmdSetTerminalVelocity::print(std::ostream & os) const
{
  os << "[CmdSetTerminalVelocity] terminal_velocity: " << getParameter<double>("terminal_velocity");
}

CmdEnableStopFlag::CmdEnableStopFlag(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("CmdEnableStopFlag  ", base)
{
}

Status CmdEnableStopFlag::update(
  [[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  command.stopEmergency(true);
  return Status::SUCCESS;
}

void CmdEnableStopFlag::print(std::ostream & os) const { os << "[CmdEnableStopFlag]"; }

CmdDisableStopFlag::CmdDisableStopFlag(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("CmdDisableStopFlag", base)
{
}

Status CmdDisableStopFlag::update(
  [[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  command.stopEmergency(false);
  return Status::SUCCESS;
}

void CmdDisableStopFlag::print(std::ostream & os) const { os << "[CmdDisableStopFlag]"; }

CmdLiftUpDribbler::CmdLiftUpDribbler(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("CmdLiftUpDribbler", base)
{
  setParameter("enable", true);
}

Status CmdLiftUpDribbler::update(
  [[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  command.liftUpDribbler(getParameter<bool>("enable"));
  return Status::SUCCESS;
}

void CmdLiftUpDribbler::print(std::ostream & os) const
{
  os << "[CmdLiftUpDribbler] enable: " << getParameter<bool>("enable");
}

CmdLookAt::CmdLookAt(RobotCommandWrapperBase::SharedPtr & base) : SkillBase("CmdLookAt", base)
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
  setParameter("theta_tolerance", 0.0);
  setParameter("omega_limit", 10.0);
}

Status CmdLookAt::update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  Point target{getParameter<double>("x"), getParameter<double>("y")};
  command.lookAt(target, getParameter<double>("theta_tolerance"))
    .setOmegaLimit(getParameter<double>("omega_limit"));
  return Status::SUCCESS;
}

void CmdLookAt::print(std::ostream & os) const
{
  os << "[CmdLookAt] x: " << getParameter<double>("x") << " y: " << getParameter<double>("y");
}

CmdLookAtBall::CmdLookAtBall(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("CmdLookAtBall", base)
{
  setParameter("theta_tolerance", 0.0);
  setParameter("omega_limit", 10.0);
}

Status CmdLookAtBall::update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  command.lookAtBall(getParameter<double>("theta_tolerance"))
    .setOmegaLimit(getParameter<double>("omega_limit"));
  return Status::SUCCESS;
}

void CmdLookAtBall::print(std::ostream & os) const { os << "[CmdLookAtBall]"; }

CmdLookAtBallFrom::CmdLookAtBallFrom(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("CmdLookAtBallFrom", base)
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
  setParameter("theta_tolerance", 0.0);
  setParameter("omega_limit", 10.0);
}

Status CmdLookAtBallFrom::update(
  [[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  Point target{getParameter<double>("x"), getParameter<double>("y")};
  command.lookAtBallFrom(target, getParameter<double>("theta_tolerance"))
    .setOmegaLimit(getParameter<double>("omega_limit"));
  return Status::SUCCESS;
}

void CmdLookAtBallFrom::print(std::ostream & os) const
{
  os << "[CmdLookAtBallFrom] x: " << getParameter<double>("x")
     << " y: " << getParameter<double>("y");
}
}  // namespace crane::skills
