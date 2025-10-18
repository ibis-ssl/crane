// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/robot_command_as_skill.hpp>
#include <memory>

namespace crane::skills
{

#define ONE_FRAME_IMPLEMENTATION(name, method) \
  void Cmd##name::initialize() {}              \
  Status Cmd##name::update()                   \
  {                                            \
    command->method;                           \
    return Status::SUCCESS;                    \
  }                                            \
  void Cmd##name::print([[maybe_unused]] std::ostream & os) const {}

void CmdKickWithChip::initialize() { setParameter("power", 0.5); }

Status CmdKickWithChip::update()
{
  command->kickWithChip(getParameter<double>("power"));
  return Status::SUCCESS;
}

void CmdKickWithChip::print(std::ostream & os) const
{
  os << "[CmdKickWithChip] power: " << getParameter<double>("power");
}

void CmdKickStraight::initialize() { setParameter("power", 0.5); }

Status CmdKickStraight::update()
{
  command->kickStraight(getParameter<double>("power"));
  return Status::SUCCESS;
}

void CmdKickStraight::print(std::ostream & os) const
{
  os << "[CmdKickStraight] power: " << getParameter<double>("power");
}

void CmdDribble::initialize() { setParameter("power", 0.5); }

Status CmdDribble::update()
{
  command->dribble(getParameter<double>("power"));
  return Status::SUCCESS;
}

void CmdDribble::print(std::ostream & os) const
{
  os << "[CmdDribble] power: " << getParameter<double>("power");
}

void CmdSetVelocity::initialize()
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
}

Status CmdSetVelocity::update()
{
  command->setVelocity(getParameter<double>("x"), getParameter<double>("y"));
  return Status::SUCCESS;
}

void CmdSetVelocity::print(std::ostream & os) const
{
  os << "[CmdSetVelocity] x: " << getParameter<double>("x") << " y: " << getParameter<double>("y");
}

void CmdSetTargetPosition::initialize()
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
  setParameter("tolerance", 0.0);
  setParameter("reach_threshold", 0.1);
  setParameter("exit_immediately", false);
}

Status CmdSetTargetPosition::update()
{
  Point target{getParameter<double>("x"), getParameter<double>("y")};
  command->setTargetPosition(target, getParameter<double>("tolerance"));
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

void CmdSetDribblerTargetPosition::initialize()
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
  setParameter("theta", 0.0);
  setParameter("position_tolerance", 0.0);
  setParameter("angle_tolerance", 0.0);
  setParameter("reach_threshold", 0.1);
  setParameter("exit_immediately", false);
}

Status CmdSetDribblerTargetPosition::update()
{
  Point target{getParameter<double>("x"), getParameter<double>("y")};
  command->setTargetTheta(getParameter<double>("theta"), getParameter<double>("angle_tolerance"));
  command->setDribblerTargetPosition(target, getParameter<double>("position_tolerance"));
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

void CmdSetTargetTheta::initialize()
{
  setParameter("theta", 0.0);
  setParameter("tolerance", 0.0);
  setParameter("omega_limit", 10.0);
}

Status CmdSetTargetTheta::update()
{
  command->setTargetTheta(getParameter<double>("theta"), getParameter<double>("tolerance"))
    .setOmegaLimit(getParameter<double>("omega_limit"));
  return Status::SUCCESS;
}

void CmdSetTargetTheta::print(std::ostream & os) const
{
  os << "[CmdSetTargetTheta] theta: " << getParameter<double>("theta");
}

void CmdStopHere::initialize() {}

Status CmdStopHere::update()
{
  command->stopHere();
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

void CmdSetMaxVelocity::initialize() { setParameter("max_velocity", 0.5); }

Status CmdSetMaxVelocity::update()
{
  command->setMaxVelocity("CmdSetMaxVelocity::update", getParameter<double>("max_velocity"));
  return Status::SUCCESS;
}

void CmdSetMaxVelocity::print(std::ostream & os) const
{
  os << "[CmdSetMaxVelocity] max_velocity: " << getParameter<double>("max_velocity");
}

void CmdSetMaxAcceleration::initialize() { setParameter("max_acceleration", 0.5); }

Status CmdSetMaxAcceleration::update()
{
  command->setMaxAcceleration(
    "CmdSetMaxAcceleration skill", getParameter<double>("max_acceleration"));
  return Status::SUCCESS;
}

void CmdSetMaxAcceleration::print(std::ostream & os) const
{
  os << "[CmdSetMaxAcceleration] max_acceleration: " << getParameter<double>("max_acceleration");
}

void CmdSetTerminalVelocity::initialize() { setParameter("terminal_velocity", 0.5); }

Status CmdSetTerminalVelocity::update()
{
  command->setTerminalVelocity(getParameter<double>("terminal_velocity"));
  return Status::SUCCESS;
}

void CmdSetTerminalVelocity::print(std::ostream & os) const
{
  os << "[CmdSetTerminalVelocity] terminal_velocity: " << getParameter<double>("terminal_velocity");
}

void CmdEnableStopFlag::initialize() {}

Status CmdEnableStopFlag::update()
{
  command->stopEmergency(true);
  return Status::SUCCESS;
}

void CmdEnableStopFlag::print(std::ostream & os) const { os << "[CmdEnableStopFlag]"; }

void CmdDisableStopFlag::initialize() {}

Status CmdDisableStopFlag::update()
{
  command->stopEmergency(false);
  return Status::SUCCESS;
}

void CmdDisableStopFlag::print(std::ostream & os) const { os << "[CmdDisableStopFlag]"; }

void CmdLiftUpDribbler::initialize() { setParameter("enable", true); }

Status CmdLiftUpDribbler::update()
{
  command->liftUpDribbler(getParameter<bool>("enable"));
  return Status::SUCCESS;
}

void CmdLiftUpDribbler::print(std::ostream & os) const
{
  os << "[CmdLiftUpDribbler] enable: " << getParameter<bool>("enable");
}

void CmdLookAt::initialize()
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
  setParameter("theta_tolerance", 0.0);
  setParameter("omega_limit", 10.0);
}

Status CmdLookAt::update()
{
  Point target{getParameter<double>("x"), getParameter<double>("y")};
  command->lookAt(target, getParameter<double>("theta_tolerance"))
    .setOmegaLimit(getParameter<double>("omega_limit"));
  return Status::SUCCESS;
}

void CmdLookAt::print(std::ostream & os) const
{
  os << "[CmdLookAt] x: " << getParameter<double>("x") << " y: " << getParameter<double>("y");
}

void CmdLookAtBall::initialize()
{
  setParameter("theta_tolerance", 0.0);
  setParameter("omega_limit", 10.0);
}

Status CmdLookAtBall::update()
{
  command->lookAtBall(getParameter<double>("theta_tolerance"))
    .setOmegaLimit(getParameter<double>("omega_limit"));
  return Status::SUCCESS;
}

void CmdLookAtBall::print(std::ostream & os) const { os << "[CmdLookAtBall]"; }

void CmdLookAtBallFrom::initialize()
{
  setParameter("x", 0.0);
  setParameter("y", 0.0);
  setParameter("theta_tolerance", 0.0);
  setParameter("omega_limit", 10.0);
}

Status CmdLookAtBallFrom::update()
{
  Point target{getParameter<double>("x"), getParameter<double>("y")};
  command->lookAtBallFrom(target, getParameter<double>("theta_tolerance"))
    .setOmegaLimit(getParameter<double>("omega_limit"));
  return Status::SUCCESS;
}

void CmdLookAtBallFrom::print(std::ostream & os) const
{
  os << "[CmdLookAtBallFrom] x: " << getParameter<double>("x")
     << " y: " << getParameter<double>("y");
}
}  // namespace crane::skills
