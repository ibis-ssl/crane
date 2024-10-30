// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/kickoff_attack.hpp>

namespace crane::skills
{
KickoffAttack::KickoffAttack(RobotCommandWrapperBase::SharedPtr & base)
: SkillBaseWithState<KickoffAttackState>("KickoffAttack", base, KickoffAttackState::PREPARE_KICKOFF)
{
  setParameter("target_x", 0.0f);
  setParameter("target_y", 1.0f);
  setParameter("kick_power", 0.25);
  addStateFunction(
    KickoffAttackState::PREPARE_KICKOFF,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      if (not go_over_ball) {
        go_over_ball = std::make_shared<GoOverBall>(command_base);
        go_over_ball->setParameter("next_target_x", getParameter<double>("target_x"));
        go_over_ball->setParameter("next_target_y", getParameter<double>("target_y"));
        go_over_ball->setParameter("margin", 0.3);
        command.setMaxVelocity(0.5);
        command.disableRuleAreaAvoidance();
      }
      go_over_ball_status = go_over_ball->run(visualizer);
      return Status::RUNNING;
    });
  addTransition(KickoffAttackState::PREPARE_KICKOFF, KickoffAttackState::KICKOFF, [this]() -> bool {
    return go_over_ball_status == Status::SUCCESS;
  });

  addStateFunction(
    KickoffAttackState::KICKOFF,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command.setMaxVelocity(0.5);
      command.liftUpDribbler();
      command.kickStraight(getParameter<double>("kick_power"));
      command.setTargetPosition(world_model()->ball.pos);
      command.setTerminalVelocity(0.5);
      command.disableBallAvoidance();
      command.disableRuleAreaAvoidance();
      if (world_model()->ball.vel.norm() > 0.3) {
        return Status::SUCCESS;
      } else {
        return Status::RUNNING;
      }
    });
}
}  // namespace crane::skills
