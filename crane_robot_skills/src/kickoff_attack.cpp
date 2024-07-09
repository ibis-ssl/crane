// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/kickoff_attack.hpp>

namespace crane::skills
{
KickoffAttack::KickoffAttack(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<KickoffAttackState>("KickoffAttack", id, wm, KickoffAttackState::PREPARE_KICKOFF)
{
  setParameter("target_x", 0.0f);
  setParameter("target_y", 1.0f);
  setParameter("kick_power", 0.3);
  addStateFunction(
    KickoffAttackState::PREPARE_KICKOFF,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      if (not go_over_ball) {
        go_over_ball = std::make_shared<GoOverBall>(robot->id, world_model);
        go_over_ball->setCommander(command);
        go_over_ball->setParameter("next_target_x", getParameter<double>("target_x"));
        go_over_ball->setParameter("next_target_y", getParameter<double>("target_y"));
        go_over_ball->setParameter("margin", 0.3);
        command->setMaxVelocity(0.5);
        command->disableRuleAreaAvoidance();
      }
      go_over_ball_status = go_over_ball->run(visualizer);
      return Status::RUNNING;
    });
  addTransition(KickoffAttackState::PREPARE_KICKOFF, KickoffAttackState::KICKOFF, [this]() -> bool {
    return go_over_ball_status == Status::SUCCESS;
  });

  addStateFunction(
    KickoffAttackState::KICKOFF,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      auto cmd = std::dynamic_pointer_cast<RobotCommandWrapperPosition>(command);
      cmd->setMaxVelocity(0.5);
      cmd->liftUpDribbler();
      cmd->kickStraight(getParameter<double>("kick_power"));
      cmd->setTargetPosition(world_model->ball.pos);
      cmd->setTerminalVelocity(0.5);
      cmd->disableBallAvoidance();
      cmd->disableRuleAreaAvoidance();
      if (world_model->ball.vel.norm() > 0.3) {
        return Status::SUCCESS;
      } else {
        return Status::RUNNING;
      }
    });
}
}  // namespace crane::skills
