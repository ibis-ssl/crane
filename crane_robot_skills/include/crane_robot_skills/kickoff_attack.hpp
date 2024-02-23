// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__KICKOFF_ATTACK_HPP_
#define CRANE_ROBOT_SKILLS__KICKOFF_ATTACK_HPP_

#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/go_over_ball.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
enum class KickoffAttackState {
  PREPARE_KICKOFF,
  KICKOFF,
};
class KickoffAttack : public SkillBase<KickoffAttackState>
{
public:
  explicit KickoffAttack(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
  : SkillBase<KickoffAttackState>("KickoffAttack", id, wm, KickoffAttackState::PREPARE_KICKOFF)
  {
    setParameter("target_x", 0.0f);
    setParameter("target_y", 1.0f);
    setParameter("kick_power", 0.5);
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
        }
        go_over_ball_status = go_over_ball->run(visualizer);
        return Status::RUNNING;
      });
    addTransition(
      KickoffAttackState::PREPARE_KICKOFF, KickoffAttackState::KICKOFF,
      [this]() -> bool { return go_over_ball_status == Status::SUCCESS; });

    addStateFunction(
      KickoffAttackState::KICKOFF,
      [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
        command->setMaxVelocity(0.5);
        command->kickStraight(getParameter<double>("kick_power"));
        command->setTargetPosition(world_model->ball.pos);
        command->setTerminalVelocity(0.5);
        if (world_model->ball.vel.norm() > 0.3) {
          return Status::SUCCESS;
        } else {
          return Status::RUNNING;
        }
      });
  }

  void print(std::ostream & os) const override { os << "[KickoffAttack]"; }

private:
  std::shared_ptr<GoOverBall> go_over_ball = nullptr;

  Status go_over_ball_status = Status::RUNNING;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__KICKOFF_ATTACK_HPP_
