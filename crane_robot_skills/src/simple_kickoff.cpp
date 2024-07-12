// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/simple_kickoff.hpp>

namespace crane::skills
{
SimpleKickOff::SimpleKickOff(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("SimpleKickOff", id, wm, DefaultStates::DEFAULT)
{
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      Point intermediate_point =
        world_model->ball.pos +
        (world_model->ball.pos - world_model->getTheirGoalCenter()).normalized() * 0.3;

      double dot = (robot->pose.pos - world_model->ball.pos)
                     .normalized()
                     .dot((world_model->ball.pos - world_model->getTheirGoalCenter()).normalized());
      double target_theta = getAngle(world_model->getTheirGoalCenter() - world_model->ball.pos);
      auto cmd = std::make_shared<RobotCommandWrapperPosition>(command);
      // ボールと敵ゴールの延長線上にいない && 角度があってないときは，中間ポイントを経由
      if (
        (dot < 0.95 && (robot->pose.pos - world_model->ball.pos).norm() > 0.1) ||
        std::abs(getAngleDiff(target_theta, robot->pose.theta)) > 0.2) {
        cmd->setTargetPosition(
          world_model->ball.pos +
          (world_model->ball.pos - world_model->getTheirGoalCenter()).normalized() * 0.3);
      } else {
        cmd->setTargetPosition(
          world_model->ball.pos +
          (world_model->getTheirGoalCenter() - world_model->ball.pos).normalized() * 0.3);
        cmd->dribble(0.3);
        cmd->kickWithChip(1.0);
        cmd->disableBallAvoidance();
      }
      cmd->setTargetTheta(target_theta);
      return Status::RUNNING;
    });
}
}  // namespace crane::skills
