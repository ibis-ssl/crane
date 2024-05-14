// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SIMPLE_KICKOFF_HPP_
#define CRANE_ROBOT_SKILLS__SIMPLE_KICKOFF_HPP_

#include <algorithm>
#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace crane::skills
{
class SimpleKickOff : public SkillBase<>
{
public:
  explicit SimpleKickOff(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
  : SkillBase<>("SimpleKickOff", id, wm, DefaultStates::DEFAULT)
  {
    addStateFunction(
      DefaultStates::DEFAULT,
      [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
        Point intermediate_point =
          world_model->ball.pos +
          (world_model->ball.pos - world_model->getTheirGoalCenter()).normalized() * 0.3;

        double dot =
          (robot->pose.pos - world_model->ball.pos)
            .normalized()
            .dot((world_model->ball.pos - world_model->getTheirGoalCenter()).normalized());
        double target_theta = getAngle(world_model->getTheirGoalCenter() - world_model->ball.pos);
        // ボールと敵ゴールの延長線上にいない && 角度があってないときは，中間ポイントを経由
        if (
          (dot < 0.95 && (robot->pose.pos - world_model->ball.pos).norm() > 0.1) ||
          std::abs(getAngleDiff(target_theta, robot->pose.theta)) > 0.2) {
          command->setTargetPosition(
            world_model->ball.pos +
            (world_model->ball.pos - world_model->getTheirGoalCenter()).normalized() * 0.3);
        } else {
          command->setTargetPosition(
            world_model->ball.pos +
            (world_model->getTheirGoalCenter() - world_model->ball.pos).normalized() * 0.3);
          command->dribble(0.3);
          command->kickWithChip(1.0);
          command->disableBallAvoidance();
        }
        command->setTargetTheta(target_theta);
        return Status::RUNNING;
      });
  }

  void print(std::ostream & os) const override { os << "[SimpleKickOff]"; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__SIMPLE_KICKOFF_HPP_
