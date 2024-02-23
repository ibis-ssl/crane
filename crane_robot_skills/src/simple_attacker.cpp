// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/simple_attacker.hpp>

namespace crane::skills
{
SimpleAttacker::SimpleAttacker(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("SimpleAttacker", id, wm, DefaultStates::DEFAULT)
{
  addStateFunction(
    DefaultStates::DEFAULT, [this](ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
      auto [best_angle, goal_angle_width] =
        world_model->getLargestGoalAngleRangeFromPoint(world_model->ball.pos);
      Point best_target = world_model->ball.pos + getNormVec(best_angle) * 0.5;

      // シュートの隙がないときは仲間へパス
      if (goal_angle_width < 0.07) {
        auto our_robots = world_model->ours.getAvailableRobots(robot->id);
        auto nearest_robot =
          world_model->getNearestRobotsWithDistanceFromPoint(world_model->ball.pos, our_robots);
        best_target = nearest_robot.first->pose.pos;
      }

      // 経由ポイント

      Point intermediate_point =
        world_model->ball.pos + (world_model->ball.pos - best_target).normalized() * 0.2;

      double dot = (robot->pose.pos - world_model->ball.pos)
                     .normalized()
                     .dot((world_model->ball.pos - best_target).normalized());
      double target_theta = getAngle(best_target - world_model->ball.pos);
      // ボールと敵ゴールの延長線上にいない && 角度があってないときは，中間ポイントを経由
      if (dot < 0.95 || std::abs(getAngleDiff(target_theta, robot->pose.theta)) > 0.05) {
        command->setTargetPosition(intermediate_point);
        command->enableCollisionAvoidance();
      } else {
        command->setTargetPosition(world_model->ball.pos);
        command->kickStraight(0.7).disableCollisionAvoidance();
        command->enableCollisionAvoidance();
        command->disableBallAvoidance();
      }

      command->setTargetTheta(getAngle(best_target - world_model->ball.pos));

      bool is_in_defense = world_model->isDefenseArea(world_model->ball.pos);
      bool is_in_field = world_model->isFieldInside(world_model->ball.pos);

      if ((not is_in_field) or is_in_defense) {
        command->stopHere();
      }
      return Status::RUNNING;
    });
}
}  // namespace crane::skills
