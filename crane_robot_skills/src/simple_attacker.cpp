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
  setParameter("receiver_id", 0);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      if (
        world_model->play_situation.getSituationCommandID() ==
        crane_msgs::msg::PlaySituation::STOP) {
        auto ball = world_model->ball.pos;
        command->setTargetPosition(
          ball + (world_model->getOurGoalCenter() - ball).normalized() * 0.6);
        command->lookAtBall();
        return Status::RUNNING;
      }

      auto [best_angle, goal_angle_width] =
        world_model->getLargestGoalAngleRangeFromPoint(world_model->ball.pos);
      Point best_target = world_model->ball.pos + getNormVec(best_angle) * 0.5;

      // シュートの隙がないときは仲間へパス
      if (goal_angle_width < 0.07) {
        auto our_robots = world_model->ours.getAvailableRobots(robot->id);
        int receiver_id = getParameter<int>("receiver_id");
        if (auto receiver = std::find_if(
              our_robots.begin(), our_robots.end(), [&](auto e) { return e->id == receiver_id; });
            receiver != our_robots.end()) {
          best_target = receiver->get()->pose.pos;
        } else {
          auto nearest_robot =
            world_model->getNearestRobotsWithDistanceFromPoint(world_model->ball.pos, our_robots);
          best_target = nearest_robot.first->pose.pos;
        }

        // 特に自コートでは後ろ向きの攻撃をしない
        if (
          (world_model->ball.pos.x() - best_target.x()) > 0 &&
          (best_target - world_model->getTheirGoalCenter()).norm() > 4.0) {
          best_target = world_model->getTheirGoalCenter();
        }
      }

      Point ball_pos = world_model->ball.pos + world_model->ball.vel * 0.0;
      // 経由ポイント
      Point intermediate_point = ball_pos + (ball_pos - best_target).normalized() * 0.3;

      double dot =
        (robot->pose.pos - ball_pos).normalized().dot((ball_pos - best_target).normalized());
      double target_theta = getAngle(best_target - ball_pos);
      // ボールと敵ゴールの延長線上にいない && 角度があってないときは，中間ポイントを経由
      if (
        (dot < 0.9 && (robot->pose.pos - ball_pos).norm() > 0.1) ||
        std::abs(getAngleDiff(target_theta, robot->pose.theta)) > 0.2) {
        command->setTargetPosition(intermediate_point);
        command->enableCollisionAvoidance();
        command->enableBallAvoidance();
        // ワンタッチシュート時にキックできるようにキッカーをONにしておく
        command->kickStraight(0.5);
      } else {
        command->setTargetPosition(ball_pos + (best_target - ball_pos).normalized() * 0.5);
        command->kickStraight(0.5).disableCollisionAvoidance();
        command->enableCollisionAvoidance();
        command->disableBallAvoidance();
      }
      command->setTerminalVelocity(world_model->ball.vel.norm() * 2.0);
      command->liftUpDribbler();
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
