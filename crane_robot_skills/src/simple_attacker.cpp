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
          ball + (world_model->getOurGoalCenter() - ball).normalized() * 1.0);
        command->lookAtBall();
        return Status::RUNNING;
      }

      Point best_target = [&]() {
        auto [best_angle, goal_angle_width] =
          world_model->getLargestGoalAngleRangeFromPoint(world_model->ball.pos);
        // シュートの隙がないときは仲間へパス
        if (goal_angle_width < 0.07) {
          auto our_robots = world_model->ours.getAvailableRobots(robot->id);
          int receiver_id = getParameter<int>("receiver_id");
          Point target;
          if (auto receiver = std::find_if(
                our_robots.begin(), our_robots.end(), [&](auto e) { return e->id == receiver_id; });
              receiver != our_robots.end()) {
            target = receiver->get()->pose.pos;
          } else {
            auto nearest_robot =
              world_model->getNearestRobotsWithDistanceFromPoint(world_model->ball.pos, our_robots);
            target = nearest_robot.first->pose.pos;
          }

          // 特に自コートでは後ろ向きの攻撃をしない
          if (
            (world_model->ball.pos.x() - target.x()) > 0 &&
            (target - world_model->getTheirGoalCenter()).norm() > 4.0) {
            target = world_model->getTheirGoalCenter();
          }
          return target;
        } else {
          return world_model->ball.pos + getNormVec(best_angle) * 0.5;
        }
      }();

      // ボールに速度がある場合
      if (world_model->ball.vel.norm() > 0.5) {
        // 後ろからきたボールは一旦避ける
        bool avoid_flag = [&]() {
          Segment ball_line{ball_pos, ball_pos + world_model->ball.vel * 3.0};
          ClosestPoint result;
          bg::closest_point(robot->pose.pos, ball_line, result);
          // ボールが敵ゴールに向かっているか
          double dot_dir =
            (world_model->getTheirGoalCenter() - ball_pos).dot(world_model->ball.vel);
          // ボールがロボットを追い越そうとしているか
          double dot_inter =
            (result.closest_point - ball_line.first).dot(result.closest_point - ball_line.second);
          return result.distance < 0.3 && dot_dir > 0. && dot_inter < 0.;
        }();

        if (avoid_flag) {
          // ボールラインから一旦遠ざかる
          command->setTargetPosition(
            result.closest_point + (robot->pose.pos - result.closest_point).normalized() * 0.5);
          command->enableBallAvoidance();
        } else {
          // ボールにスピードがあるときは立ちふさがるように
          auto [closest_point, distance] = [&]() {
            Segment ball_line{ball_pos, ball_pos + world_model->ball.vel * 10.0};
            ClosestPoint result;
            bg::closest_point(robot->pose.pos, ball_line, result);
            return std::make_pair(result.closest_point, result.distance);
          }();
          // 経由ポイント
          Point intermediate_point =
            ball_pos + world_model->ball.vel.normalized() *
                         (distance + world_model->ball.vel.norm() * 1.0 + 0.3);
          // double dot =
          // (robot->pose.pos - ball_pos).normalized().dot((ball_pos - best_target).normalized());
          // double target_theta = getAngle(best_target - ball_pos);
          // // ボールと敵ゴールの延長線上にいない && 角度があってないときは，中間ポイントを経由
          // if (
          //            (dot < 0.95 && (robot->pose.pos - ball_pos).norm() > 0.1) ||
          //            std::abs(getAngleDiff(target_theta, robot->pose.theta)) > 0.2) {
          //            command->setTargetPosition(intermediate_point);
          //            command->enableCollisionAvoidance();
          //            command->enableBallAvoidance();
          //            // ワンタッチシュート時にキックできるようにキッカーをONにしておく
          //            command->kickStraight(0.8);
          //          }
        }
      } else {
        Point ball_pos = world_model->ball.pos + world_model->ball.vel * 0.0;
        // 経由ポイント
        Point intermediate_point = ball_pos + (ball_pos - best_target).normalized() * 0.3;
        intermediate_point += (intermediate_point - robot->pose.pos) * 0.1;

        double dot =
          (robot->pose.pos - ball_pos).normalized().dot((ball_pos - best_target).normalized());
        double target_theta = getAngle(best_target - ball_pos);
        // ボールと敵ゴールの延長線上にいない && 角度があってないときは，中間ポイントを経由
        if (
          (dot < 0.95 && (robot->pose.pos - ball_pos).norm() > 0.1) ||
          std::abs(getAngleDiff(target_theta, robot->pose.theta)) > 0.2) {
          command->setTargetPosition(intermediate_point);
          command->enableCollisionAvoidance();
          command->enableBallAvoidance();
          // ワンタッチシュート時にキックできるようにキッカーをONにしておく
          command->kickStraight(0.8);

        } else {
          command->setTargetPosition(ball_pos + (best_target - ball_pos).normalized() * 0.5);
          command->kickStraight(0.8).disableCollisionAvoidance();
          command->enableCollisionAvoidance();
          command->disableBallAvoidance();
        }
        command->setTerminalVelocity(world_model->ball.vel.norm() * 3.0);
        command->liftUpDribbler();
        command->setTargetTheta(getAngle(best_target - world_model->ball.pos));

        bool is_in_defense = world_model->isDefenseArea(world_model->ball.pos);
        bool is_in_field = world_model->isFieldInside(world_model->ball.pos);

        if ((not is_in_field) or is_in_defense) {
          command->stopHere();
        }
        return Status::RUNNING;
      }
    });
}
}  // namespace crane::skills
