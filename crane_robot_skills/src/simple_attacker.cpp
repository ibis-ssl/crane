// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/simple_attacker.hpp>

namespace crane::skills
{
SimpleAttacker::SimpleAttacker(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBaseWithState<SimpleAttackerState>(
    "SimpleAttacker", id, wm, SimpleAttackerState::ENTRY_POINT)
{
  setParameter("receiver_id", 0);
  addStateFunction(
    SimpleAttackerState::ENTRY_POINT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      std::cout << "ENTRY_POINT" << std::endl;
      kick_target = [&]() -> Point {
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
      return Status::RUNNING;
    });

  addTransition(SimpleAttackerState::ENTRY_POINT, SimpleAttackerState::THROUGH, [this]() -> bool {
    return isBallComingFromBack(0.5);
  });

  addTransition(SimpleAttackerState::THROUGH, SimpleAttackerState::ENTRY_POINT, [this]() -> bool {
    return not isBallComingFromBack(0.5);
  });

  addStateFunction(
    SimpleAttackerState::THROUGH,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      std::cout << "THROUGH" << std::endl;
      Segment ball_line{world_model->ball.pos, world_model->ball.pos + world_model->ball.vel * 3.0};
      auto closest_point = getClosestPointAndDistance(robot->pose.pos, ball_line).closest_point;
      // ボールラインから一旦遠ざかる(0.5m)
      command
        ->setTargetPosition(closest_point + (robot->pose.pos - closest_point).normalized() * 0.5)
        .enableBallAvoidance();
      return Status::RUNNING;
    });

  addTransition(
    SimpleAttackerState::ENTRY_POINT, SimpleAttackerState::RECEIVE_APPROACH, [this]() -> bool {
      // 速度がある場合はボールに先回りする形でアプローチ
      return world_model->ball.vel.norm() > 0.3;
    });

  addStateFunction(
    SimpleAttackerState::RECEIVE_APPROACH,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      std::cout << "RECEIVE_APPROACH" << std::endl;
      auto ball_pos = world_model->ball.pos;
      auto [closest_point, distance] = [&]() {
        Segment ball_line{ball_pos, ball_pos + world_model->ball.vel * 10.0};
        auto result = getClosestPointAndDistance(robot->pose.pos, ball_line);
        return std::make_pair(result.closest_point, result.distance);
      }();
      // 立ちふさがるように経由ポイント
      Point target_point = ball_pos + world_model->ball.vel.normalized() *
                                        (distance / (robot->vel.linear.norm() + 0.5) +
                                         world_model->ball.vel.norm() * 0.5 + 0.3);
      command->setTargetPosition(target_point)
        .setTargetTheta([&]() {
          auto to_target = (kick_target - target_point).normalized();
          auto to_ball = (world_model->ball.pos - target_point).normalized();
          // 0.5m/sのときにボールとゴールの中間方向を向く
          // ボールが速いとよりボールの方向を向く
          return getAngle(to_target + 2.0 * world_model->ball.vel.norm() * to_ball);
        }())
        .disableBallAvoidance()
        .liftUpDribbler()
        .kickStraight(0.8);
      return Status::RUNNING;
    });

  addTransition(
    SimpleAttackerState::RECEIVE_APPROACH, SimpleAttackerState::ENTRY_POINT,
    [this]() -> bool { return world_model->ball.vel.norm() < 0.3; });

  addStateFunction(
    SimpleAttackerState::NORMAL_APPROACH,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      std::cout << "NORMAL_APPROACH" << std::endl;
      Point ball_pos = world_model->ball.pos + world_model->ball.vel * 0.0;
      // 経由ポイント
      Point intermediate_point = ball_pos + (ball_pos - kick_target).normalized() * 0.3;
      intermediate_point += (intermediate_point - robot->pose.pos) * 0.1;

      double dot =
        (robot->pose.pos - ball_pos).normalized().dot((ball_pos - kick_target).normalized());
      double target_theta = getAngle(kick_target - ball_pos);
      // ボールと敵ゴールの延長線上にいない && 角度があってないときは，中間ポイントを経由
      if (
        (dot < 0.95 && (robot->pose.pos - ball_pos).norm() > 0.1) ||
        std::abs(getAngleDiff(target_theta, robot->pose.theta)) > 0.2) {
        command->setTargetPosition(intermediate_point)
          .enableCollisionAvoidance()
          .enableBallAvoidance()
          .kickStraight(0.8);  // ワンタッチシュート時にキックできるようにキッカーをONにしておく
        // 後ろからきたボールは一旦避ける
        Segment ball_line{ball_pos, ball_pos + world_model->ball.vel * 3.0};
        auto result = getClosestPointAndDistance(robot->pose.pos, ball_line);
        // ボールが敵ゴールに向かっているか
        double dot_dir = (world_model->getTheirGoalCenter() - ball_pos).dot(world_model->ball.vel);
        // ボールがロボットを追い越そうとしているか
        double dot_inter =
          (result.closest_point - ball_line.first).dot(result.closest_point - ball_line.second);

        if (result.distance < 0.3 && dot_dir > 0. && dot_inter < 0.) {
          // ボールラインから一旦遠ざかる
          command
            ->setTargetPosition(
              result.closest_point + (robot->pose.pos - result.closest_point).normalized() * 0.5)
            .enableBallAvoidance();
        }
      } else {
        command->setTargetPosition(ball_pos + (kick_target - ball_pos).normalized() * 0.5)
          .kickStraight(0.8)
          .disableCollisionAvoidance()
          .enableCollisionAvoidance()
          .disableBallAvoidance();
      }
      command->setTerminalVelocity(world_model->ball.vel.norm() * 3.0)
        .liftUpDribbler()
        .setTargetTheta(getAngle(kick_target - world_model->ball.pos));
      return Status::RUNNING;
    });

  addTransition(SimpleAttackerState::ENTRY_POINT, SimpleAttackerState::STOP, [this]() -> bool {
    return world_model->play_situation.getSituationCommandID() ==
           crane_msgs::msg::PlaySituation::STOP;
  });

  addTransition(SimpleAttackerState::STOP, SimpleAttackerState::ENTRY_POINT, [this]() -> bool {
    return world_model->play_situation.getSituationCommandID() !=
           crane_msgs::msg::PlaySituation::STOP;
  });

  addStateFunction(
    SimpleAttackerState::STOP,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      std::cout << "STOP" << std::endl;
      // 自陣ゴールとボールの間に入って一定距離を保つ
      command
        ->setTargetPosition(
          world_model->ball.pos +
          (world_model->getOurGoalCenter() - world_model->ball.pos).normalized() * 1.0)
        .lookAtBall();
      return Status::RUNNING;
    });

  addTransition(
    SimpleAttackerState::ENTRY_POINT, SimpleAttackerState::NORMAL_APPROACH, [this]() -> bool {
      // THROUGHでもRECEIVE_APPROACHでもない場合
      return true;
    });
}

bool SimpleAttacker::isBallComingFromBack(double ball_vel_threshold) const
{
  auto ball_pos = world_model->ball.pos;
  Segment ball_line{ball_pos, ball_pos + world_model->ball.vel * 3.0};
  auto result = getClosestPointAndDistance(robot->pose.pos, ball_line);
  // ボールが敵ゴールに向かっているか
  double dot_dir = (world_model->getTheirGoalCenter() - ball_pos).dot(world_model->ball.vel);
  // ボールがロボットを追い越そうとしているか
  double dot_inter =
    (result.closest_point - ball_line.first).dot(result.closest_point - ball_line.second);
  return world_model->ball.vel.norm() > ball_vel_threshold && result.distance < 0.3 &&
         dot_dir > 0. && dot_inter < 0.;
}
}  // namespace crane::skills
