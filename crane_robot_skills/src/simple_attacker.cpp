// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/simple_attacker.hpp>

namespace crane::skills
{
SimpleAttacker::SimpleAttacker(RobotCommandWrapperBase::SharedPtr & base)
: SkillBaseWithState<SimpleAttackerState>("SimpleAttacker", base, SimpleAttackerState::ENTRY_POINT),
  kick_target(getContextReference<Point>("kick_target")),
  kick_skill(base)
{
  setParameter("receiver_id", 0);
  addStateFunction(
    SimpleAttackerState::ENTRY_POINT,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      std::cout << "ENTRY_POINT" << std::endl;
      kick_target = [&]() -> Point {
        auto [best_angle, goal_angle_width] =
          world_model()->getLargestGoalAngleRangeFromPoint(world_model()->ball.pos);
        // シュートの隙がないときは仲間へパス
        if (goal_angle_width < 0.07) {
          auto our_robots = world_model()->ours.getAvailableRobots(robot()->id);
          int receiver_id = getParameter<int>("receiver_id");
          Point target;
          if (auto receiver = std::find_if(
                our_robots.begin(), our_robots.end(), [&](auto e) { return e->id == receiver_id; });
              receiver != our_robots.end()) {
            target = receiver->get()->pose.pos;
          } else {
            auto nearest_robot = world_model()->getNearestRobotWithDistanceFromPoint(
              world_model()->ball.pos, our_robots);
            target = nearest_robot.first->pose.pos;
          }

          // 特に自コートでは後ろ向きの攻撃をしない
          if (
            (world_model()->ball.pos.x() - target.x()) > 0 &&
            (target - world_model()->getTheirGoalCenter()).norm() > 4.0) {
            target = world_model()->getTheirGoalCenter();
          }
          return target;
        } else {
          // シュートの隙があるときはシュート
          return world_model()->ball.pos + getNormVec(best_angle) * 0.5;
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
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      std::cout << "THROUGH" << std::endl;
      Segment ball_line{
        world_model()->ball.pos, world_model()->ball.pos + world_model()->ball.vel * 3.0};
      auto closest_point = getClosestPointAndDistance(robot()->pose.pos, ball_line).closest_point;
      // ボールラインから一旦遠ざかる(0.5m)
      command.setTargetPosition(
        closest_point + (robot()->pose.pos - closest_point).normalized() * 0.5);
      command.enableBallAvoidance();
      return Status::RUNNING;
    });

  addTransition(
    SimpleAttackerState::ENTRY_POINT, SimpleAttackerState::RECEIVE_APPROACH, [this]() -> bool {
      // 速度がある場合はボールに先回りする形でアプローチ
      return world_model()->ball.vel.norm() > 0.3;
    });

  addStateFunction(
    SimpleAttackerState::RECEIVE_APPROACH,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      std::cout << "RECEIVE_APPROACH" << std::endl;
      auto [min_slack_point, max_slack_point] =
        world_model()->getMinMaxSlackInterceptPoint({robot()});
      auto ball_pos = world_model()->ball.pos;
      auto [closest_point, distance] = [&]() {
        Segment ball_line{ball_pos, ball_pos + world_model()->ball.vel * 10.0};
        auto result = getClosestPointAndDistance(robot()->pose.pos, ball_line);
        return std::make_pair(result.closest_point, result.distance);
      }();
      // 立ちふさがるように経由ポイント
      //      Point target_point = ball_pos + world_model->ball.vel.normalized() *
      //                                        (distance / (robot()->vel.linear.norm() + 0.5) +
      //                                         world_model->ball.vel.norm() * 0.5 + 0.3);
      Point target = [&]() -> Point {
        if (min_slack_point) {
          return min_slack_point.value();
        } else {
          return (ball_pos + world_model()->ball.vel.normalized() * 0.5);
        }
      }();
      command.setTargetPosition(target)
        .setTargetTheta([&]() {
          auto to_target = (kick_target - target).normalized();
          auto to_ball = (world_model()->ball.pos - target).normalized();
          // 0.5m/sのときにボールとゴールの中間方向を向く
          // ボールが速いとよりボールの方向を向く
          return getAngle(to_target + 2.0 * world_model()->ball.vel.norm() * to_ball);
        }())
        .disableBallAvoidance()
        .liftUpDribbler()
        .kickStraight(0.8);
      return Status::RUNNING;
    });

  addTransition(
    SimpleAttackerState::RECEIVE_APPROACH, SimpleAttackerState::ENTRY_POINT,
    [this]() -> bool { return world_model()->ball.vel.norm() < 0.3; });

  addStateFunction(
    SimpleAttackerState::NORMAL_APPROACH,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      std::cout << "NORMAL_APPROACH" << std::endl;
      Point ball_pos = world_model()->ball.pos + world_model()->ball.vel * 0.0;
      auto [min_slack_point, max_slack_point] =
        world_model()->getMinMaxSlackInterceptPoint({robot()});
      kick_skill.setParameter("target", kick_target);
      kick_skill.setParameter("kick_power", 0.8);
      kick_skill.run(visualizer);
      command.setTerminalVelocity(world_model()->ball.vel.norm() * 3.0);
      if (robot()->getDistance(ball_pos) < 0.2) {
        command.disableCollisionAvoidance();
      }
      return Status::RUNNING;
    });

  addTransition(
    SimpleAttackerState::NORMAL_APPROACH, SimpleAttackerState::THROUGH,
    [this]() -> bool { return isBallComingFromBack(0.5); });

  addTransition(SimpleAttackerState::ENTRY_POINT, SimpleAttackerState::STOP, [this]() -> bool {
    return world_model()->play_situation.getSituationCommandID() ==
           crane_msgs::msg::PlaySituation::STOP;
  });

  addTransition(SimpleAttackerState::STOP, SimpleAttackerState::ENTRY_POINT, [this]() -> bool {
    return world_model()->play_situation.getSituationCommandID() !=
           crane_msgs::msg::PlaySituation::STOP;
  });

  addStateFunction(
    SimpleAttackerState::STOP,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      std::cout << "STOP" << std::endl;
      // 自陣ゴールとボールの間に入って一定距離を保つ
      command.setTargetPosition(
        world_model()->ball.pos +
        (world_model()->getOurGoalCenter() - world_model()->ball.pos).normalized() * 1.0);
      command.lookAtBall();
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
  auto ball_pos = world_model()->ball.pos;
  Segment ball_line{ball_pos, ball_pos + world_model()->ball.vel * 3.0};
  auto result = getClosestPointAndDistance(robot()->pose.pos, ball_line);
  // ボールが敵ゴールに向かっているか
  double dot_dir = (world_model()->getTheirGoalCenter() - ball_pos).dot(world_model()->ball.vel);
  // ボールがロボットを追い越そうとしているか
  double dot_inter =
    (result.closest_point - ball_line.first).dot(result.closest_point - ball_line.second);
  return world_model()->ball.vel.norm() > ball_vel_threshold && result.distance < 0.3 &&
         dot_dir > 0. && dot_inter < 0.;
}
}  // namespace crane::skills
