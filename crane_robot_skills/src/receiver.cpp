// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/receiver.hpp>

namespace crane::skills
{
Receiver::Receiver(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("Receiver", id, wm, DefaultStates::DEFAULT)
{
  //  setParameter("passer_id", 0);
  //  setParameter("receive_x", 0.0);
  //  setParameter("receive_y", 0.0);
  setParameter("ball_vel_threshold", 0.5);
  setParameter("kicker_power", 0.7);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      auto dpps_points = getDPPSPoints(this->world_model->ball.pos, 0.25, 16);
      // モード判断
      //  こちらへ向かう速度成分
      float ball_vel =
        world_model->ball.vel.dot((robot->pose.pos - world_model->ball.pos).normalized());
      if (ball_vel > getParameter<double>("ball_vel_threshold")) {
        Segment ball_line(
          world_model->ball.pos,
          (world_model->ball.pos +
           world_model->ball.vel.normalized() * (world_model->ball.pos - robot->pose.pos).norm()));

        Segment goal_line(
          world_model->getTheirGoalPosts().first, world_model->getTheirGoalPosts().second);
        // シュートをブロックしない
        // TODO(HansRobo): これでは延長線上に相手ゴールのあるパスが全くできなくなるので要修正
        if (bg::intersects(ball_line, goal_line)) {
          command->stopHere();
        } else {
          //  ボールの進路上に移動
          ClosestPoint result;
          bg::closest_point(robot->pose.pos, ball_line, result);

          // ゴールとボールの中間方向を向く
          // TODO(Hansobo): ボールの速さ・キッカーの強さでボールの反射する角度が変わるため、要考慮
          auto [goal_angle, width] =
            world_model->getLargestGoalAngleRangeFromPoint(result.closest_point);
          auto to_goal = getNormVec(goal_angle);
          auto to_ball = (world_model->ball.pos - result.closest_point).normalized();
          double intermediate_angle = getAngle(2 * to_goal + to_ball);
          command->setTargetTheta(intermediate_angle);

          // キッカーの中心のためのオフセット
          command->setTargetPosition(
            result.closest_point - (2 * to_goal + to_ball).normalized() * 0.13);
        }
      } else {
        Point best_position;
        double best_score = 0.0;
        for (const auto & dpps_point : dpps_points) {
          Segment line{world_model->ball.pos, dpps_point};
          double closest_distance = [&]() -> double {
            double closest_distance = std::numeric_limits<double>::max();
            for (const auto & robot : world_model->theirs.getAvailableRobots()) {
              ClosestPoint result;
              bg::closest_point(robot->pose.pos, line, result);
              if (result.distance < closest_distance) {
                closest_distance = result.distance;
              }
            }
            return closest_distance;
          }();

          if (closest_distance < 0.4) {
            continue;
          }

          auto [angle, width] = world_model->getLargestGoalAngleRangeFromPoint(dpps_point);
          // ゴールが見える角度が大きいほどよい
          double score = width;
          // 反射角　小さいほどよい
          auto reflect_angle = getAngleDiff(angle, getAngle(world_model->ball.pos - dpps_point));
          score *= (1.0 - std::min(reflect_angle, 1.0));
          // 距離 大きいほどよい
          const double dist = world_model->getDistanceFromRobot(robot->id, dpps_point);
          score = score * (1.0 - dist / 10.0);

          // シュートラインに近すぎる場所は避ける
          Segment shoot_line{world_model->getOurGoalCenter(), world_model->ball.pos};
          const auto dist_to_shoot_line = bg::distance(dpps_point, shoot_line);
          if (dist_to_shoot_line < 0.5) {
            score = 0.0;
          }

          if (score > best_score) {
            best_score = score;
            best_position = dpps_point;
          }
        }
        command->setTargetPosition(best_position);
      }

      // ゴールとボールの中間方向を向く
      Point target_pos{command->latest_msg.target_x.front(), command->latest_msg.target_y.front()};
      auto [goal_angle, width] = world_model->getLargestGoalAngleRangeFromPoint(target_pos);
      auto to_goal = getNormVec(goal_angle);
      auto to_ball = (world_model->ball.pos - target_pos).normalized();
      command->setTargetTheta(getAngle(to_goal + to_ball));
      command->liftUpDribbler();
      command->kickStraight(getParameter<double>("kicker_power"));

      return Status::RUNNING;
    });
}
}  // namespace crane::skills
