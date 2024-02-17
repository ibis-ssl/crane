// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/receiver.hpp>

namespace crane::skills
{
Receiver::Receiver(uint8_t id, const std::shared_ptr<WorldModelWrapper> & world_model)
: SkillBase<>("Receiver", id, world_model, DefaultStates::DEFAULT)
{
  setParameter("passer_id", 0);
  setParameter("receive_x", 0.0);
  setParameter("receive_y", 0.0);
  setParameter("ball_vel_threshold", 0.5);
  setParameter("kicker_power", 0.7);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot, crane::RobotCommandWrapper & command,
      ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
      auto dpps_points = getDPPSPoints(world_model->ball.pos, 0.25, 16);
      // モード判断
      //  こちらへ向かう速度成分
      float ball_vel =
        world_model->ball.vel.dot((robot->pose.pos - world_model->ball.pos).normalized());
      command.kickStraight(getParameter<double>("kicker_power"));
      if (ball_vel > getParameter<double>("ball_vel_threshold")) {
        Segment ball_line(
          world_model->ball.pos,
          (world_model->ball.pos +
           world_model->ball.vel.normalized() * (world_model->ball.pos - robot->pose.pos).norm()));

        Segment goal_line(
          world_model->getTheirGoalPosts().first, world_model->getTheirGoalPosts().second);
        // シュートをブロックしない
        // TODO: これでは延長線上に相手ゴールのあるパスが全くできなくなるので要修正
        if (bg::intersects(ball_line, goal_line)) {
          command.stopHere();
        } else {
          //  ボールの進路上に移動
          ClosestPoint result;
          bg::closest_point(robot->pose.pos, ball_line, result);

          // ゴールとボールの中間方向を向く
          // TODO: ボールの速さ・キッカーの強さでボールの反射する角度が変わるため、要考慮
          auto goal_angle = getLargestGoalAngleFromPosition(result.closest_point);
          auto to_goal = getNormVec(goal_angle);
          auto to_ball = (world_model->ball.pos - result.closest_point).normalized();
          double intermediate_angle = getAngle(2 * to_goal + to_ball);
          command.setTargetTheta(intermediate_angle);

          // キッカーの中心のためのオフセット
          command.setTargetPosition(
            result.closest_point - (2 * to_goal + to_ball).normalized() * 0.06);
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

          double score = getLargestGoalAngleWidthFromPosition(dpps_point);
          const double dist = (robot->pose.pos - dpps_point).norm();
          score = score * (1.0 - dist / 10.0);

          if (score > best_score) {
            best_score = score;
            best_position = dpps_point;
          }
        }
        command.setTargetPosition(best_position);
        //        target.setTargetTheta(getAngle(world_model->ball.pos - best_position));
      }

      // ゴールとボールの中間方向を向く
      Point target_pos{command.latest_msg.target_x.front(), command.latest_msg.target_y.front()};
      auto goal_angle = getLargestGoalAngleFromPosition(target_pos);
      auto to_goal = getNormVec(goal_angle);
      auto to_ball = (world_model->ball.pos - target_pos).normalized();
      command.setTargetTheta(getAngle(to_goal + to_ball));

      return Status::RUNNING;
    });
}
}  // namespace crane::skills
