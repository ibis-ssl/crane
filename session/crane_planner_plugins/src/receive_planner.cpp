// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/receive_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
ReceivePlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
{
  auto dpps_points = getDPPSPoints(world_model->ball.pos, 0.25, 16);

  std::vector<crane_msgs::msg::RobotCommand> commands;
  for (const auto & robot : robots) {
    auto robot_info = world_model->getRobot(robot);
    crane::RobotCommandWrapper target(robot.robot_id, world_model);
    // モード判断
    //  こちらへ向かう速度成分
    auto ball_vel =
      world_model->ball.vel.dot((robot_info->pose.pos - world_model->ball.pos).normalized());
    target.liftUpDribbler();
    target.kickStraight(0.4);
    if (ball_vel > 0.5) {
      Segment ball_line(
        world_model->ball.pos,
        (world_model->ball.pos + world_model->ball.vel.normalized() *
                                   (world_model->ball.pos - robot_info->pose.pos).norm()));

      Segment goal_line(
        world_model->getTheirGoalPosts().first, world_model->getTheirGoalPosts().second);
      if (bg::intersects(ball_line, goal_line)) {
        // シュートをブロックしない
        target.stopHere();
      } else {
        //  ボールの進路上に移動
        ClosestPoint result;
        bg::closest_point(robot_info->pose.pos, ball_line, result);

        // ゴールとボールの中間方向を向く
        auto [goal_angle, width] =
          world_model->getLargestGoalAngleRangeFromPoint(result.closest_point);
        auto to_goal = getNormVec(goal_angle);
        auto to_ball = (world_model->ball.pos - result.closest_point).normalized();
        double intermediate_angle = getAngle(2 * to_goal + to_ball);
        target.setTargetTheta(intermediate_angle);

        // キッカーの中心のためのオフセット
        target.setTargetPosition(
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
        const double dist = world_model->getDistanceFromRobot(robot, dpps_point);
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
      target.setTargetPosition(best_position);
    }

    // ゴールとボールの中間方向を向く
    Point target_pos{target.latest_msg.target_x.front(), target.latest_msg.target_y.front()};
    auto [goal_angle, width] = world_model->getLargestGoalAngleRangeFromPoint(target_pos);
    auto to_goal = getNormVec(goal_angle);
    auto to_ball = (world_model->ball.pos - target_pos).normalized();
    target.setTargetTheta(getAngle(to_goal + to_ball));

    commands.push_back(target.getMsg());
  }
  return {PlannerBase::Status::RUNNING, commands};
}

std::vector<std::pair<double, Point>> ReceivePlanner::getPositionsWithScore(
  double dpps_unit, int dpps_num, Point base)
{
  auto dpps_points = getDPPSPoints(world_model->ball.pos, 0.25, 16);
  std::vector<std::pair<double, Point>> position_with_score;
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
    double score = width;
    // 反射角　小さいほどよい
    auto reflect_angle = getAngleDiff(angle, getAngle(world_model->ball.pos - dpps_point));
    score *= (1.0 - std::min(reflect_angle, 1.0));
    // 距離 大きいほどよい
    const double dist = (base - dpps_point).norm();
    score = score * (1.0 - dist / 10.0);
    position_with_score.push_back({score, dpps_point});
  }
  return position_with_score;
}

auto ReceivePlanner::getPassResponse(
  const std::shared_ptr<crane_msgs::srv::PassRequest::Request> & request)
  -> crane_msgs::srv::PassRequest::Response
{
  //    RCLCPP_INFO(get_logger(), "receive pass request!");
  pass_info.passer_id = request->pass_plan.passer_id;
  pass_info.receiver_id = request->pass_plan.receiver_id;
  auto & ball = world_model->ball;
  auto pos = world_model->ours.robots.at(pass_info.passer_id.data)->pose.pos;
  //  こちらへ向かう速度成分
  auto ball_vel = ball.vel.dot((pos - ball.pos).normalized());
  Segment ball_line(ball.pos, (ball.pos + ball.vel.normalized() * (ball.pos - pos).norm()));
  Point target;
  if (ball_vel > 0.5f) {
    //  ボールの進路上に移動
    ClosestPoint result;
    bg::closest_point(pos, ball_line, result);
    if (
      not world_model->isDefenseArea(result.closest_point) and
      world_model->isFieldInside(result.closest_point)) {
      target = result.closest_point;
    }
  }

  auto & recv_pos = pass_info.passer_receive_position;
  recv_pos.x = target.x();
  recv_pos.y = target.y();
  recv_pos.z = 0.0;
  //    pass_info_pub->publish(pass_info);

  RobotIdentifier receiver_id{true, static_cast<uint8_t>(pass_info.receiver_id.data)};
  auto receiver = world_model->getRobot(receiver_id);
  //    if (!receiver) {
  //      return;
  //    }

  std::vector<PositionsWithScore> positions_with_score;

  auto receive_pos_candidates = getPoints(receiver->pose.pos, 0.05, 20);
  for (const auto & receive_pos : receive_pos_candidates) {
    auto pos_score = getPositionsWithScore(ball_line, receive_pos);
    auto max_score_pos = std::max_element(
      pos_score.begin(), pos_score.end(),
      [](const auto & a, const auto & b) { return a.first < b.first; });
    PositionsWithScore record;
    record.score = max_score_pos->first;
    record.passer_pos = max_score_pos->second;
    record.receiver_pos = receive_pos;
    positions_with_score.emplace_back(record);
  }

  auto max_record = std::max_element(
    positions_with_score.begin(), positions_with_score.end(),
    [](const auto & a, const auto & b) { return a.score < b.score; });

  crane_msgs::srv::PassRequest::Response response;
  std::tie(response.passer_target_pose.theta, response.receiver_target_pose.theta) =
    calcRobotsTargetAngle(*max_record, ball_line);

  response.passer_target_pose.x = max_record->passer_pos.x();
  response.passer_target_pose.y = max_record->passer_pos.y();

  response.receiver_target_pose.x = max_record->receiver_pos.x();
  response.receiver_target_pose.y = max_record->receiver_pos.y();

  response.success = true;
  response.message = "publish receiver point successfully";

  return response;
}
std::pair<double, double> ReceivePlanner::calcRobotsTargetAngle(
  const ReceivePlanner::PositionsWithScore & record, const Segment & ball_line)
{
  std::pair<double, double> ret;
  // calculate passer angle
  auto passer2ball = (ball_line.first - record.passer_pos).normalized();
  auto passer2receiver = (record.receiver_pos - record.passer_pos).normalized();
  auto passer_direction = passer2receiver + passer2ball;
  ret.first = atan2(passer_direction.y(), passer_direction.x());

  ret.second = atan2(-passer2receiver.y(), -passer2receiver.x());

  return ret;
}

std::vector<std::pair<double, Point>> ReceivePlanner::getPositionsWithScore(
  const Segment & ball_line, const Point & next_target) const
{
  auto points = getPoints(ball_line, 0.05);
  std::vector<std::pair<double, Point>> position_with_score;
  for (const auto & point : points) {
    double score = getPointScore(point, next_target);
    position_with_score.emplace_back(score, point);
  }
  return position_with_score;
}

std::vector<Point> ReceivePlanner::getPoints(const Segment & ball_line, double interval) const
{
  std::vector<Point> points;
  double ball_line_len = (ball_line.first - ball_line.second).norm();
  auto norm_vec = (ball_line.second - ball_line.first).normalized();
  for (double d = 0.0; d <= ball_line_len; d += interval) {
    points.emplace_back(ball_line.first + d * norm_vec);
  }
  return points;
}
std::vector<Point> ReceivePlanner::getPoints(const Point & center, double unit, int unit_num) const
{
  std::vector<Point> points;
  for (double x = center.x() - unit * (unit_num / 2.f); x <= center.x() + unit * (unit_num / 2.f);
       x += unit) {
    for (double y = center.y() - unit * (unit_num / 2.f); y <= center.y() + unit * (unit_num / 2.f);
         y += unit) {
      points.emplace_back(x, y);
    }
  }
  return points;
}
std::vector<Point> ReceivePlanner::getDPPSPoints(
  Point center, double r_resolution, int theta_div_num) const
{
  std::vector<Point> points;
  for (int theta_index = 0; theta_index < theta_div_num; theta_index++) {
    double theta = 2.0 * M_PI * theta_index / theta_div_num;
    for (double r = r_resolution; r <= 10.0; r += r_resolution) {
      points.emplace_back(center.x() + r * cos(theta), center.y() + r * sin(theta));
    }
  }
  points.erase(
    std::remove_if(
      points.begin(), points.end(),
      [&](const auto & point) {
        return (not world_model->isFieldInside(point)) or world_model->isDefenseArea(point);
      }),
    points.end());

  return points;
}
double ReceivePlanner::getPointScore(const Point & p, const Point & next_target) const
{
  double nearest_dist;
  RobotIdentifier receiver{true, static_cast<uint8_t>(session_info.receiver_id)};
  return evaluation::getNextTargetVisibleScore(p, next_target, world_model) *
         evaluation::getReachScore(receiver, p, nearest_dist, world_model) *
         evaluation::getAngleScore(receiver, p, next_target, world_model) *
         evaluation::getEnemyDistanceScore(p, world_model);
}
}  // namespace crane
