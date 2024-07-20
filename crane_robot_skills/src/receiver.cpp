// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/receiver.hpp>

namespace crane::skills
{
Receiver::Receiver(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase("Receiver", id, wm)
{
  //  setParameter("passer_id", 0);
  //  setParameter("receive_x", 0.0);
  //  setParameter("receive_y", 0.0);
  setParameter("ball_vel_threshold", 0.2);
  setParameter("kicker_power", 0.8);
}

Status Receiver::update(const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  auto dpps_points = getDPPSPoints(this->world_model->ball.pos, 0.25, 64, world_model);
  // モード判断
  //  こちらへ向かう速度成分
  float ball_vel =
    world_model->ball.vel.dot((robot->pose.pos - world_model->ball.pos).normalized());
  if (
    ball_vel > getParameter<double>("ball_vel_threshold") &&
    ball_vel / world_model->ball.vel.norm() > 0.7) {
    Segment ball_line(
      world_model->ball.pos,
      (world_model->ball.pos +
       world_model->ball.vel.normalized() * (world_model->ball.pos - robot->pose.pos).norm()));

    // 後ろからきたボールは一旦避ける
    Segment short_ball_line{
      world_model->ball.pos, world_model->ball.pos + world_model->ball.vel * 3.0};
    auto result = getClosestPointAndDistance(robot->pose.pos, short_ball_line);
    // ボールが敵ゴールに向かっているか
    double dot_dir =
      (world_model->getTheirGoalCenter() - world_model->ball.pos).dot(world_model->ball.vel);
    // ボールがロボットを追い越そうとしているか
    double dot_inter = (result.closest_point - short_ball_line.first)
                         .dot(result.closest_point - short_ball_line.second);

    if (result.distance < 0.3 && dot_dir > 0. && dot_inter < 0.) {
      // ボールラインから一旦遠ざかる
      command->setTargetPosition(
        result.closest_point + (robot->pose.pos - result.closest_point).normalized() * 0.5);
      command->enableBallAvoidance();
      visualizer->addPoint(
        robot->pose.pos.x(), robot->pose.pos.y(), 0, "red", 1., "ボールラインから一旦遠ざかる");
    } else {
      //  ボールの進路上に移動
      visualizer->addPoint(
        robot->pose.pos.x(), robot->pose.pos.y(), 0, "red", 1., "ボールの進路上に移動");
      auto result = getClosestPointAndDistance(robot->pose.pos, ball_line);

      // ゴールとボールの中間方向を向く
      // TODO(Hansobo): ボールの速さ・キッカーの強さでボールの反射する角度が変わるため、要考慮
      auto [goal_angle, width] =
        world_model->getLargestGoalAngleRangeFromPoint(result.closest_point);
      auto to_goal = getNormVec(goal_angle);
      auto to_ball = (world_model->ball.pos - result.closest_point).normalized();
      double intermediate_angle = getAngle(2 * to_goal + to_ball);
      command->setTargetTheta(intermediate_angle);
      command->liftUpDribbler();
      command->kickStraight(getParameter<double>("kicker_power"));

      // キッカーの中心のためのオフセット
      command->setTargetPosition(
        result.closest_point - (2 * to_goal + to_ball).normalized() * 0.13);
    }
  } else {
    visualizer->addPoint(
      robot->pose.pos.x(), robot->pose.pos.y(), 0, "red", 1., "ベストポジションへ移動");
    Point best_position = robot->pose.pos;
    double best_score = 0.0;
    for (const auto & dpps_point : dpps_points) {
      double score = getPointScore(dpps_point, world_model->ball.pos, world_model);
      visualizer->addPoint(
        dpps_point.x(), dpps_point.y(), std::clamp(static_cast<int>(score * 100), 0, 20), "blue",
        1.);

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
  visualizer->addLine(
    target_pos, target_pos + to_goal * 3.0, 2, "yellow", 1.0, "Supporterシュートライン");
  command->setTargetTheta(getAngle(to_goal + to_ball));
  command->liftUpDribbler();
  command->kickStraight(getParameter<double>("kicker_power"));

  return Status::RUNNING;
}

std::vector<std::pair<double, Point>> Receiver::getPositionsWithScore(
  Segment ball_line, Point next_target, const WorldModelWrapper::SharedPtr & world_model)
{
  auto points = getPoints(ball_line, 0.05);
  std::vector<std::pair<double, Point>> position_with_score;
  for (auto point : points) {
    double score = getPointScore(point, next_target, world_model);
    position_with_score.push_back(std::make_pair(score, point));
  }
  return position_with_score;
}

std::vector<Point> Receiver::getPoints(Segment ball_line, double interval)
{
  std::vector<Point> points;
  float ball_line_len = (ball_line.first - ball_line.second).norm();
  auto norm_vec = (ball_line.second - ball_line.first).normalized();
  for (double d = 0.0; d <= ball_line_len; d += interval) {
    points.emplace_back(ball_line.first + d * norm_vec);
  }
  return points;
}

std::vector<Point> Receiver::getPoints(Point center, float unit, int unit_num)
{
  std::vector<Point> points;
  for (float x = center.x() - unit * (unit_num / 2.f); x <= center.x() + unit * (unit_num / 2.f);
       x += unit) {
    for (float y = center.y() - unit * (unit_num / 2.f); y <= center.y() + unit * (unit_num / 2.f);
         y += unit) {
      points.emplace_back(Point(x, y));
    }
  }
  return points;
}

std::vector<Point> Receiver::getDPPSPoints(
  Point center, double r_resolution, int theta_div_num,
  const WorldModelWrapper::SharedPtr & world_model)
{
  std::vector<Point> points;
  for (int theta_index = 0; theta_index < theta_div_num; theta_index++) {
    double theta = 2.0 * M_PI * theta_index / theta_div_num;
    for (double r = r_resolution; r <= 10.0; r += r_resolution) {
      points.emplace_back(Point(center.x() + r * cos(theta), center.y() + r * sin(theta)));
    }
  }
  points.erase(
    std::remove_if(
      points.begin(), points.end(),
      [&](const auto & point) {
        return (not world_model->point_checker.isFieldInside(point)) or
               world_model->point_checker.isPenaltyArea(point);
      }),
    points.end());

  return points;
}

double Receiver::getPointScore(
  Point p, Point next_target, const WorldModelWrapper::SharedPtr & world_model)
{
  Segment line{world_model->ball.pos, p};
  auto closest_result = [&]() -> ClosestPoint {
    ClosestPoint closest_result;
    closest_result.distance = std::numeric_limits<double>::max();
    for (const auto & robot : world_model->theirs.getAvailableRobots()) {
      auto result = getClosestPointAndDistance(robot->pose.pos, line);
      if (result.distance < closest_result.distance) {
        closest_result = result;
      }
    }
    return closest_result;
  }();

  auto [angle, width] = world_model->getLargestGoalAngleRangeFromPoint(p);
  // ゴールが見える角度が大きいほどよい
  double score = width;

  // 敵が動いてボールをブロック出来るかどうか
  double enemy_closest_to_ball_dist = (closest_result.closest_point - world_model->ball.pos).norm();
  double ratio = closest_result.distance / enemy_closest_to_ball_dist;
  // ratioが大きいほどよい / 0.1以下は厳しい
  if (ratio < 0.1) {
    score = 0.;
  } else {
    score *= std::clamp(ratio * 5, 0.0, 1.0);
  }

  if (
    std::abs(world_model->ball.pos.x() - world_model->goal.x()) > std::abs(world_model->goal.x())) {
    // 反射角　小さいほどよい（敵ゴールに近い場合のみ）
    auto reflect_angle = std::abs(getAngleDiff(angle, getAngle(world_model->ball.pos - p)));
    score *= (1.0 - std::min(reflect_angle * 0.5, 1.0));
  }
  // 距離 大きいほどよい
  const double dist = world_model->getDistanceFromBall(p);
  score = score * std::max(1.0 - dist / 10.0, 0.0);

  // シュートラインに近すぎる場所は避ける
  Segment shoot_line{world_model->getOurGoalCenter(), world_model->ball.pos};
  const auto dist_to_shoot_line = bg::distance(p, shoot_line);
  if (dist_to_shoot_line < 0.5) {
    score = 0.0;
  }
  return score;
}
}  // namespace crane::skills
