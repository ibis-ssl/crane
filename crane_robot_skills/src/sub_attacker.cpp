// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_geometry/ddps.hpp>
#include <crane_msg_wrappers/sub_attacker_evaluation.hpp>
#include <crane_robot_skills/sub_attacker.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>

namespace crane::skills
{
void SubAttacker::initialize()
{
  setParameter("ball_vel_threshold", 0.2);
  setParameter("kicker_power", 0.8);
}

Status SubAttacker::update()
{
  auto points = getDPPSPoints(world_model()->ball().pos, 0.25, 10., 64);
  auto dpps_points = points | ranges::views::filter([&](const Point & p) {
                       return world_model()->point_checker.isFieldInside(p) &&
                              not world_model()->point_checker.isPenaltyArea(p);
                     }) |
                     ranges::to<std::vector>();
  // モード判断
  //  こちらへ向かう速度成分
  float ball_vel =
    world_model()->ball().vel.dot((robot()->pose.pos - world_model()->ball().pos).normalized());
  if (
    ball_vel > getParameter<double>("ball_vel_threshold") &&
    ball_vel / world_model()->ball().vel.norm() > 0.7) {
    Segment ball_line(
      world_model()->ball().pos,
      (world_model()->ball().pos + world_model()->ball().vel.normalized() *
                                     (world_model()->ball().pos - robot()->pose.pos).norm()));

    // 後ろからきたボールは一旦避ける
    Segment short_ball_line = world_model()->ball().getTrajectorySegmentByTime(3.0);
    auto result = getClosestPointAndDistance(robot()->pose.pos, short_ball_line);
    // ボールが敵ゴールに向かっているか
    double dot_dir = (world_model()->getAttackGoalCenter() - world_model()->ball().pos)
                       .dot(world_model()->ball().vel);
    // ボールがロボットを追い越そうとしているか
    double dot_inter = (result.closest_point - short_ball_line.first)
                         .dot(result.closest_point - short_ball_line.second);

    if (result.distance < 0.3 && dot_dir > 0. && dot_inter < 0.) {
      // ボールラインから一旦遠ざかる
      command->setTargetPosition(
        result.closest_point + (robot()->pose.pos - result.closest_point).normalized() * 0.5);
      command->enableBallAvoidance();
      visualizer->drawDebugLabel(robot()->pose.pos, "ボールラインから一旦遠ざかる");
    } else {
      //  ボールの進路上に移動
      visualizer->drawDebugLabel(robot()->pose.pos, "ボールの進路上に移動");
      auto result = getClosestPointAndDistance(robot()->pose.pos, ball_line);

      // ゴールとボールの中間方向を向く
      // TODO(Hansobo): ボールの速さ・キッカーの強さでボールの反射する角度が変わるため、要考慮
      auto [goal_angle, width] =
        world_model()->getLargestGoalAngleRangeFromPoint(result.closest_point);
      auto to_goal = getNormVec(goal_angle);
      auto to_ball = (world_model()->ball().pos - result.closest_point).normalized();
      double intermediate_angle = getAngle(2 * to_goal + to_ball);
      command->setTargetTheta(intermediate_angle);
      command->kickStraight(getParameter<double>("kicker_power"));

      // キッカーの中心のためのオフセット
      command->setTargetPosition(
        result.closest_point - (2 * to_goal + to_ball).normalized() * 0.13);
    }
  } else {
    visualizer->drawDebugLabel(robot()->pose.pos, "ベストポジションへ移動");
    Point best_position = robot()->pose.pos;
    double best_score = 0.0;
    for (const auto & dpps_point : dpps_points) {
      double score = getPointScore(dpps_point, world_model()->ball().pos, world_model());

      if (score > best_score) {
        best_score = score;
        best_position = dpps_point;
      }
    }
    command->setTargetPosition(best_position);
    // ゴールとボールの中間方向を向く
    auto [goal_angle, width] = world_model()->getLargestGoalAngleRangeFromPoint(best_position);
    auto to_goal = getNormVec(goal_angle);
    auto to_ball = (world_model()->ball().pos - best_position).normalized();
    command->setTargetTheta(getAngle(to_goal + to_ball));
    command->kickStraight(getParameter<double>("kicker_power"));
  }

  return Status::RUNNING;
}

std::vector<std::pair<double, Point>> SubAttacker::getPositionsWithScore(
  const Segment & ball_line, const Point & next_target,
  const WorldModelWrapper::SharedPtr & world_model)
{
  std::vector<std::pair<double, Point>> position_with_score;
  for (const auto & point : getPoints(ball_line, 0.05)) {
    double score = getPointScore(point, next_target, world_model);
    position_with_score.push_back(std::make_pair(score, point));
  }
  return position_with_score;
}

double SubAttacker::getPointScore(
  const Point & p, [[maybe_unused]] const Point & next_target,
  const WorldModelWrapper::SharedPtr & world_model)
{
  return crane::evaluateSubAttackerPosition(p, *world_model);
}
}  // namespace crane::skills
