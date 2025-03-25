// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/goal_kick.hpp>

namespace crane::skills
{

GoalKick::GoalKick(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("GoalKick", base), kick_skill(base)
{
  setParameter("キック角度の最低要求精度[deg]", 1.0);
  kick_skill.setParameter("kick_power", 0.8);
  kick_skill.setParameter("chip_kick", false);
  kick_skill.setParameter("with_dribble", false);
}

Status GoalKick::update()
{
  double best_angle = getBestAngleToShootFromPoint(
    getParameter<double>("キック角度の最低要求精度[deg]") * M_PI / 180., world_model()->ball.pos,
    world_model(), visualizer);

  Point target = world_model()->ball.pos + getNormVec(best_angle) * 0.5;
  {
    Segment segment{
      world_model()->ball.pos, world_model()->ball.pos + getNormVec(best_angle) * 20.0};
    Segment goal_line(
      Point(world_model()->getTheirGoalCenter().x(), world_model()->field_size.y() * 0.5),
      Point(world_model()->getTheirGoalCenter().x(), -world_model()->field_size.y() * 0.5));
    if (auto intersections = getIntersections(segment, goal_line); not intersections.empty()) {
      visualizer->line()
        .start(world_model()->ball.pos)
        .end(intersections.front())
        .stroke("red", 0.5)
        .strokeWidth(20)
        .build();
    }
  }
  kick_skill.setParameter("target", target);
  return kick_skill.run();
}

double GoalKick::getBestAngleToShootFromPoint(
  double minimum_angle_accuracy, const Point from_point,
  const WorldModelWrapper::SharedPtr & world_model,
  const CraneVisualizerBuffer::MessageBuilder::UniquePtr & visualizer)
{
  auto [best_angle, goal_angle_width] = world_model->getLargestGoalAngleRangeFromPoint(from_point);
  auto intersection_positive = [&]() {
    double angle = best_angle + goal_angle_width * 0.5;
    Segment segment{from_point, from_point + getNormVec(angle) * 20.0};
    Segment goal_line(
      Point(world_model->getTheirGoalCenter().x(), world_model->field_size.y() * 0.5),
      Point(world_model->getTheirGoalCenter().x(), -world_model->field_size.y() * 0.5));
    return getIntersections(segment, goal_line);
  }();
  auto intersection_negative = [&]() {
    double angle = best_angle - goal_angle_width * 0.5;
    Segment segment{from_point, from_point + getNormVec(angle) * 20.0};
    Segment goal_line(
      Point(world_model->getTheirGoalCenter().x(), world_model->field_size.y() * 0.5),
      Point(world_model->getTheirGoalCenter().x(), -world_model->field_size.y() * 0.5));
    return getIntersections(segment, goal_line);
  }();

  if (intersection_positive.empty() or intersection_negative.empty()) {
    return best_angle;
  } else {
    {
      SvgPathBuilder path;
      path.definition.moveTo(from_point)
        .lineTo(intersection_positive.front())
        .lineTo(intersection_negative.front())
        .lineTo(from_point);
      path.strokeWidth(10).stroke("green");
      visualizer->add(path.getSvgString());
    }
    if (goal_angle_width < 0.) {
      // ゴールが見えない場合はgoal_angle_widthが負になる
      // その場合は相手ゴール中心を狙う
      return getAngle(world_model->getTheirGoalCenter() - from_point);
    }
    // 隙間のなかで更に良い角度を計算する。
    // キック角度の最低要求精度をオフセットとしてできるだけ端っこを狙う
    if (goal_angle_width > minimum_angle_accuracy * 2.0) {
      auto theirs = world_model->theirs.getAvailableRobots();
      auto ret_pos = world_model->getNearestRobotWithDistanceFromSegment(
        Segment{from_point, intersection_positive.front()}, theirs);
      auto ret_neg = world_model->getNearestRobotWithDistanceFromSegment(
        Segment{from_point, intersection_negative.front()}, theirs);
      if (not ret_pos.has_value() or not ret_neg.has_value()) {
        return best_angle;
      } else {
        if (ret_pos->distance < 0.1 && ret_neg->distance < 0.1) {
          // 両方ロボットのときはど真ん中
          return best_angle;
        } else if (ret_pos->distance < 0.1) {
          // ret_pos側だけにロボットがいるときはneg側から
          return best_angle - goal_angle_width / 2.0 + minimum_angle_accuracy;
        } else if (ret_neg->distance < 0.1) {
          // ret_neg側だけにロボットがいるときはpos側から
          return best_angle + goal_angle_width / 2.0 - minimum_angle_accuracy;
        } else {
          // どちらもロボットではない場合は真ん中
          return best_angle;
        }
      }
    } else {
      return best_angle;
    }
  }
}
}  // namespace crane::skills
