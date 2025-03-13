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
    world_model());

  Point target = world_model()->ball.pos + getNormVec(best_angle) * 0.5;
  {
    SvgLineBuilder line_builder;
    Segment segment{world_model()->ball.pos, getNormVec(best_angle) * 20.0};
    Segment goal_line(
      Point(world_model()->getTheirGoalCenter().x(), world_model()->field_size.y() * 0.5),
      Point(world_model()->getTheirGoalCenter().x(), -world_model()->field_size.y() * 0.5));
    if (auto intersections = getIntersections(segment, goal_line); not intersections.empty()) {
      line_builder.start(world_model()->ball.pos)
        .end(intersections.front())
        .stroke("red")
        .strokeWidth(30);
      visualizer->add(line_builder.getSvgString());
    }
  }
  kick_skill.setParameter("target", target);
  return kick_skill.run();
}

double GoalKick::getBestAngleToShootFromPoint(
  double minimum_angle_accuracy, const Point from_point,
  const WorldModelWrapper::SharedPtr & world_model)
{
  auto [best_angle, goal_angle_width] = world_model->getLargestGoalAngleRangeFromPoint(from_point);
  {
    SvgPathBuilder path;
    bool is_valid = true;
    path.definition.moveTo(world_model->ball.pos)
      .lineTo([&]() {
        double angle = best_angle + goal_angle_width * 0.5;
        Segment segment{world_model->ball.pos, getNormVec(angle) * 20.0};
        Segment goal_line(
          Point(world_model->getTheirGoalCenter().x(), world_model->field_size.y() * 0.5),
          Point(world_model->getTheirGoalCenter().x(), -world_model->field_size.y() * 0.5));
        auto intersection = getIntersections(segment, goal_line);
        if (not intersection.empty()) {
          return intersection.front();
        } else {
          is_valid = false;
          return Point();
        }
      }())
      .lineTo([&]() {
        double angle = best_angle - goal_angle_width * 0.5;
        Segment segment{world_model->ball.pos, getNormVec(angle) * 20.0};
        Segment goal_line(
          Point(world_model->getTheirGoalCenter().x(), world_model->field_size.y() * 0.5),
          Point(world_model->getTheirGoalCenter().x(), -world_model->field_size.y() * 0.5));
        auto intersection = getIntersections(segment, goal_line);
        if (not intersection.empty()) {
          return intersection.front();
        } else {
          is_valid = false;
          return Point();
        }
      }())
      .lineTo(world_model->ball.pos);

    if (is_valid) {
      path.strokeWidth(20).stroke("red");
      visualizer->add(path.getSvgString());
    }
  }
  if (goal_angle_width < 0.) {
    // ゴールが見えない場合はgoal_angle_widthが負になる
    // その場合は相手ゴール中心を狙う
    best_angle = getAngle(world_model->getTheirGoalCenter() - from_point);
  }
  // 隙間のなかで更に良い角度を計算する。
  // キック角度の最低要求精度をオフセットとしてできるだけ端っこを狙う
  if (goal_angle_width < minimum_angle_accuracy * 2.0) {
    double best_angle1 = best_angle - goal_angle_width / 2.0 + minimum_angle_accuracy;
    double best_angle2 = best_angle + goal_angle_width / 2.0 - minimum_angle_accuracy;
    Point their_goalie_pos = [&]() -> Point {
      if (auto nearest = world_model->getNearestRobotWithDistanceFromPoint(
            world_model->getTheirGoalCenter(), world_model->theirs.getAvailableRobots());
          nearest.has_value()) {
        return nearest->robot->pose.pos;
      } else {
        return world_model->getTheirGoalCenter();
      }
    }();
    double their_goalie_angle = getAngle(their_goalie_pos - from_point);
    // 敵ゴールキーパーから角度差が大きい方を選択
    if (
      std::abs(getAngleDiff(their_goalie_angle, best_angle1)) <
      std::abs(getAngleDiff(their_goalie_angle, best_angle2))) {
      best_angle = best_angle2;
    } else {
      best_angle = best_angle1;
    }
  }
  return best_angle;
}
}  // namespace crane::skills
