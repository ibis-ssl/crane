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
  setParameter("dot_threshold", 0.95);
  kick_skill.setParameter("kick_power", 0.8);
  kick_skill.setParameter("chip_kick", false);
  kick_skill.setParameter("with_dribble", false);
  kick_skill.setParameter("dot_threshold", getParameter<double>("dot_threshold"));
}

Status GoalKick::update(const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  double best_angle = getBestAngleToShootFromPoint(
    getParameter<double>("キック角度の最低要求精度[deg]") * M_PI / 180., world_model()->ball.pos,
    world_model());

  Point target = world_model()->ball.pos + getNormVec(best_angle) * 0.5;
  visualizer->addLine(world_model()->ball.pos, target, 2, "red");
  kick_skill.setParameter("target", target);
  kick_skill.setParameter("dot_threshold", getParameter<double>("dot_threshold"));
  return kick_skill.run(visualizer);
}

double GoalKick::getBestAngleToShootFromPoint(
  double minimum_angle_accuracy, const Point from_point,
  const WorldModelWrapper::SharedPtr & world_model)
{
  auto [best_angle, goal_angle_width] = world_model->getLargestGoalAngleRangeFromPoint(from_point);
  if (goal_angle_width < 0.) {
    // ゴールが見えない場合はgoal_angle_widthが負になる
    // その場合は相手ゴール中心を狙う
    best_angle = getAngle(world_model->getTheirGoalCenter() - from_point);
  }
  // 隙間のなかで更に良い角度を計算する。
  // キック角度の最低要求精度をオフセットとしてできるだけ端っこを狙う
  if (goal_angle_width < minimum_angle_accuracy * 2.0) {
    double best_angle1, best_angle2;
    best_angle1 = best_angle - goal_angle_width / 2.0 + minimum_angle_accuracy;
    best_angle2 = best_angle + goal_angle_width / 2.0 - minimum_angle_accuracy;
    Point their_goalie_pos = [&]() -> Point {
      const auto & enemy_robots = world_model->theirs.getAvailableRobots();
      if (not enemy_robots.empty()) {
        return world_model
          ->getNearestRobotWithDistanceFromPoint(world_model->getTheirGoalCenter(), enemy_robots)
          .first->pose.pos;
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
