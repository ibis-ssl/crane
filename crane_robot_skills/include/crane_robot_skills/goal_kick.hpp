// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__GOAL_KICK_HPP_
#define CRANE_ROBOT_SKILLS__GOAL_KICK_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/kick.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
class GoalKick : public SkillBase<RobotCommandWrapperPosition>
{
public:
  explicit GoalKick(RobotCommandWrapperBase::SharedPtr & base)
  : SkillBase("GoalKick", base), kick_skill(base)
  {
    setParameter("キック角度の最低要求精度[deg]", 1.0);
    kick_skill.setParameter("kick_power", 0.8);
    kick_skill.setParameter("chip_kick", false);
    kick_skill.setParameter("with_dribble", false);
    kick_skill.setParameter("dot_threshold", 0.95);
  }

  Status update(const ConsaiVisualizerWrapper::SharedPtr & visualizer) override
  {
    double best_angle = getBestAngleToShootFromBall(
      getParameter<double>("キック角度の最低要求精度[deg]") * M_PI / 180., world_model());

    Point target = world_model()->ball.pos + getNormVec(best_angle) * 0.5;
    kick_skill.setParameter("target", target);
    return kick_skill.run(visualizer);
  }

  void print(std::ostream & os) const override { os << "[GoalKick] "; }

  Kick kick_skill;

  static double getBestAngleToShootFromBall(
    double minimum_angle_accuracy, const WorldModelWrapper::SharedPtr & world_model)
  {
    auto [best_angle, goal_angle_width] =
      world_model->getLargestGoalAngleRangeFromPoint(world_model->ball.pos);
    // 隙間のなかで更に良い角度を計算する。
    // キック角度の最低要求精度をオフセットとしてできるだけ端っこを狙う
    if (goal_angle_width < minimum_angle_accuracy * 2.0) {
      double best_angle1, best_angle2;
      best_angle1 = best_angle - goal_angle_width / 2.0 + minimum_angle_accuracy;
      best_angle2 = best_angle + goal_angle_width / 2.0 - minimum_angle_accuracy;
      Point their_goalie_pos =
        world_model
          ->getNearestRobotsWithDistanceFromPoint(
            world_model->getTheirGoalCenter(), world_model->theirs.getAvailableRobots())
          .first->pose.pos;
      double their_goalie_angle = getAngle(their_goalie_pos - world_model->ball.pos);
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
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__GOAL_KICK_HPP_
