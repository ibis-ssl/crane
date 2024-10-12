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
  explicit GoalKick(RobotCommandWrapperBase::SharedPtr & base);

  Status update(const ConsaiVisualizerWrapper::SharedPtr & visualizer) override;

  void print(std::ostream & os) const override { os << "[GoalKick] "; }

  Kick kick_skill;

  static double getBestAngleToShootFromPoint(
    double minimum_angle_accuracy, const Point from_point,
    const WorldModelWrapper::SharedPtr & world_model);
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__GOAL_KICK_HPP_
