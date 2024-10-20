// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__GO_OVER_BALL_HPP_
#define CRANE_ROBOT_SKILLS__GO_OVER_BALL_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <utility>
#include <vector>

namespace crane::skills
{
class GoOverBall : public SkillBase<RobotCommandWrapperPosition>
{
public:
  explicit GoOverBall(RobotCommandWrapperBase::SharedPtr & base);

  Status update(const ConsaiVisualizerWrapper::SharedPtr & visualizer) override;

  void print(std::ostream & out) const override;

private:
  bool & has_started;

  bool & has_passed_intermediate_target;
  //  bool has_
  Point & final_target_pos;

  std::pair<Point &, Point &> intermediate_target_pos;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__GO_OVER_BALL_HPP_
