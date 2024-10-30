// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__BALL_NEARBY_POSITIONER_HPP_
#define CRANE_ROBOT_SKILLS__BALL_NEARBY_POSITIONER_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
class BallNearByPositioner : public SkillBase<RobotCommandWrapperPosition>
{
public:
  explicit BallNearByPositioner(RobotCommandWrapperBase::SharedPtr & base);

  Status update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) override;

  void print(std::ostream & os) const override { os << "[BallNearByPositioner]"; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__BALL_NEARBY_POSITIONER_HPP_
