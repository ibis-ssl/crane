// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__KICKOFF_SUPPORT_HPP_
#define CRANE_ROBOT_SKILLS__KICKOFF_SUPPORT_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
class KickoffSupport : public SkillBase<RobotCommandWrapperPosition>
{
public:
  explicit KickoffSupport(RobotCommandWrapperBase::SharedPtr & base)
  : SkillBase("KickoffSupport", base)
  {
    setParameter("target_x", 0.0f);
    setParameter("target_y", 0.5f);
  }

  Status update(const ConsaiVisualizerWrapper::SharedPtr & visualizer) override
  {
    Point target(getParameter<double>("target_x"), getParameter<double>("target_y"));
    command.setDribblerTargetPosition(target).lookAtBallFrom(target);
    return Status::RUNNING;
  }

  void print(std::ostream & os) const override { os << "[KickoffSupport]"; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__KICKOFF_SUPPORT_HPP_
