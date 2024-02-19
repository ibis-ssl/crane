// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__KICKOFF_SUPPORT_HPP_
#define CRANE_ROBOT_SKILLS__KICKOFF_SUPPORT_HPP_

#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>

namespace crane::skills
{
class KickoffSupport : public SkillBase<>
{
public:
  explicit KickoffSupport(uint8_t id, const std::shared_ptr<WorldModelWrapper> & world_model)
  : SkillBase<>("KickoffSupport", id, world_model, DefaultStates::DEFAULT)
  {
    setParameter("target_x", 0.0f);
    setParameter("target_y", 1.0f);
    addStateFunction(
      DefaultStates::DEFAULT,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot, RobotCommandWrapper & command,
        ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
        Point target(getParameter<double>("target_x"), getParameter<double>("target_y"));
        command.setTargetPosition(target);
        command.lookAtBallFrom(target);
        return Status::RUNNING;
      });
  }

  void print(std::ostream & os) const override { os << "[KickoffSupport]"; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__KICKOFF_SUPPORT_HPP_
