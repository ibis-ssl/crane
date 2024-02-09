// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__IDLE_HPP_
#define CRANE_ROBOT_SKILLS__IDLE_HPP_

#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>

namespace crane
{
class Idle : public SkillBase<>
{
public:
  explicit Idle(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
  : SkillBase<>("Idle", id, world_model, DefaultStates::DEFAULT)
  {
    setParameter("stop_by_position", true);
    addStateFunction(
      DefaultStates::DEFAULT,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot, crane::RobotCommandWrapper & command,
        ConsaiVisualizerWrapper & visualizer) -> SkillBase::Status {
        // TODO: モーターをOFFにするようにしたほうがバッテリーに優しいかも
        if (getParameter<bool>("stop_by_position")) {
          command.stopHere();
        } else {
          command.setVelocity(0., 0.);
        }
        return SkillBase::Status::RUNNING;
      });
  }

  void print(std::ostream & os) const override
  {
    os << "[Idle] stop_by_position: " << getParameter<bool>("stop_by_position") ? "true" : "false";
  }
};
}  // namespace crane
#endif  // CRANE_ROBOT_SKILLS__IDLE_HPP_
