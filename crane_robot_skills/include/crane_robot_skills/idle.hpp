// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__IDLE_HPP_
#define CRANE_ROBOT_SKILLS__IDLE_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
class Idle : public SkillBase
{
public:
  explicit Idle(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
  : SkillBase("Idle", id, wm)
  {
  }

  Status update(const ConsaiVisualizerWrapper::SharedPtr & visualizer) override
  {
    auto cmd = std::make_shared<RobotCommandWrapperPosition>(command);
    command->stopHere();
    return Status::RUNNING;
  }

  void print(std::ostream & os) const override { os << "[Idle]"; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__IDLE_HPP_
