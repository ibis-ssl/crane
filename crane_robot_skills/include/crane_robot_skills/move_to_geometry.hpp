// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__MOVE_TO_GEOMETRY_HPP
#define CRANE_PLANNER_PLUGINS__MOVE_TO_GEOMETRY_HPP

#include <crane_robot_skills/skill_base.hpp>

namespace crane
{
class MoveToGeometry : public SkillBase
{
public:
  explicit MoveToGeometry(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
  : SkillBase("move_to_geometry", world_model, id, DefaultStates::DEFAULT)
  {
    addStateFunction(
      DefaultStates::DEFAULT,
      [](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot,
        crane::RobotCommandWrapper & command) -> SkillBase::Status {
        if ((robot->pose.pos - getTargetPoint()).norm() < 0.1) {
          return SkillBase::Status::SUCCESS;
        } else {
          command.setTargetPosition(getTargetPoint());
          return SkillBase::Status::RUNNING;
        }
      });
  }

  Point getTargetPoint() { return Point(); }
};
}  // namespace crane
#endif  //CRANE_PLANNER_PLUGINS__MOVE_TO_GEOMETRY_HPP
