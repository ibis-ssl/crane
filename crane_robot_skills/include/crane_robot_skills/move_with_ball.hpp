// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__MOVE_WITH_BALL_HPP_
#define CRANE_ROBOT_SKILLS__MOVE_WITH_BALL_HPP_

#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>

namespace crane
{
class MoveWithBall : public SkillBase<>
{
public:
  explicit MoveWithBall(
    uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
  : SkillBase<>("move_with_ball", id, world_model, DefaultStates::DEFAULT)
  {
    addStateFunction(
      DefaultStates::DEFAULT,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot,
        crane::RobotCommandWrapper & command) -> SkillBase::Status {
        return SkillBase::Status::RUNNING;
      });
  }
};
}  // namespace crane
#endif  // CRANE_ROBOT_SKILLS__MOVE_WITH_BALL_HPP_
