// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__FREEKICK_SAVER_HPP_
#define CRANE_ROBOT_SKILLS__FREEKICK_SAVER_HPP_

#include <algorithm>
#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace crane::skills
{
class FreeKickSaver : public SkillBase
{
public:
  explicit FreeKickSaver(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
  : SkillBase("FreeKickSaver", id, wm)
  {
  }

  Status update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) override
  {
    auto cmd = std::make_shared<RobotCommandWrapperPosition>(command);
    auto & ball = world_model->ball.pos;
    auto [their_nearest, distance] = world_model->getNearestRobotsWithDistanceFromPoint(
      ball, world_model->theirs.getAvailableRobots());
    Point target = ball + (ball - their_nearest->pose.pos).normalized() * 0.7;
    cmd->setTargetPosition(target);
    command->lookAtBallFrom(target);
    return Status::RUNNING;
  }

  void print(std::ostream & os) const override { os << "[FreeKickSaver]"; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__FREEKICK_SAVER_HPP_
