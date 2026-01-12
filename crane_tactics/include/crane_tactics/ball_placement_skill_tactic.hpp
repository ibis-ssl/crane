// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__BALL_PLACEMENT_SKILL_TACTIC_HPP_
#define CRANE_TACTICS__BALL_PLACEMENT_SKILL_TACTIC_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/single_ball_placement.hpp>
#include <crane_tactics/tactic_base.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class BallPlacementSkillTactic : public TacticBase
{
public:
  std::shared_ptr<skills::SingleBallPlacement> skill = nullptr;

  COMPOSITION_PUBLIC explicit BallPlacementSkillTactic(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : TacticBase("ball_placement_skill", world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> override;

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    auto wm = world_model;
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      if (robot->id == wm->getOurGoalieId()) {
        return 10000.0;  // ゴールキーパーは除外
      }
      return robot->getSquareDistance(wm->ball().pos);
    };
  }

protected:
  void onRobotsChanged() override { skill.reset(); }
};
}  // namespace crane
#endif  // CRANE_TACTICS__BALL_PLACEMENT_SKILL_TACTIC_HPP_
