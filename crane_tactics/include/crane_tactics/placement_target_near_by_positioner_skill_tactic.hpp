// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__PLACEMENT_TARGET_NEAR_BY_POSITIONER_SKILL_TACTIC_HPP_
#define CRANE_TACTICS__PLACEMENT_TARGET_NEAR_BY_POSITIONER_SKILL_TACTIC_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/ball_nearby_positioner.hpp>
#include <crane_tactics/tactic_base.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class PlacementTargetNearByPositionerSkillTactic : public TacticBase
{
public:
  std::vector<std::shared_ptr<skills::BallNearByPositioner>> skills;

  COMPOSITION_PUBLIC explicit PlacementTargetNearByPositionerSkillTactic(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : TacticBase("placement_target_nearby_positioner_skill", world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> override;

protected:
  void onRobotsChanged() override { skills.clear(); }
};
}  // namespace crane
#endif  // CRANE_TACTICS__PLACEMENT_TARGET_NEAR_BY_POSITIONER_SKILL_TACTIC_HPP_
