// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__PLACEMENT_TARGET_NEAR_BY_POSITIONER_SKILL_SESSION_HPP_
#define CRANE_SESSIONS__PLACEMENT_TARGET_NEAR_BY_POSITIONER_SKILL_SESSION_HPP_

#include <crane_sessions/ball_near_by_positioner_skill_session.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "visibility_control.h"

namespace crane
{
class PlacementTargetNearByPositionerSkillSession : public BallNearByPositionerSkillSession
{
public:
  COMPOSITION_PUBLIC explicit PlacementTargetNearByPositionerSkillSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  : BallNearByPositionerSkillSession(world_model, node, "placement_target_nearby_positioner_skill")
  {
  }

protected:
  bool shouldExcludeGoalie() const override { return true; }

  std::string getPositioningPolicy() const override { return "goal"; }

  void setupBeforeRun(const std::shared_ptr<skills::BallNearByPositioner> & skill) override
  {
    auto target = world_model->getBallPlacementTarget().value_or(world_model->ball().pos);
    skill->setParameter("alternative_target_mode", true);
    skill->setParameter("alternative_target", target);
  }
};
}  // namespace crane
#endif  // CRANE_SESSIONS__PLACEMENT_TARGET_NEAR_BY_POSITIONER_SKILL_SESSION_HPP_
