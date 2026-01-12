// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__GOALIE_SKILL_TACTIC_HPP_
#define CRANE_TACTICS__GOALIE_SKILL_TACTIC_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/goalie.hpp>
#include <crane_tactics/tactic_base.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class GoalieSkillTactic : public TacticBase
{
public:
  std::shared_ptr<skills::Goalie> skill = nullptr;

  COMPOSITION_PUBLIC explicit GoalieSkillTactic(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : TacticBase("goalie_skill", world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> override;

  bool isHardConstraint() const override { return true; }

protected:
  void onRobotsChanged() override { skill.reset(); }
};
}  // namespace crane
#endif  // CRANE_TACTICS__GOALIE_SKILL_TACTIC_HPP_
