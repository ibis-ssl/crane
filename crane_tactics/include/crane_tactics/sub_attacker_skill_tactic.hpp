// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__SUB_ATTACKER_SKILL_TACTIC_HPP_
#define CRANE_TACTICS__SUB_ATTACKER_SKILL_TACTIC_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/sub_attacker.hpp>
#include <crane_tactics/tactic_base.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class SubAttackerSkillTactic : public TacticBase
{
public:
  std::shared_ptr<skills::SubAttacker> skill = nullptr;

  COMPOSITION_PUBLIC explicit SubAttackerSkillTactic(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : TacticBase("sub_attacker_skill", world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> override;

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    auto wm = world_model;
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      const auto & ga = wm->getMsg().game_analysis;

      // 有効な推奨位置がない場合は高コスト
      if (!ga.has_sub_attacker_position) {
        return 10000.0;
      }

      // 推奨位置への距離をコストとして返す
      Point recommended_pos{
        ga.recommended_sub_attacker_position.x, ga.recommended_sub_attacker_position.y};
      return robot->getSquareDistance(recommended_pos);
    };
  }

protected:
  void onRobotsChanged() override { skill.reset(); }
};
}  // namespace crane
#endif  // CRANE_TACTICS__SUB_ATTACKER_SKILL_TACTIC_HPP_
