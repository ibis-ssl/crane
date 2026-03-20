// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__SUB_ATTACKER_SKILL_SESSION_HPP_
#define CRANE_SESSIONS__SUB_ATTACKER_SKILL_SESSION_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/sub_attacker.hpp>
#include <crane_sessions/single_skill_session.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

namespace crane
{
class SubAttackerSkillSession : public SingleSkillSession<skills::SubAttacker>
{
public:
  COMPOSITION_PUBLIC explicit SubAttackerSkillSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : SingleSkillSession("sub_attacker_skill", world_model)
  {
  }

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    auto wm = world_model;
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      const auto & ga = wm->getMsg().game_analysis;

      // 有効な推奨位置がない場合は高コスト
      if (!ga.has_sub_attacker_position) {
        return NO_SUITABLE_POSITION_COST;
      }

      // 推奨位置への距離をコストとして返す
      Point recommended_pos{
        ga.recommended_sub_attacker_position.x, ga.recommended_sub_attacker_position.y};
      return robot->getSquareDistance(recommended_pos);
    };
  }

protected:
  auto createSkill(uint8_t robot_id) -> std::shared_ptr<skills::SubAttacker> override
  {
    return std::make_shared<skills::SubAttacker>(robot_id, world_model);
  }
};
}  // namespace crane
#endif  // CRANE_SESSIONS__SUB_ATTACKER_SKILL_SESSION_HPP_
