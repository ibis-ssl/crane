// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__FREE_KICKER_SKILL_SESSION_HPP_
#define CRANE_SESSIONS__FREE_KICKER_SKILL_SESSION_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/free_kicker.hpp>
#include <crane_sessions/single_skill_session.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

namespace crane
{
class FreeKickerSkillSession : public SingleSkillSession<skills::FreeKicker>
{
public:
  COMPOSITION_PUBLIC explicit FreeKickerSkillSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : SingleSkillSession("free_kicker_skill", world_model)
  {
  }

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    auto wm = world_model;
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      if (robot->id == wm->getOurGoalieId()) {
        return GOALIE_EXCLUSION_COST;
      }
      // ボール最寄りのロボットをフリーキッカーとして割り当てる
      return robot->getSquareDistance(wm->ball().pos);
    };
  }

protected:
  auto createSkill(uint8_t robot_id) -> std::shared_ptr<skills::FreeKicker> override
  {
    return std::make_shared<skills::FreeKicker>(robot_id, world_model);
  }
};
}  // namespace crane
#endif  // CRANE_SESSIONS__FREE_KICKER_SKILL_SESSION_HPP_
