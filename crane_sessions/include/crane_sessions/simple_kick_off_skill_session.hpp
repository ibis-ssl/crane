// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__SIMPLE_KICK_OFF_SKILL_SESSION_HPP_
#define CRANE_SESSIONS__SIMPLE_KICK_OFF_SKILL_SESSION_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/simple_kickoff.hpp>
#include <crane_sessions/single_skill_session.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

namespace crane
{
class SimpleKickOffSkillSession : public SingleSkillSession<skills::SimpleKickOff>
{
public:
  COMPOSITION_PUBLIC explicit SimpleKickOffSkillSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : SingleSkillSession("simple_kickoff", world_model)
  {
  }

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    auto wm = world_model;
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      return robot->getSquareDistance(wm->ball().pos);
    };
  }

protected:
  auto createSkill(uint8_t robot_id) -> std::shared_ptr<skills::SimpleKickOff> override
  {
    return std::make_shared<skills::SimpleKickOff>(
      "simple_kick_off_skill_planner", robot_id, world_model);
  }
};
}  // namespace crane
#endif  // CRANE_SESSIONS__SIMPLE_KICK_OFF_SKILL_SESSION_HPP_
