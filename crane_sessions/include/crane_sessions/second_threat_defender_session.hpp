// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__SECOND_THREAT_DEFENDER_SESSION_HPP_
#define CRANE_SESSIONS__SECOND_THREAT_DEFENDER_SESSION_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/second_threat_defender.hpp>
#include <crane_sessions/single_skill_session.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

namespace crane
{
class SecondThreatDefenderSession : public SingleSkillSession<skills::SecondThreatDefender>
{
public:
  COMPOSITION_PUBLIC
  explicit SecondThreatDefenderSession(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : SingleSkillSession("second_threat_defender", world_model)
  {
  }

  bool isHardConstraint() const override { return true; }

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    // セカンド脅威守備位置に近いロボットを優先
    auto wm = world_model;
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      constexpr double offset = 0.3;
      auto target = skills::SecondThreatDefender::getDefaultPoint(wm, offset);
      return robot->getDistance(target);
    };
  }

protected:
  auto createSkill(uint8_t robot_id) -> std::shared_ptr<skills::SecondThreatDefender> override
  {
    return std::make_shared<skills::SecondThreatDefender>(robot_id, world_model);
  }
};
}  // namespace crane
#endif  // CRANE_SESSIONS__SECOND_THREAT_DEFENDER_SESSION_HPP_
