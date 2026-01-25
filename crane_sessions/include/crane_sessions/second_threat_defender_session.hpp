// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__SECOND_THREAT_DEFENDER_TACTIC_HPP_
#define CRANE_SESSIONS__SECOND_THREAT_DEFENDER_TACTIC_HPP_

#include <algorithm>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/second_threat_defender.hpp>
#include <crane_sessions/session_base.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class SecondThreatDefenderSession : public SessionBase
{
public:
  COMPOSITION_PUBLIC
  explicit SecondThreatDefenderSession(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : SessionBase("second_threat_defender", world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> override;

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    // セカンド脅威守備位置に近いロボットを優先
    auto wm = world_model;  // shared_ptrをコピー
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      constexpr double offset = 0.3;
      auto target = skills::SecondThreatDefender::getDefaultPoint(wm, offset);
      return robot->getDistance(target);
    };
  }

protected:
  void onRobotsChanged() override { skill.reset(); }

private:
  std::shared_ptr<skills::SecondThreatDefender> skill;
};
}  // namespace crane
#endif  // CRANE_SESSIONS__SECOND_THREAT_DEFENDER_TACTIC_HPP_
