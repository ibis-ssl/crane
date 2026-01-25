// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_physics/position_assignments.hpp>
#include <crane_sessions/second_threat_defender_session.hpp>

namespace crane
{
auto SecondThreatDefenderSession::calculatePositionCommand(
  const std::vector<RobotIdentifier> & robots)
  -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>>
{
  if (robots.empty()) {
    return {SessionBase::Status::RUNNING, {}};
  }
  if (not skill) {
    skill = std::make_shared<skills::SecondThreatDefender>(robots.front().id, world_model);
  }
  skill->run();
  return {SessionBase::Status::RUNNING, {skill->getRobotCommand()}};
}
}  // namespace crane
