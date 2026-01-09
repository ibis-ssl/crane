// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_physics/position_assignments.hpp>
#include <crane_tactics/second_threat_defender_tactic.hpp>

namespace crane
{
auto SecondThreatDefenderTactic::calculatePositionCommand(
  [[maybe_unused]] const std::vector<RobotIdentifier> & robots)
  -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>>
{
  if (skill) {
    skill->run();
    return {TacticBase::Status::RUNNING, {skill->getRobotCommand()}};
  } else {
    return {TacticBase::Status::RUNNING, {}};
  }
}
}  // namespace crane
