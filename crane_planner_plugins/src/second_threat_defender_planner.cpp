// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_physics/position_assignments.hpp>
#include <crane_planner_plugins/second_threat_defender_planner.hpp>

namespace crane
{
auto SecondThreatDefenderPlanner::calculatePositionCommand(
  [[maybe_unused]] const std::vector<RobotIdentifier> & robots)
  -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>>
{
  if (skill) {
    skill->run();
    return {PlannerBase::Status::RUNNING, {skill->getRobotCommand()}};
  } else {
    return {PlannerBase::Status::RUNNING, {}};
  }
}

auto SecondThreatDefenderPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  if (selectable_robots_num < 1) {
    return {};
  } else {
    constexpr double offset = 0.3;
    auto target = skills::SecondThreatDefender::getDefaultPoint(world_model, offset);
    auto selected = this->getSelectedRobotsByScore(
      1, selectable_robots,
      [&](const std::shared_ptr<RobotInfo> & robot) {
        // ターゲットに一番近いロボット
        return 100. / robot->getDistance(target);
      },
      prev_roles);
    skill = std::make_shared<skills::SecondThreatDefender>(selected.front(), world_model);
    skill->setParameter("offset", offset);
    return selected;
  }
}
}  // namespace crane
