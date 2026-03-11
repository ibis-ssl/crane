// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_sessions/marker_functions.hpp>
#include <crane_sessions/marker_session.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>

namespace crane
{
std::pair<SessionBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
MarkerSession::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
{
  std::vector<uint8_t> robot_ids =
    robots | ranges::views::transform([&](const auto & robot) { return robot.id; }) |
    ranges::to<std::vector>();
  auto lock = std::lock_guard(markers_mutex);
  visualizer->clearBuffer();
  auto result = assignMarkersToEnemies(robot_ids, world_model, visualizer, "marker_planner", true);
  markers = std::move(result.markers);

  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  for (const auto & skill : markers) {
    skill->run();
    robot_commands.emplace_back(skill->getRobotCommand());
  }
  return {SessionBase::Status::RUNNING, robot_commands};
}
}  // namespace crane
