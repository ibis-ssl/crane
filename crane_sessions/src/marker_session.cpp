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
  // ボール停止中＋敵がボール近くにいる＝相手セットプレイ → save_goalモードでゴール前を守る
  // それ以外は守備ライン上に配置するpenalty_areaモードを使用
  std::string mark_mode = "penalty_area";
  if (!world_model->ball().isMoving(0.5)) {
    auto their_robots = world_model->theirs().robotsWhere().available().get();
    bool enemy_near_ball = std::any_of(
      their_robots.begin(), their_robots.end(),
      [&](const auto & r) { return r->getDistance(world_model->ball().pos) < 1.0; });
    if (enemy_near_ball) {
      mark_mode = "save_goal";
    }
  }
  auto result = assignMarkersToEnemies(
    robot_ids, world_model, visualizer, "marker_planner", true, mark_mode, prev_assignments_);
  prev_assignments_ = std::move(result.enemy_to_marker);
  markers = std::move(result.markers);

  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  for (const auto & skill : markers) {
    skill->run();
    robot_commands.emplace_back(skill->getRobotCommand());
  }
  return {SessionBase::Status::RUNNING, robot_commands};
}
}  // namespace crane
