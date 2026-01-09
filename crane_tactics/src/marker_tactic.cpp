// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_physics/position_assignments.hpp>
#include <crane_tactics/marker_functions.hpp>
#include <crane_tactics/marker_tactic.hpp>
#include <range/v3/algorithm/find_if.hpp>
#include <range/v3/algorithm/min.hpp>
#include <range/v3/functional/comparisons.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>

namespace crane
{
std::pair<TacticBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
MarkerTactic::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
{
  std::vector<uint8_t> robot_ids =
    robots | ranges::views::transform([&](const auto & robot) { return robot.id; }) |
    ranges::to<std::vector>();
  auto lock = std::lock_guard(markers_mutex);
  assignMarkingTarget(robot_ids.size(), robot_ids);
  std::vector<crane_msgs::msg::PositionCommand> robot_commands;
  for (const auto & skill : markers) {
    skill->run();
    robot_commands.emplace_back(skill->getRobotCommand());
  }
  return {TacticBase::Status::RUNNING, robot_commands};
}

auto MarkerTactic::assignMarkingTarget(
  uint8_t selectable_robots_num, const std::vector<uint8_t> selectable_robots)
  -> std::vector<uint8_t>
{
  visualizer->clearBuffer();
  auto dander_enemies = getDangerEnemies(world_model);

  for (const auto & [robot, score] : dander_enemies) {
    visualizer->drawDebugLabel(
      robot->pose.pos + Point(0., 0.2), "MarkerScore: " + std::to_string(score));
  }

  if (dander_enemies.size() > selectable_robots_num) {
    dander_enemies.resize(selectable_robots_num);
  }

  RobotList remaining_selectable_robots =
    selectable_robots |
    ranges::views::transform([&](const auto & id) { return world_model->getOurRobot(id); }) |
    ranges::to<std::vector>();

  std::vector<uint8_t> selected_robots;

  markers.clear();

  for (const auto & [enemy_robot, score] : dander_enemies) {
    // マークする敵ロボットに一番近い味方ロボットを選択
    if (not remaining_selectable_robots.empty()) {
      auto best_marking_robot = ranges::min(
        remaining_selectable_robots, ranges::less{},
        [&](const auto & robot) { return (robot->pose.pos - enemy_robot->pose.pos).norm(); });

      // marking_target_map[best_marking_robot->id] = enemy_robot->id;
      selected_robots.push_back(best_marking_robot->id);
      remaining_selectable_robots.erase(
        ranges::find_if(remaining_selectable_robots, [best_marking_robot](const auto & robot) {
          return robot->id == best_marking_robot->id;
        }));

      // skillを作って設定
      markers.emplace_back(
        std::make_shared<skills::Marker>(
          "marker_planner", static_cast<uint8_t>(best_marking_robot->id), world_model));

      markers.back()->setParameter("marking_robot_id", enemy_robot->id);
      markers.back()->setParameter("mark_mode", std::string("intercept_pass"));
      markers.back()->setParameter("mark_distance", 0.5);
      // if ((world_model->ball().pos - enemy_robot->pose.pos).norm() > 3.0) {
      //   markers.back()->setParameter("mark_mode", std::string("intercept_pass"));
      //   markers.back()->setParameter("mark_distance", 0.5);
      // } else {
      //   markers.back()->setParameter("mark_mode", std::string("save_goal"));
      //   double distance = (world_model->goal() - enemy_robot->pose.pos).norm() * 0.1 + 0.2;
      //   markers.back()->setParameter("mark_distance", distance);
      // }

      visualizer->drawCircle(enemy_robot->pose.pos, 0.3, "black", 10);
      visualizer->drawLine(
        best_marking_robot->pose.pos,
        enemy_robot->pose.pos +
          (enemy_robot->pose.pos - best_marking_robot->pose.pos).normalized() * 0.3,
        "black", 20);
    }
  }
  return selected_robots;
}
}  // namespace crane
