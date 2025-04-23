// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_basics/position_assignments.hpp>
#include <crane_planner_plugins/marker_planner.hpp>
#include <range/v3/all.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
MarkerPlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots, PlannerContext &)
{
  std::vector<uint8_t> robot_ids =
    robots | ranges::views::transform([&](const auto & robot) { return robot.id; }) |
    ranges::to<std::vector>();
  auto lock = std::lock_guard(markers_mutex);
  assignMarkingTarget(robot_ids.size(), robot_ids);
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  for (const auto & skill : markers) {
    skill->run();
    robot_commands.emplace_back(skill->getRobotCommand());
  }
  return {PlannerBase::Status::RUNNING, robot_commands};
}

auto MarkerPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> &, PlannerContext &) -> std::vector<uint8_t>
{
  auto lock = std::lock_guard(markers_mutex);
  return assignMarkingTarget(selectable_robots_num, selectable_robots);
}

auto MarkerPlanner::getDangerEnemies() -> std::vector<std::pair<std::shared_ptr<RobotInfo>, double>>
{
  RobotList defense_robots;
  defense_robots.emplace_back(world_model->getOurRobot((world_model->getOurGoalieId())));

  const auto their_robots = world_model->theirs.getAvailableRobots();
  auto robots_and_scores =
    their_robots | ranges::views::filter([&](const auto & robot) {
      if (not world_model->point_checker.isInOurHalf(robot->pose.pos)) {
        // 相手コートにいる敵ロボットはマークしない
        return false;
      } else if (robot->getDistance(world_model->ball.pos) < 1.0) {
        // ボールに近い敵ロボットはマークしない
        return false;
      } else {
        return true;
      }
    }) |
    ranges::views::transform([&](const auto & robot) {
      auto [_, angle_width] =
        world_model->getLargestOurGoalAngleRangeFromPoint(robot->pose.pos, defense_robots);
      double x_diff = std::abs(world_model->getOurGoalCenter().x() - robot->pose.pos.x());
      double score = [&]() {
        double angle_deg_width = angle_width * boost::math::constants::radian<double>();
        if (angle_deg_width > 15.0) {
          return angle_deg_width;
        } else {
          return angle_deg_width + 10.0 - std::clamp(x_diff * 2.0, 1.0, 10.0);
        }
      }();
      return std::make_pair(robot, score);
    }) |
    ranges::to<std::vector>();

  // 高スコアが前
  std::ranges::sort(robots_and_scores, [&](auto & a, auto & b) { return a.second > b.second; });
  return robots_and_scores;
}

auto MarkerPlanner::assignMarkingTarget(
  uint8_t selectable_robots_num, const std::vector<uint8_t> selectable_robots)
  -> std::vector<uint8_t>
{
  auto dander_enemies = getDangerEnemies();

  for (const auto & [robot, score] : dander_enemies) {
    visualizer->text()
      .text("MarkerScore: " + std::to_string(score))
      .fontSize(100)
      .position(robot->pose.pos + Point(0., 0.2))
      .build();
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
    auto robot_with_distance =
      remaining_selectable_robots | ranges::views::transform([&](const auto & robot) {
        return std::make_pair(robot, (robot->pose.pos - enemy_robot->pose.pos).norm());
      }) |
      ranges::to<std::vector>();

    if (not robot_with_distance.empty()) {
      auto best_marking_robot =
        ranges::min_element(robot_with_distance, [](const auto & a, const auto & b) {
          return a.second < b.second;
        })->first;

      // marking_target_map[best_marking_robot->id] = enemy_robot->id;
      selected_robots.push_back(best_marking_robot->id);
      remaining_selectable_robots.erase(ranges::find_if(
        remaining_selectable_robots,
        [best_marking_robot](const auto & robot) { return robot->id == best_marking_robot->id; }));

      // skillを作って設定
      markers.emplace_back(std::make_shared<skills::Marker>(
        "marker_planner", static_cast<uint8_t>(best_marking_robot->id), world_model));

      markers.back()->setParameter("marking_robot_id", enemy_robot->id);
      if ((world_model->ball.pos - enemy_robot->pose.pos).norm() > 3.0) {
        markers.back()->setParameter("mark_mode", std::string("intercept_pass"));
        markers.back()->setParameter("mark_distance", 0.5);
      } else {
        markers.back()->setParameter("mark_mode", std::string("save_goal"));
        double distance = (world_model->goal - enemy_robot->pose.pos).norm() * 0.1 + 0.2;
        markers.back()->setParameter("mark_distance", distance);
      }

      visualizer->circle()
        .center(enemy_robot->pose.pos)
        .radius(0.3)
        .stroke("black")
        .strokeWidth(10)
        .build();
      visualizer->line()
        .start(best_marking_robot->pose.pos)
        .end(
          enemy_robot->pose.pos +
          (enemy_robot->pose.pos - best_marking_robot->pose.pos).normalized() * 0.3)
        .stroke("black")
        .strokeWidth(20)
        .build();
    }
  }
  return selectable_robots;
}
}  // namespace crane
