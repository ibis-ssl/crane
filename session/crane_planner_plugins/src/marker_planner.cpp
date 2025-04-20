// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_basics/position_assignments.hpp>
#include <crane_planner_plugins/marker_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
MarkerPlanner::calculateRobotCommand(
  [[maybe_unused]] const std::vector<RobotIdentifier> & robots, PlannerContext & context)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;

  for (auto & [id, skill] : skill_map) {
    skill->run();
    robot_commands.emplace_back(skill->getRobotCommand());
  }
  return {PlannerBase::Status::RUNNING, robot_commands};
}
auto MarkerPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
  -> std::vector<uint8_t>
{
  if (selectable_robots_num >= selectable_robots.size()) {
    selectable_robots_num = selectable_robots.size();
  }

  marking_target_map.clear();
  skill_map.clear();
  std::vector<uint8_t> selected_robots;

  RobotList defense_robots;
  for (const auto & prev_role : prev_roles) {
    // cspell: ignore defen
    if (
      prev_role.second.planner_name.find("goalie") != std::string::npos ||
      prev_role.second.planner_name.find("defen") != std::string::npos) {
      try {
        defense_robots.emplace_back(world_model->getOurRobot(prev_role.first));
      } catch (...) {
      }
    }
  }

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
        double angle_deg_width = angle_width * boost::math::constants::degree<double>();
        if (angle_deg_width > 3.0) {
          return angle_deg_width;
        } else {
          return 3.0 - std::clamp(x_diff, 10., 1.0);
        }
      }();
      return std::make_pair(robot, score);
    }) |
    ranges::to<std::vector>();

  std::ranges::sort(robots_and_scores, [&](auto & a, auto & b) {
    // ゴールへの角度が大きいほど選択優先度が高い
    return a.second > b.second;
  });

  // selectable_robots_numより大きければ末尾を削除
  if (robots_and_scores.size() > selectable_robots_num) {
    robots_and_scores.resize(selectable_robots_num);
  }

  RobotList remaining_selectable_robots =
    selectable_robots |
    ranges::views::transform([&](const auto & id) { return world_model->getOurRobot(id); }) |
    ranges::to<std::vector>();

  for (const auto & [enemy_robot, score] : robots_and_scores) {
    // マークする敵ロボットに一番近い味方ロボットを選択
    auto robot_with_distance =
      remaining_selectable_robots | ranges::views::transform([&](const auto & robot) {
        return std::make_pair(robot, (robot->pose.pos - enemy_robot->pose.pos).norm());
      }) |
      ranges::to<std::vector>();

    auto best_marking_robot =
      ranges::min_element(robot_with_distance, [](const auto & a, const auto & b) {
        return a.second < b.second;
      })->first;
    marking_target_map[best_marking_robot->id] = enemy_robot->id;
    selected_robots.push_back(best_marking_robot->id);
    remaining_selectable_robots.erase(ranges::find_if(
      remaining_selectable_robots,
      [best_marking_robot](const auto & robot) { return robot->id == best_marking_robot->id; }));
    skill_map.try_emplace(
      best_marking_robot->id,
      std::make_shared<skills::Marker>(
        "marker_planner", static_cast<uint8_t>(best_marking_robot->id), world_model));
    skill_map[best_marking_robot->id]->setParameter("marking_robot_id", enemy_robot->id);
    if ((world_model->ball.pos - enemy_robot->pose.pos).norm() > 3.0) {
      skill_map[best_marking_robot->id]->setParameter("mark_mode", std::string("intercept_pass"));
      skill_map[best_marking_robot->id]->setParameter("mark_distance", 0.5);
    } else {
      skill_map[best_marking_robot->id]->setParameter("mark_mode", std::string("save_goal"));
      double distance = (world_model->goal - enemy_robot->pose.pos).norm() * 0.1 + 0.2;
      skill_map[best_marking_robot->id]->setParameter("mark_distance", distance);
    }
  }

  return selected_robots;
}
}  // namespace crane
