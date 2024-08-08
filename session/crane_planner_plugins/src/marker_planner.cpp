// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/marker_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
MarkerPlanner::calculateRobotCommand([[maybe_unused]] const std::vector<RobotIdentifier> & robots)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;

  for (auto & [id, skill] : skill_map) {
    skill->run(visualizer);
    robot_commands.emplace_back(skill->getRobotCommand());
  }
  return {PlannerBase::Status::RUNNING, robot_commands};
}
auto MarkerPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  [[maybe_unused]] const std::unordered_map<uint8_t, RobotRole> & prev_roles)
  -> std::vector<uint8_t>
{
  if (selectable_robots_num >= selectable_robots.size()) {
    selectable_robots_num = selectable_robots.size();
  }

  marking_target_map.clear();
  skill_map.clear();
  std::vector<uint8_t> selected_robots;
  // 味方ゴールに近い敵ロボットをselectable_robots_num台選択
  std::vector<std::pair<std::shared_ptr<RobotInfo>, double>> robots_and_goal_angles;
  for (const auto & enemy_robot : world_model->theirs.getAvailableRobots()) {
    auto [best_angle, angle_width] =
      world_model->getLargestOurGoalAngleRangeFromPoint(enemy_robot->pose.pos);
    robots_and_goal_angles.emplace_back(enemy_robot, angle_width);
  }
  std::sort(robots_and_goal_angles.begin(), robots_and_goal_angles.end(), [&](auto & a, auto & b) {
    // ゴールへの角度が大きいほど選択優先度が高い
    return a.second > b.second;
  });

  for (int i = 0; i < selectable_robots_num; i++) {
    auto enemy_robot = robots_and_goal_angles.at(i).first;
    if (not world_model->point_checker.isInOurHalf(enemy_robot->pose.pos)) {
      // 相手コートにいる敵ロボットはマークしない
      continue;
    } else if (robots_and_goal_angles.at(i).second < 3.0 * M_PI / 180.) {
      // シュートコースが狭い場合はマークしない
      continue;
    } else if ((enemy_robot->pose.pos - world_model->ball.pos).norm() < 1.0) {
      // ボールに近い敵ロボットはマークしない
      continue;
    } else {
      // マークする敵ロボットに一番近い味方ロボットを選択
      double min_distance = 1000000.0;
      uint8_t min_index = 0;
      for (size_t j = 0; j < selectable_robots.size(); j++) {
        double distance =
          world_model->getOurRobot(selectable_robots[j])->getDistance(enemy_robot->pose.pos);
        if (
          distance < min_distance &&
          std::count(selected_robots.begin(), selected_robots.end(), selectable_robots[j]) == 0) {
          min_distance = distance;
          min_index = j;
        }
      }
      marking_target_map[selectable_robots[min_index]] = enemy_robot->id;
      selected_robots.push_back(selectable_robots[min_index]);
      auto marker_base = std::make_shared<RobotCommandWrapperBase>(
        "marker_planner", selectable_robots[min_index], world_model);
      skill_map.emplace(
        selectable_robots[min_index], std::make_shared<skills::Marker>(marker_base));
      skill_map[selectable_robots[min_index]]->setParameter("marking_robot_id", enemy_robot->id);
      if ((world_model->ball.pos - enemy_robot->pose.pos).norm() > 3.0) {
        skill_map[selectable_robots[min_index]]->setParameter(
          "mark_mode", std::string("intercept_pass"));
        skill_map[selectable_robots[min_index]]->setParameter("mark_distance", 0.5);
      } else {
        skill_map[selectable_robots[min_index]]->setParameter(
          "mark_mode", std::string("save_goal"));
        double distance = (world_model->goal - enemy_robot->pose.pos).norm() * 0.1 + 0.2;
        skill_map[selectable_robots[min_index]]->setParameter("mark_distance", distance);
      }
    }
  }

  return selected_robots;
}
}  // namespace crane
