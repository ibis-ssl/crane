// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/marker_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
MarkerPlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
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
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  if (selectable_robots_num >= selectable_robots.size()) {
    selectable_robots_num = selectable_robots.size();
  }

  marking_target_map.clear();
  skill_map.clear();
  std::vector<uint8_t> selected_robots;
  // 味方ゴールに近い敵ロボットをselectable_robots_num台選択
  std::vector<std::pair<std::shared_ptr<RobotInfo>, double>> robots_and_distances;
  for (auto enemy_robot : world_model->theirs.getAvailableRobots()) {
    robots_and_distances.emplace_back(
      enemy_robot, std::abs(world_model->goal.x() - enemy_robot->pose.pos.x()));
  }
  std::sort(robots_and_distances.begin(), robots_and_distances.end(), [&](auto & a, auto & b) {
    return a.second < b.second;
  });

  for (int i = 0; i < selectable_robots_num; i++) {
    auto enemy_robot = robots_and_distances.at(i).first;
    // マークする敵ロボットに一番近い味方ロボットを選択
    double min_distance = 1000000.0;
    uint8_t min_index = 0;
    for (int j = 0; j < selectable_robots.size(); j++) {
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
    skill_map.emplace(
      selectable_robots[min_index],
      std::make_shared<skills::Marker>(selectable_robots[min_index], world_model));
    skill_map[selectable_robots[min_index]]->setParameter("marking_robot_id", enemy_robot->id);
    if ((world_model->ball.pos - world_model->goal).norm() > 6.0) {
      skill_map[selectable_robots[min_index]]->setParameter(
        "mark_mode", std::string("intercept_pass"));
      skill_map[selectable_robots[min_index]]->setParameter("mark_distance", 0.5);
    } else {
      skill_map[selectable_robots[min_index]]->setParameter("mark_mode", std::string("save_goal"));
      double distance = (world_model->goal - enemy_robot->pose.pos).norm() * 0.1 + 0.2;
      skill_map[selectable_robots[min_index]]->setParameter("mark_distance", distance);
    }
  }

  return selected_robots;
}
}  // namespace crane
