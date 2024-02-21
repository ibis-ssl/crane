// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__MARKER_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__MARKER_PLANNER_HPP_

#include <algorithm>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <crane_robot_skills/marker.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class MarkerPlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit MarkerPlanner(
    WorldModelWrapper::SharedPtr & world_model, ConsaiVisualizerWrapper::SharedPtr visualizer)
  : PlannerBase("marker", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::vector<crane_msgs::msg::RobotCommand> robot_commands;

    for (auto & [id, value] : skill_map) {
      auto & [command, skill] = value;
      robot_commands.emplace_back(command->getMsg());
      skill->run(visualizer);
    }
    return {PlannerBase::Status::RUNNING, robot_commands};
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots)
    -> std::vector<uint8_t> override
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
          std::count(selected_robots.begin(), selected_robots.end(), j) == 0) {
          min_distance = distance;
          min_index = j;
        }
      }
      marking_target_map[selectable_robots[min_index]] = enemy_robot->id;
      selected_robots.push_back(selectable_robots[min_index]);
      skill_map.emplace(
        selectable_robots[min_index],
        std::make_pair(
          std::make_shared<RobotCommandWrapper>(selectable_robots[min_index], world_model),
          std::make_shared<skills::Marker>(selectable_robots[min_index], world_model)));
      skill_map[selectable_robots[min_index]].second->setParameter(
        "marking_robot_id", enemy_robot->id);
      skill_map[selectable_robots[min_index]].second->setParameter("mark_distance", 0.5);
    }

    return selected_robots;
  }

private:
  // key: ID of our robot in charge, value: ID of the enemy marked robot
  std::unordered_map<uint8_t, uint8_t> marking_target_map;

  std::unordered_map<
    uint8_t, std::pair<std::shared_ptr<RobotCommandWrapper>, std::shared_ptr<skills::Marker>>>
    skill_map;
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__MARKER_PLANNER_HPP_
