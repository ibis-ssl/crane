// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__MARKER_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__MARKER_PLANNER_HPP_

#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/control_target.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

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

  std::vector<crane_msgs::msg::RobotCommand> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::vector<crane_msgs::msg::RobotCommand> control_targets;
    for (auto robot_id : robots) {
      auto robot = world_model->getRobot(robot_id);

      auto marked_robot = world_model->getTheirRobot(marking_target_map.at(robot_id.robot_id));
      auto enemy_pos = marked_robot->pose.pos;
      constexpr double INTERVAL = 0.5;

      Point marking_point =
        enemy_pos + (world_model->getOurGoalCenter() - enemy_pos).normalized() * INTERVAL;
      double target_theta = getAngle(enemy_pos - world_model->getOurGoalCenter());

      crane::RobotCommandWrapper target(robot_id.robot_id, world_model);
      target.setTargetPosition(marking_point, target_theta);

      control_targets.emplace_back(target.getMsg());
    }
    return control_targets;
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots)
    -> std::vector<uint8_t> override
  {
    if (selectable_robots_num >= selectable_robots.size()) {
      selectable_robots_num = selectable_robots.size();
    }

    marking_target_map.clear();
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
          (enemy_robot->pose.pos - world_model->getOurRobot(selectable_robots[j])->pose.pos).norm();
        if (
          distance < min_distance &&
          std::count(selected_robots.begin(), selected_robots.end(), j) == 0) {
          min_distance = distance;
          min_index = j;
        }
      }
      marking_target_map[selectable_robots[min_index]] = enemy_robot->id;
      selected_robots.push_back(selectable_robots[min_index]);
    }

    return selected_robots;
  }

private:
  // key: ID of our robot in charge, value: ID of the enemy marked robot
  std::unordered_map<uint8_t, uint8_t> marking_target_map;
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__MARKER_PLANNER_HPP_
