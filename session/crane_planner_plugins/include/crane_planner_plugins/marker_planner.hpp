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
#include <string>
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
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("marker", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override;

private:
  // key: ID of our robot in charge, value: ID of the enemy marked robot
  std::unordered_map<uint8_t, uint8_t> marking_target_map;

  std::unordered_map<uint8_t, std::shared_ptr<skills::Marker>> skill_map;
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__MARKER_PLANNER_HPP_
