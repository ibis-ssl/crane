// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__ATTACKER_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__ATTACKER_PLANNER_HPP_

#include <algorithm>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
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
class AttackerPlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit AttackerPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("attacker", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override;

protected:
  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override
  {
    return this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots,
      [this](const std::shared_ptr<RobotInfo> & robot) {
        // ボールに近いほどスコアが高い
        return 100.0 - std::max(world_model->getSquareDistanceFromRobotToBall(robot->id), 0.01);
      },
      prev_roles,
      [this](const std::shared_ptr<RobotInfo> & robot) {
        // ヒステリシスは1m
        return 1.;
      });
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__ATTACKER_PLANNER_HPP_
