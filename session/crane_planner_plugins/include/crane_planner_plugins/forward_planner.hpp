// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__FORWARD_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__FORWARD_PLANNER_HPP_

#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_planner_plugins/planner_base.hpp>
#include <crane_robot_skills/forward.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class ForwardPlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit ForwardPlanner(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : PlannerBase("forward", world_model)
  {
  }

  auto createForwardLines() const -> std::vector<Segment>;

  auto calculateRobotCommand(const std::vector<RobotIdentifier> & robots, PlannerContext &)
    -> std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
    -> std::vector<uint8_t> override;

  std::vector<std::shared_ptr<skills::Forward>> forward_skills;
};
}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__FORWARD_PLANNER_HPP_
