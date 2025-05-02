// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__KICK_TEST_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__KICK_TEST_PLANNER_HPP_

#include <algorithm>
#include <crane_basics/parameter_with_event.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_plugins/planner_base.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class KickTestPlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit KickTestPlanner(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node);

  auto calculateRobotCommand(const std::vector<RobotIdentifier> & robots, PlannerContext &)
    -> std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
    -> std::vector<uint8_t> override;

private:
  ParameterWithEvent<double> kick_power;

  ParameterWithEvent<bool> chip_enable;

  ParameterWithEvent<double> dribble_power;

  ParameterWithEvent<double> target_x;

  ParameterWithEvent<double> max_vel;
};
}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__KICK_TEST_PLANNER_HPP_
