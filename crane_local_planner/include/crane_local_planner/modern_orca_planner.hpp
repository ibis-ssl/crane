// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__MODERN_ORCA_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__MODERN_ORCA_PLANNER_HPP_

#include <crane_basics/parameter_with_event.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <memory>
#include <modern_orca/modern_orca.hpp>
#include <modern_orca/ssl_constraints/ssl_constraint_manager.hpp>
#include <unordered_map>

#include "planner_base.hpp"

namespace crane
{
class ModernORCAPlanner : public LocalPlannerBase
{
public:
  explicit ModernORCAPlanner(rclcpp::Node & node);

  auto calculateRobotCommand(const crane_msgs::msg::RobotCommands & msg, double theta_offset)
    -> crane_msgs::msg::RobotCommands override;

private:
  std::unique_ptr<modern_orca::SSLConstraintManagerForCircularAgent> ssl_constraint_manager_;
  std::unordered_map<uint32_t, std::unique_ptr<modern_orca::CircularAgent>> agents_;

  double MAX_VEL = 4.0;
  double ACCELERATION = 4.0;
  ParameterWithEvent<double> acceleration_factor;

  // Helper methods
  void updateAgentsFromCommands(const crane_msgs::msg::RobotCommands & commands);
  void updateConstraintsFromWorldModel();
  crane_msgs::msg::RobotCommands generateCommandsFromORCA(
    const crane_msgs::msg::RobotCommands & original_commands, double theta_offset);
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__MODERN_ORCA_PLANNER_HPP_
