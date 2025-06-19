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
#include <crane_basics/geometry_operations.hpp>
#include <memory>
#include <modern_orca/modern_orca.hpp>
#include <modern_orca/ssl_constraints/ssl_constraint_manager.hpp>
#include <modern_orca/constraints/orca_constraint.hpp>
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
  double ORCA_TIME_STEP = 0.1;
  ParameterWithEvent<double> acceleration_factor;

  // Store previous commands for velocity planning
  crane_msgs::msg::RobotCommands pre_commands;

  // Temporary storage for final planned values
  mutable double final_planned_acceleration_ = 0.0;
  mutable double final_planned_max_velocity_ = 0.0;

  // Helper methods
  void updateAgentsFromCommands(const crane_msgs::msg::RobotCommands & commands);
  void updateConstraintsFromWorldModel();
  crane_msgs::msg::RobotCommands generateCommandsFromORCA(
    const crane_msgs::msg::RobotCommands & original_commands, double theta_offset);
  
  // Advanced position control methods
  Vector2d calculateTrapezoidalVelocityProfile(
    const crane_msgs::msg::RobotCommand & command, const Point & current_position);
  double getPreviousVelocity(uint32_t robot_id) const;
  bool isWithinPositionTolerance(
    const crane_msgs::msg::RobotCommand & command, const Point & current_position) const;
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__MODERN_ORCA_PLANNER_HPP_
