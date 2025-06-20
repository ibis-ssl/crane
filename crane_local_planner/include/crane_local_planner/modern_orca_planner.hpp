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
#include <crane_msgs/msg/robot_feedback_array.hpp>
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
  
  rclcpp::Node & node_;  // Store reference to node for logging
  
  // Robot feedback subscription
  rclcpp::Subscription<crane_msgs::msg::RobotFeedbackArray>::SharedPtr sub_feedback_array;
  crane_msgs::msg::RobotFeedbackArray latest_feedback;

  double MAX_VEL = 4.0;
  double ACCELERATION = 4.0;
  double ORCA_TIME_STEP = 0.1;
  double ORCA_NEIGHBOR_DIST = 15.0;
  int ORCA_MAX_NEIGHBORS = 10;
  double ORCA_TIME_HORIZON = 2.0;
  double ORCA_RADIUS = 0.05;
  double ORCA_MAX_SPEED = 4.0;
  double ORCA_TRAPEZOIDAL_FRAME_RATE = 60.0;
  
  // Debug visualization parameters
  bool debug_visualize_constraints_ = false;
  bool debug_visualize_orca_lines_ = false;
  bool debug_show_performance_metrics_ = false;
  
  ParameterWithEvent<double> acceleration_factor;

  // Store previous commands for velocity planning
  crane_msgs::msg::RobotCommands pre_commands;

  // Temporary storage for final planned values
  mutable double final_planned_acceleration_ = 0.0;
  mutable double final_planned_max_velocity_ = 0.0;
  
  // Performance monitoring variables
  mutable double solve_time_ms_ = 0.0;
  mutable double total_constraints_ = 0.0;

  // Helper methods
  void updateAgentsFromCommands(const crane_msgs::msg::RobotCommands & commands);
  void updateConstraintsFromWorldModel();
  crane_msgs::msg::RobotCommands generateCommandsFromORCA(
    const crane_msgs::msg::RobotCommands & original_commands, double theta_offset);
  
  // Constraint configuration methods
  void applyConstraintFlags(const crane_msgs::msg::LocalPlannerConfig & config);
  void setConstraintLineup(const modern_orca::SSLConstraintManagerForCircularAgent::ConstraintLineup & lineup);
  modern_orca::SSLConstraintManagerForCircularAgent::ConstraintLineup getConstraintLineup() const;
  void enableConstraint(modern_orca::SSLConstraintType type, bool enabled);
  bool isConstraintEnabled(modern_orca::SSLConstraintType type) const;
  
  // Debug visualization methods
  void visualizeConstraints(uint32_t robot_id, const std::vector<modern_orca::HalfPlane> & constraints);
  void visualizeORCALines(uint32_t robot_id, const std::vector<modern_orca::HalfPlane> & orca_constraints);
  void visualizePerformanceMetrics();
  
  // Advanced position control methods
  Vector2d calculateTrapezoidalVelocityProfile(
    const crane_msgs::msg::RobotCommand & command, const Point & current_position);
  double getPreviousVelocity(uint32_t robot_id) const;
  bool isWithinPositionTolerance(
    const crane_msgs::msg::RobotCommand & command, const Point & current_position) const;
  
  // Robot feedback integration
  Point getCurrentPosition(const crane_msgs::msg::RobotCommand & command) const;
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__MODERN_ORCA_PLANNER_HPP_
