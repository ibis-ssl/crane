// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/modern_orca_planner.hpp"

#include <robocup_ssl_msgs/msg/referee.hpp>

namespace crane
{
ModernORCAPlanner::ModernORCAPlanner(rclcpp::Node & node)
: LocalPlannerBase("modern_orca_local_planner", node),
  acceleration_factor("acceleration_factor", node, 1.5)
{
  node.declare_parameter("max_vel", MAX_VEL);
  MAX_VEL = node.get_parameter("max_vel").as_double();

  node.declare_parameter("max_acc", ACCELERATION);
  ACCELERATION = node.get_parameter("max_acc").as_double();

  // Initialize SSL constraint manager
  ssl_constraint_manager_ = std::make_unique<modern_orca::SSLConstraintManagerForCircularAgent>();

  RCLCPP_INFO(node.get_logger(), "ModernORCAPlanner initialized with SSL constraint manager");
}

auto ModernORCAPlanner::calculateRobotCommand(
  const crane_msgs::msg::RobotCommands & msg, double theta_offset) -> crane_msgs::msg::RobotCommands
{
  // Update agents from commands
  updateAgentsFromCommands(msg);
  
  // Update constraints from world model
  updateConstraintsFromWorldModel();
  
  // Generate commands using ORCA with SSL constraints
  return generateCommandsFromORCA(msg, theta_offset);
}

void ModernORCAPlanner::updateAgentsFromCommands(const crane_msgs::msg::RobotCommands & commands)
{
  for (const auto & command : commands.robot_commands) {
    const auto robot_id = command.robot_id;
    
    // Convert current position and velocity to modern_orca types
    Vector2d position(command.current_pose.x, command.current_pose.y);
    Vector2d velocity(command.current_velocity.x, command.current_velocity.y);
    Vector2d preferred_velocity(0.0, 0.0); // Will be set based on command type
    
    // Calculate preferred velocity based on command type
    switch (command.control_mode) {
      case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE: {
        if (!command.position_target_mode.empty()) {
          const auto & target = command.position_target_mode.front();
          Vector2d target_pos(target.target_x, target.target_y);
          preferred_velocity = (target_pos - position).normalized() * MAX_VEL;
        }
        break;
      }
      case crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE: {
        if (!command.simple_velocity_target_mode.empty()) {
          const auto & vel_target = command.simple_velocity_target_mode.front();
          preferred_velocity = Vector2d(vel_target.target_vx, vel_target.target_vy);
        }
        break;
      }
      case crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE: {
        if (!command.polar_velocity_target_mode.empty()) {
          const auto & polar_target = command.polar_velocity_target_mode.front();
          const double v_r = polar_target.target_velocity_r;
          const double v_theta = polar_target.target_velocity_theta;
          preferred_velocity = Vector2d(v_r * cos(v_theta), v_r * sin(v_theta));
        }
        break;
      }
    }
    
    // Create or update agent
    if (agents_.find(robot_id) == agents_.end()) {
      agents_[robot_id] = std::make_unique<modern_orca::CircularAgent>(
        robot_id, position, preferred_velocity, MAX_VEL, 0.09); // 9cm radius
    } else {
      agents_[robot_id]->setPosition(position);
      agents_[robot_id]->setVelocity(velocity);
      agents_[robot_id]->setPreferredVelocity(preferred_velocity);
      agents_[robot_id]->setMaxSpeed(std::min(static_cast<double>(command.local_planner_config.max_velocity), MAX_VEL));
    }
  }
}

void ModernORCAPlanner::updateConstraintsFromWorldModel()
{
  if (world_model) {
    // Update constraint manager with current world model
    ssl_constraint_manager_->updateFromWorldModel(world_model);
    
    // Update referee command
    const auto referee_command = world_model->getMsg().play_situation.command_raw.value;
    ssl_constraint_manager_->updateFromRefereeCommand(referee_command);
    
    // Apply automatic constraint adjustments
    ssl_constraint_manager_->applyAutomaticConstraintAdjustments();
  }
}

crane_msgs::msg::RobotCommands ModernORCAPlanner::generateCommandsFromORCA(
  const crane_msgs::msg::RobotCommands & original_commands, double theta_offset)
{
  crane_msgs::msg::RobotCommands result = original_commands;
  
  for (auto & command : result.robot_commands) {
    const auto robot_id = command.robot_id;
    
    if (agents_.find(robot_id) != agents_.end()) {
      auto & agent = *agents_[robot_id];
      
      // Generate constraints for this agent
      auto constraints = ssl_constraint_manager_->generateAllHalfPlanes(agent, 0.1); // 0.1s time step
      
      // For now, use preferred velocity as the ORCA result
      // TODO: Implement proper ORCA solver integration
      auto preferred_vel = agent.preferredVelocity();
      
      // Apply constraints by projecting preferred velocity
      for (const auto & constraint : constraints) {
        if (!constraint.contains(agent.position() + preferred_vel * 0.1)) {
          // Project velocity to satisfy constraint
          auto projected_point = constraint.project(agent.position() + preferred_vel * 0.1);
          preferred_vel = (projected_point - agent.position()) / 0.1;
        }
      }
      
      // Convert back to ROS message format
      command.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
      command.polar_velocity_target_mode.clear();
      command.polar_velocity_target_mode.reserve(1);
      
      crane_msgs::msg::PolarVelocityTargetMode target;
      target.target_velocity_r = preferred_vel.norm();
      target.target_velocity_theta = std::atan2(preferred_vel.y(), preferred_vel.x()) + theta_offset;
      
      command.polar_velocity_target_mode.push_back(target);
    }
  }
  
  return result;
}

}  // namespace crane
