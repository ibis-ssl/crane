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

  RCLCPP_INFO(node.get_logger(), "ModernORCAPlanner initialized with SSL constraint manager and ORCA solver");
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
    Vector2d preferred_velocity(0.0, 0.0);  // Will be set based on command type

    // Calculate preferred velocity based on command type
    switch (command.control_mode) {
      case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE: {
        if (!command.position_target_mode.empty()) {
          // Use advanced trapezoidal velocity profile for position control
          preferred_velocity = calculateTrapezoidalVelocityProfile(command, position);
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
        robot_id, position, preferred_velocity, MAX_VEL, 0.09);  // 9cm radius
    } else {
      agents_[robot_id]->setPosition(position);
      agents_[robot_id]->setVelocity(velocity);
      agents_[robot_id]->setPreferredVelocity(preferred_velocity);
      agents_[robot_id]->setMaxSpeed(
        std::min(static_cast<double>(command.local_planner_config.max_velocity), MAX_VEL));
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
      auto constraints =
        ssl_constraint_manager_->generateAllHalfPlanes(agent, 0.1);  // 0.1s time step

      // Use ORCA solver to find optimal velocity
      auto preferred_vel = agent.preferredVelocity();
      
      // Create solver with agent's max speed
      modern_orca::OptimalLinearProgram2DSolver agent_solver(agent.maxSpeed());
      
      // Solve for optimal velocity using ORCA
      auto optimal_vel = agent_solver.solve(constraints, preferred_vel);

      // Use the solver result
      preferred_vel = optimal_vel;

      // Set final planned values if this was a position target
      if (original_commands.robot_commands[&command - &result.robot_commands[0]].control_mode == 
          crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE) {
        command.local_planner_config.final_planned_max_acceleration = final_planned_acceleration_;
        command.local_planner_config.final_planned_max_velocity = final_planned_max_velocity_;
      }

      // Convert back to ROS message format
      command.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
      command.polar_velocity_target_mode.clear();
      command.polar_velocity_target_mode.reserve(1);

      crane_msgs::msg::PolarVelocityTargetMode target;
      target.target_velocity_r = preferred_vel.norm();
      target.target_velocity_theta =
        std::atan2(preferred_vel.y(), preferred_vel.x()) + theta_offset;

      command.polar_velocity_target_mode.push_back(target);
    }
  }

  // Store commands for next iteration
  pre_commands = result;
  return result;
}

Vector2d ModernORCAPlanner::calculateTrapezoidalVelocityProfile(
  const crane_msgs::msg::RobotCommand & command, const Point & current_position)
{
  if (command.position_target_mode.empty()) {
    return Vector2d(0.0, 0.0);
  }

  const auto & target_config = command.position_target_mode.front();
  Vector2d target_pos(target_config.target_x, target_config.target_y);
  Vector2d position_diff = target_pos - current_position;
  
  // Check if within position tolerance first
  if (isWithinPositionTolerance(command, current_position)) {
    return Vector2d(0.0, 0.0);
  }

  // Calculate target velocity direction
  Vector2d target_vel = position_diff;

  // Apply square root velocity scaling for deceleration
  // v = sqrt(2 * a * x) for each axis
  double max_acc = std::min(
    ACCELERATION, static_cast<double>(command.local_planner_config.max_acceleration));
  
  target_vel.x() = std::copysign(
    std::sqrt(2.0 * max_acc * std::abs(target_vel.x())), target_vel.x());
  target_vel.y() = std::copysign(
    std::sqrt(2.0 * max_acc * std::abs(target_vel.y())), target_vel.y());

  // Get previous velocity for acceleration limiting
  double pre_vel = getPreviousVelocity(command.robot_id);
  
  // Calculate acceleration and deceleration limits
  double acceleration = max_acc * acceleration_factor.getValue();

  // Velocity limit by deceleration distance
  double max_vel_by_decel = std::sqrt(2.0 * acceleration * position_diff.norm());

  // Velocity limit by acceleration constraint
  double max_vel_by_acc = pre_vel + acceleration * ORCA_TIME_STEP;

  // Combine all velocity limits
  double max_vel = std::min(
    static_cast<double>(command.local_planner_config.max_velocity), MAX_VEL);
  max_vel = std::min(max_vel, max_vel_by_decel);
  max_vel = std::min(max_vel, max_vel_by_acc);

  // Apply referee command velocity limits
  if (world_model && 
      world_model->getMsg().play_situation.command_raw.value == 
      robocup_ssl_msgs::msg::Referee::COMMAND_STOP) {
    max_vel = std::min(max_vel, 1.0);
  }

  // Store final planned values (these will be set in the calling function)
  final_planned_acceleration_ = acceleration;
  final_planned_max_velocity_ = max_vel;

  // Normalize and scale to maximum velocity
  target_vel = target_vel.normalized() * max_vel;

  // Apply terminal velocity constraint
  if (target_vel.norm() < command.local_planner_config.terminal_velocity) {
    target_vel = target_vel.normalized() * command.local_planner_config.terminal_velocity;
  }

  return target_vel;
}

double ModernORCAPlanner::getPreviousVelocity(uint32_t robot_id) const
{
  auto it = std::find_if(
    pre_commands.robot_commands.begin(), pre_commands.robot_commands.end(),
    [robot_id](const auto & c) { return c.robot_id == robot_id; });
    
  if (it != pre_commands.robot_commands.end()) {
    if (!it->simple_velocity_target_mode.empty()) {
      const auto & vel = it->simple_velocity_target_mode.front();
      return std::hypot(vel.target_vx, vel.target_vy);
    } else if (!it->polar_velocity_target_mode.empty()) {
      return it->polar_velocity_target_mode.front().target_velocity_r;
    }
  }
  return 0.0;
}

bool ModernORCAPlanner::isWithinPositionTolerance(
  const crane_msgs::msg::RobotCommand & command, const Point & current_position) const
{
  if (command.control_mode != crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE ||
      command.position_target_mode.empty()) {
    return false;
  }

  const auto & target_config = command.position_target_mode.front();
  double distance = std::hypot(
    target_config.target_x - current_position.x(),
    target_config.target_y - current_position.y());

  // Check explicit position tolerance
  if (distance < target_config.position_tolerance) {
    return true;
  }

  // Default tolerance when terminal velocity is 0
  if (command.local_planner_config.terminal_velocity == 0.0 &&
      target_config.position_tolerance == 0.0 && distance < 0.03) {
    return true;
  }

  return false;
}

}  // namespace crane
