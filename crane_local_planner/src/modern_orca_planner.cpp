// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/modern_orca_planner.hpp"

#include <chrono>
#include <robocup_ssl_msgs/msg/referee.hpp>
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace crane
{
ModernORCAPlanner::ModernORCAPlanner(rclcpp::Node & node)
: LocalPlannerBase("modern_orca_local_planner", node),
  node_(node),
  acceleration_factor("acceleration_factor", node, 1.5)
{
  node.declare_parameter("max_vel", MAX_VEL);
  MAX_VEL = node.get_parameter("max_vel").as_double();

  node.declare_parameter("max_acc", ACCELERATION);
  ACCELERATION = node.get_parameter("max_acc").as_double();

  // Declare ORCA-specific parameters (equivalent to RVO2 parameters)
  node.declare_parameter("orca_time_step", ORCA_TIME_STEP);
  ORCA_TIME_STEP = node.get_parameter("orca_time_step").as_double();
  node.declare_parameter("orca_neighbor_dist", ORCA_NEIGHBOR_DIST);
  ORCA_NEIGHBOR_DIST = node.get_parameter("orca_neighbor_dist").as_double();
  node.declare_parameter("orca_max_neighbors", ORCA_MAX_NEIGHBORS);
  ORCA_MAX_NEIGHBORS = node.get_parameter("orca_max_neighbors").as_int();
  node.declare_parameter("orca_time_horizon", ORCA_TIME_HORIZON);
  ORCA_TIME_HORIZON = node.get_parameter("orca_time_horizon").as_double();
  node.declare_parameter("orca_radius", ORCA_RADIUS);
  ORCA_RADIUS = node.get_parameter("orca_radius").as_double();
  node.declare_parameter("orca_max_speed", ORCA_MAX_SPEED);
  ORCA_MAX_SPEED = node.get_parameter("orca_max_speed").as_double();
  node.declare_parameter("orca_trapezoidal_frame_rate", ORCA_TRAPEZOIDAL_FRAME_RATE);
  ORCA_TRAPEZOIDAL_FRAME_RATE = node.get_parameter("orca_trapezoidal_frame_rate").as_double();

  // Initialize SSL constraint manager
  ssl_constraint_manager_ = std::make_unique<modern_orca::SSLConstraintManagerForCircularAgent>();

  // Declare constraint configuration parameters
  node.declare_parameter("constraint_ball_avoidance_enabled", true);
  node.declare_parameter("constraint_penalty_area_avoidance_enabled", true);
  node.declare_parameter("constraint_ball_placement_avoidance_enabled", true);
  node.declare_parameter("constraint_referee_command_enabled", true);
  node.declare_parameter("constraint_robot_collision_enabled", true);

  // Set initial constraint lineup from parameters
  modern_orca::SSLConstraintManagerForCircularAgent::ConstraintLineup initial_lineup;
  initial_lineup.ball_avoidance_enabled = node.get_parameter("constraint_ball_avoidance_enabled").as_bool();
  initial_lineup.penalty_area_avoidance_enabled = node.get_parameter("constraint_penalty_area_avoidance_enabled").as_bool();
  initial_lineup.ball_placement_avoidance_enabled = node.get_parameter("constraint_ball_placement_avoidance_enabled").as_bool();
  initial_lineup.referee_command_enabled = node.get_parameter("constraint_referee_command_enabled").as_bool();
  initial_lineup.robot_collision_enabled = node.get_parameter("constraint_robot_collision_enabled").as_bool();
  
  setConstraintLineup(initial_lineup);

  // Declare debug visualization parameters
  node.declare_parameter("debug_visualize_constraints", debug_visualize_constraints_);
  debug_visualize_constraints_ = node.get_parameter("debug_visualize_constraints").as_bool();
  node.declare_parameter("debug_visualize_orca_lines", debug_visualize_orca_lines_);
  debug_visualize_orca_lines_ = node.get_parameter("debug_visualize_orca_lines").as_bool();
  node.declare_parameter("debug_show_performance_metrics", debug_show_performance_metrics_);
  debug_show_performance_metrics_ = node.get_parameter("debug_show_performance_metrics").as_bool();

  // Initialize robot feedback subscription
  sub_feedback_array = node.create_subscription<crane_msgs::msg::RobotFeedbackArray>(
    "/robot_feedback", 1,
    [this](const crane_msgs::msg::RobotFeedbackArray & msg) { latest_feedback = msg; });

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

    // Get current position (with robot feedback integration)
    Point current_position = getCurrentPosition(command);
    
    // Convert current position and velocity to modern_orca types
    Vector2d position(current_position.x(), current_position.y());
    Vector2d velocity(command.current_velocity.x, command.current_velocity.y);
    Vector2d preferred_velocity(0.0, 0.0);  // Will be set based on command type

    // Calculate preferred velocity based on command type
    switch (command.control_mode) {
      case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE: {
        if (!command.position_target_mode.empty()) {
          // Use advanced trapezoidal velocity profile for position control
          preferred_velocity = calculateTrapezoidalVelocityProfile(command, current_position);
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

    // Calculate dynamic radius based on velocity (same as RVO2)
    double velocity_norm = velocity.norm();
    double dynamic_radius = ORCA_RADIUS + velocity_norm * 0.1; // Base radius + velocity factor

    // Use configured max speed or command override
    double max_speed = std::min(
      std::min(static_cast<double>(command.local_planner_config.max_velocity), MAX_VEL), 
      ORCA_MAX_SPEED);

    // Create or update agent
    if (agents_.find(robot_id) == agents_.end()) {
      agents_[robot_id] = std::make_unique<modern_orca::CircularAgent>(
        robot_id, position, preferred_velocity, max_speed, dynamic_radius);
    } else {
      agents_[robot_id]->setPosition(position);
      agents_[robot_id]->setVelocity(velocity);
      agents_[robot_id]->setPreferredVelocity(preferred_velocity);
      agents_[robot_id]->collisionModel().setRadius(dynamic_radius);
      agents_[robot_id]->setMaxSpeed(max_speed);
    }

    // Visualize friendly robot radius and velocity (same as RVO2)
    if (visualizer && world_model) {
      auto robot = world_model->getOurRobot(robot_id);
      if (robot) {
        visualizer->circle()
          .radius(dynamic_radius)
          .center(robot->pose.pos)
          .stroke("yellow", 0.2)
          .strokeWidth(10)
          .build();
        
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << velocity_norm << "m/s";
        visualizer->text()
          .text(ss.str())
          .fontSize(50)
          .position(robot->pose.pos + Point(0, dynamic_radius + 0.07))
          .textAnchor("middle")
          .fill("yellow", 0.5)
          .build();
      }
    }
  }

  // Add enemy robot agents
  if (world_model) {
    for (const auto & enemy_robot : world_model->theirs().robots) {
      const auto enemy_id = enemy_robot->id + 20; // Enemy robots: 20-39
      
      if (enemy_robot->available) {
        // Convert enemy robot position and velocity to modern_orca types
        Vector2d enemy_pos(enemy_robot->pose.pos.x(), enemy_robot->pose.pos.y());
        Vector2d enemy_vel(enemy_robot->vel.linear.x(), enemy_robot->vel.linear.y());
        
        // Calculate dynamic radius based on velocity (same as RVO2)
        double velocity_norm = enemy_vel.norm();
        double enemy_radius = ORCA_RADIUS + velocity_norm * 0.1; // Base radius + velocity factor
        
        if (agents_.find(enemy_id) == agents_.end()) {
          // Create new enemy agent
          agents_[enemy_id] = std::make_unique<modern_orca::CircularAgent>(
            enemy_id, enemy_pos, enemy_vel, ORCA_MAX_SPEED, enemy_radius);
        } else {
          // Update existing enemy agent
          agents_[enemy_id]->setPosition(enemy_pos);
          agents_[enemy_id]->setVelocity(enemy_vel);
          agents_[enemy_id]->setPreferredVelocity(enemy_vel);
          agents_[enemy_id]->collisionModel().setRadius(enemy_radius);
          agents_[enemy_id]->setMaxSpeed(ORCA_MAX_SPEED);
        }

        // Visualize enemy robot radius and velocity (same as RVO2)
        if (visualizer) {
          visualizer->circle()
            .radius(enemy_radius)
            .center(enemy_robot->pose.pos)
            .stroke("red", 0.2)
            .strokeWidth(10)
            .build();
          
          std::stringstream ss;
          ss << std::fixed << std::setprecision(2) << velocity_norm << "m/s";
          visualizer->text()
            .text(ss.str())
            .fontSize(50)
            .position(enemy_robot->pose.pos + Point(0, enemy_radius + 0.07))
            .textAnchor("middle")
            .fill("red", 0.5)
            .build();
        }
      } else {
        // Place unavailable enemy robots far away (same as RVO2)
        Vector2d far_position(20.0, 20.0);
        Vector2d zero_velocity(0.0, 0.0);
        
        if (agents_.find(enemy_id) == agents_.end()) {
          agents_[enemy_id] = std::make_unique<modern_orca::CircularAgent>(
            enemy_id, far_position, zero_velocity, ORCA_MAX_SPEED, ORCA_RADIUS);
        } else {
          agents_[enemy_id]->setPosition(far_position);
          agents_[enemy_id]->setVelocity(zero_velocity);
          agents_[enemy_id]->setPreferredVelocity(zero_velocity);
          agents_[enemy_id]->setMaxSpeed(ORCA_MAX_SPEED);
        }
      }
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
  auto start_time = std::chrono::high_resolution_clock::now();
  total_constraints_ = 0.0;
  
  crane_msgs::msg::RobotCommands result = original_commands;

  for (auto & command : result.robot_commands) {
    const auto robot_id = command.robot_id;

    if (agents_.find(robot_id) != agents_.end()) {
      auto & agent = *agents_[robot_id];

      // Apply constraint disable flags from local_planner_config for this robot
      applyConstraintFlags(command.local_planner_config);

      // Generate SSL constraints for this agent
      auto constraints =
        ssl_constraint_manager_->generateAllHalfPlanes(agent, ORCA_TIME_STEP);
      total_constraints_ += constraints.size();

      // Debug visualization for all constraints
      if (debug_visualize_constraints_) {
        visualizeConstraints(robot_id, constraints);
      }

      // Generate ORCA constraints with other agents (both friendly and enemy)
      // Only if collision avoidance is not disabled
      if (!command.local_planner_config.disable_collision_avoidance) {
        std::vector<modern_orca::CircularAgent*> other_agents;
        for (const auto & [other_id, other_agent] : agents_) {
          if (other_id != robot_id) {
            other_agents.push_back(other_agent.get());
          }
        }

        if (!other_agents.empty()) {
          modern_orca::ORCAConstraint<modern_orca::CircularAgent> orca_constraint(
            other_agents, ORCA_TIME_HORIZON);
          auto orca_half_planes = orca_constraint.generateHalfPlanes(agent, ORCA_TIME_STEP);
          
          // Debug visualization for ORCA lines
          if (debug_visualize_orca_lines_) {
            visualizeORCALines(robot_id, orca_half_planes);
          }
          
          constraints.insert(constraints.end(), orca_half_planes.begin(), orca_half_planes.end());
          total_constraints_ += orca_half_planes.size();
        }
      }

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
  
  // Calculate solve time
  auto end_time = std::chrono::high_resolution_clock::now();
  solve_time_ms_ = std::chrono::duration<double, std::milli>(end_time - start_time).count();
  
  // Show performance metrics if enabled
  if (debug_show_performance_metrics_) {
    visualizePerformanceMetrics();
  }
  
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

void ModernORCAPlanner::applyConstraintFlags(const crane_msgs::msg::LocalPlannerConfig & config)
{
  // Apply constraint disable flags from local_planner_config
  // Map LocalPlannerConfig flags to SSL constraint types
  ssl_constraint_manager_->setConstraintEnabled(
    modern_orca::SSLConstraintType::BALL_AVOIDANCE, 
    !config.disable_ball_avoidance);
    
  ssl_constraint_manager_->setConstraintEnabled(
    modern_orca::SSLConstraintType::PENALTY_AREA_AVOIDANCE, 
    !config.disable_goal_area_avoidance);
    
  ssl_constraint_manager_->setConstraintEnabled(
    modern_orca::SSLConstraintType::BALL_PLACEMENT_AVOIDANCE, 
    !config.disable_placement_avoidance);
    
  // Note: disable_collision_avoidance is handled separately in generateCommandsFromORCA
  // to control ORCA constraints directly
}

void ModernORCAPlanner::setConstraintLineup(
  const modern_orca::SSLConstraintManagerForCircularAgent::ConstraintLineup & lineup)
{
  ssl_constraint_manager_->setConstraintLineup(lineup);
  RCLCPP_INFO(node_.get_logger(), 
    "ModernORCAPlanner: Updated constraint lineup - Ball: %s, PenaltyArea: %s, Placement: %s, Referee: %s, Collision: %s",
    lineup.ball_avoidance_enabled ? "ON" : "OFF",
    lineup.penalty_area_avoidance_enabled ? "ON" : "OFF", 
    lineup.ball_placement_avoidance_enabled ? "ON" : "OFF",
    lineup.referee_command_enabled ? "ON" : "OFF",
    lineup.robot_collision_enabled ? "ON" : "OFF");
}

modern_orca::SSLConstraintManagerForCircularAgent::ConstraintLineup 
ModernORCAPlanner::getConstraintLineup() const
{
  return ssl_constraint_manager_->getConstraintLineup();
}

void ModernORCAPlanner::enableConstraint(modern_orca::SSLConstraintType type, bool enabled)
{
  ssl_constraint_manager_->setConstraintEnabled(type, enabled);
}

bool ModernORCAPlanner::isConstraintEnabled(modern_orca::SSLConstraintType type) const
{
  return ssl_constraint_manager_->isConstraintEnabled(type);
}

void ModernORCAPlanner::visualizeConstraints(
  uint32_t robot_id, const std::vector<modern_orca::HalfPlane> & constraints)
{
  if (!visualizer || !world_model) return;
  
  auto robot = world_model->getOurRobot(robot_id);
  if (!robot) return;
  
  // Visualize constraint half-planes as lines
  for (size_t i = 0; i < constraints.size(); ++i) {
    const auto & constraint = constraints[i];
    
    // Calculate line endpoints for visualization
    Vector2d normal = constraint.normal;
    Vector2d point = constraint.point;
    
    // Create perpendicular direction for line visualization
    Vector2d tangent(-normal.y(), normal.x());
    Point line_start = point + tangent * 2.0;  // 2m line length
    Point line_end = point - tangent * 2.0;
    
    // Color code constraints: SSL constraints in blue, ORCA constraints in green
    std::string color = (i < 10) ? "blue" : "green";  // Rough heuristic
    
    visualizer->line()
      .start(line_start)
      .end(line_end)
      .stroke(color, 0.3)
      .strokeWidth(5)
      .build();
      
    // Show constraint normal direction
    visualizer->line()
      .start(point)
      .end(point + normal * 0.5)
      .stroke(color, 0.7)
      .strokeWidth(8)
      .build();
  }
  
  // Show constraint count
  std::stringstream ss;
  ss << "Constraints: " << constraints.size();
  visualizer->text()
    .text(ss.str())
    .fontSize(40)
    .position(robot->pose.pos + Point(0.2, 0.2))
    .textAnchor("start")
    .fill("blue", 0.8)
    .build();
}

void ModernORCAPlanner::visualizeORCALines(
  uint32_t robot_id, const std::vector<modern_orca::HalfPlane> & orca_constraints)
{
  if (!visualizer || !world_model) return;
  
  auto robot = world_model->getOurRobot(robot_id);
  if (!robot) return;
  
  // Visualize ORCA lines specifically
  for (const auto & constraint : orca_constraints) {
    Vector2d normal = constraint.normal;
    Vector2d point = constraint.point;
    
    // Create perpendicular direction for ORCA line visualization
    Vector2d tangent(-normal.y(), normal.x());
    Point line_start = point + tangent * 1.5;  // 1.5m ORCA line length
    Point line_end = point - tangent * 1.5;
    
    visualizer->line()
      .start(line_start)
      .end(line_end)
      .stroke("orange", 0.6)
      .strokeWidth(6)
      .build();
      
    // Show velocity space constraint direction
    visualizer->line()
      .start(point)
      .end(point + normal * 0.3)
      .stroke("red", 0.8)
      .strokeWidth(10)
      .build();
  }
  
  // Show ORCA constraint count
  std::stringstream ss;
  ss << "ORCA: " << orca_constraints.size();
  visualizer->text()
    .text(ss.str())
    .fontSize(35)
    .position(robot->pose.pos + Point(0.2, -0.1))
    .textAnchor("start")
    .fill("orange", 0.8)
    .build();
}

void ModernORCAPlanner::visualizePerformanceMetrics()
{
  if (!visualizer || !debug_show_performance_metrics_) return;
  
  // Show performance metrics in a corner of the field
  Point metrics_pos(-4.0, 3.0);  // Top-left corner
  
  std::stringstream ss;
  ss << "ModernORCA Performance:\n";
  ss << "Agents: " << agents_.size() << "\n";
  ss << "Time Step: " << ORCA_TIME_STEP << "s\n";
  ss << "Max Speed: " << ORCA_MAX_SPEED << "m/s\n";
  ss << "Time Horizon: " << ORCA_TIME_HORIZON << "s\n";
  
  // Show timing metrics (if available)
  if (solve_time_ms_ > 0.0) {
    ss << "Solve Time: " << std::fixed << std::setprecision(2) << solve_time_ms_ << "ms\n";
    ss << "Constraints/Agent: " << (total_constraints_ / std::max(1.0, static_cast<double>(agents_.size()))) << "\n";
  }
  
  auto lineup = getConstraintLineup();
  ss << "Constraints Active:\n";
  ss << "  Ball: " << (lineup.ball_avoidance_enabled ? "ON" : "OFF") << "\n";
  ss << "  PenaltyArea: " << (lineup.penalty_area_avoidance_enabled ? "ON" : "OFF") << "\n";
  ss << "  Placement: " << (lineup.ball_placement_avoidance_enabled ? "ON" : "OFF") << "\n";
  ss << "  Collision: " << (lineup.robot_collision_enabled ? "ON" : "OFF");
  
  visualizer->text()
    .text(ss.str())
    .fontSize(30)
    .position(metrics_pos)
    .textAnchor("start")
    .fill("white", 0.9)
    .build();
}

Point ModernORCAPlanner::getCurrentPosition(const crane_msgs::msg::RobotCommand & command) const
{
  // Use robot feedback position if available, otherwise fall back to command position
  if (auto feedback = std::find_if(
        latest_feedback.feedback.begin(), latest_feedback.feedback.end(),
        [&](const auto & f) { return f.robot_id == command.robot_id; });
      feedback != latest_feedback.feedback.end()) {
    return Point(feedback->odom[0], feedback->odom[1]);
  } else {
    return Point(command.current_pose.x, command.current_pose.y);
  }
}

}  // namespace crane
