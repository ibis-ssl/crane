// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/modern_orca_planner.hpp"

#include <algorithm>
#include <chrono>
#include <crane_local_planner/visualization_helpers.hpp>
#include <iomanip>
#include <robocup_ssl_msgs/msg/referee.hpp>
#include <sstream>

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

  node.declare_parameter("stop_state_max_velocity", STOP_STATE_MAX_VELOCITY);
  STOP_STATE_MAX_VELOCITY = node.get_parameter("stop_state_max_velocity").as_double();

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
  initial_lineup.ball_avoidance_enabled =
    node.get_parameter("constraint_ball_avoidance_enabled").as_bool();
  initial_lineup.penalty_area_avoidance_enabled =
    node.get_parameter("constraint_penalty_area_avoidance_enabled").as_bool();
  initial_lineup.ball_placement_avoidance_enabled =
    node.get_parameter("constraint_ball_placement_avoidance_enabled").as_bool();
  initial_lineup.referee_command_enabled =
    node.get_parameter("constraint_referee_command_enabled").as_bool();
  initial_lineup.robot_collision_enabled =
    node.get_parameter("constraint_robot_collision_enabled").as_bool();

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

  RCLCPP_INFO(
    node.get_logger(), "ModernORCAPlanner initialized with SSL constraint manager and ORCA solver");
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
  for (const auto & raw_command : commands.robot_commands) {
    crane_msgs::msg::RobotCommand command = raw_command;
    const auto robot_id = command.robot_id;

    // Get current position (with robot feedback integration)
    Point current_position = getCurrentPosition(command);

    // Convert current position and velocity to modern_orca types
    Vector2 position(current_position.x(), current_position.y());
    Vector2 velocity(command.current_velocity.x, command.current_velocity.y);
    Vector2 preferred_velocity(0.0, 0.0);  // Will be set based on command type

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
          preferred_velocity = Vector2(vel_target.target_vx, vel_target.target_vy);
        }
        break;
      }
      case crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE: {
        if (!command.polar_velocity_target_mode.empty()) {
          const auto & polar_target = command.polar_velocity_target_mode.front();
          const double v_r = polar_target.target_velocity_r;
          const double v_theta = polar_target.target_velocity_theta;
          preferred_velocity = Vector2(v_r * cos(v_theta), v_r * sin(v_theta));
        }
        break;
      }
    }

    // 速度に基づいて動的半径を計算（RVO2と同じ）
    double velocity_norm = velocity.norm();
    double dynamic_radius = ORCA_RADIUS + velocity_norm * 0.1;  // Base radius + velocity factor

    // Use configured max speed or command override
    command.local_planner_config.max_velocity_factors.emplace_back(
      crane_msgs::msg::NamedFloat().set__name("ORCA_MAX_SPEED").set__value(ORCA_MAX_SPEED));

    // Apply referee command velocity limits
    if (
      world_model && world_model->getMsg().play_situation.referee_raw.command.value ==
                       robocup_ssl_msgs::msg::RefereeCommand::STOP) {
      command.local_planner_config.max_velocity_factors.emplace_back(
        crane_msgs::msg::NamedFloat()
          .set__name("ModernORCAPlanner STOP制限")
          .set__value(STOP_STATE_MAX_VELOCITY));
    }

    double max_speed = resolveMaxVelocityFactors(command, MAX_VEL);

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
        drawRobotRadiusWithSpeed(
          visualizer, robot->pose.pos, dynamic_radius, velocity_norm, "yellow");
      }
    }
  }

  // Add enemy robot agents
  if (world_model) {
    for (const auto & enemy_robot : world_model->theirs().robots) {
      const auto enemy_id = enemy_robot->id + 20;  // Enemy robots: 20-39

      if (enemy_robot->available()) {
        // Convert enemy robot position and velocity to modern_orca types
        Vector2 enemy_pos(enemy_robot->pose.pos.x(), enemy_robot->pose.pos.y());
        Vector2 enemy_vel(enemy_robot->vel.linear.x(), enemy_robot->vel.linear.y());

        // 速度に基づいて動的半径を計算（RVO2と同じ）
        double velocity_norm = enemy_vel.norm();
        double enemy_radius = ORCA_RADIUS + velocity_norm * 0.1;  // Base radius + velocity factor

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
          drawRobotRadiusWithSpeed(
            visualizer, enemy_robot->pose.pos, enemy_radius, velocity_norm, "red");
        }
      } else {
        // Place unavailable enemy robots far away (same as RVO2)
        Vector2 far_position(20.0, 20.0);
        Vector2 zero_velocity(0.0, 0.0);

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
    const auto & referee_command = world_model->getMsg().play_situation.referee_raw.command;
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
      auto constraints = ssl_constraint_manager_->generateAllHalfPlanes(agent, ORCA_TIME_STEP);
      total_constraints_ += constraints.size();

      // Debug visualization for all constraints
      if (debug_visualize_constraints_) {
        visualizeConstraints(robot_id, constraints);
      }

      // Generate ORCA constraints with other agents (both friendly and enemy)
      // Only if collision avoidance is not disabled
      if (!command.local_planner_config.disable_collision_avoidance) {
        std::vector<modern_orca::CircularAgent *> other_agents;
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
      if (
        original_commands.robot_commands[&command - &result.robot_commands[0]].control_mode ==
        crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE) {
        command.local_planner_config.final_planned_max_acceleration.set__name("modern_orca")
          .set__value(final_planned_acceleration_);
        command.local_planner_config.final_planned_max_velocity.set__name("modern_orca")
          .set__value(final_planned_max_velocity_);
      }

      // Convert back to ROS message format
      command.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
      command.polar_velocity_target_mode.clear();
      command.polar_velocity_target_mode.reserve(1);

      crane_msgs::msg::PolarVelocityTargetMode target;
      target.target_velocity_r = preferred_vel.norm();
      target.target_velocity_theta =
        std::atan2(preferred_vel.y(), preferred_vel.x()) + theta_offset;

      // 効率的な加速のための回転制御
      if (command.local_planner_config.enable_rotation_stop_on_accel) {
        double move_angle = std::atan2(preferred_vel.y(), preferred_vel.x());
        auto robot = world_model->getOurRobot(robot_id);
        if (robot) {
          double angle_diff = getAngleDiff(robot->pose.theta, move_angle);

          constexpr double ANGLE_THRESHOLD = 15.0 * M_PI / 180.0;  // 15度
          bool is_forward_or_backward =
            (std::abs(angle_diff) <= ANGLE_THRESHOLD) ||                 // 前方
            (std::abs(std::abs(angle_diff) - M_PI) <= ANGLE_THRESHOLD);  // 後方

          double current_speed = robot->vel.linear.norm();
          double target_speed = preferred_vel.norm();
          double max_speed = agent.maxSpeed();

          bool is_accelerating = current_speed < target_speed;
          bool is_low_speed = current_speed <= max_speed * 0.5;

          // 加速初期段階かつ前後方向に向いている場合、回転を停止
          if (is_forward_or_backward && is_low_speed && is_accelerating && target_speed > 0.01) {
            command.omega_limit = 0.0;
          }
        }
      }

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

Vector2 ModernORCAPlanner::calculateTrapezoidalVelocityProfile(
  const crane_msgs::msg::RobotCommand & raw_command, const Point & current_position)
{
  crane_msgs::msg::RobotCommand command = raw_command;
  if (command.position_target_mode.empty()) {
    return Vector2(0.0, 0.0);
  }

  const auto & target_config = command.position_target_mode.front();
  Point target_pos(target_config.target_x, target_config.target_y);

  // NaN値検証とフォールバック処理
  if (std::isnan(target_pos.x()) || std::isnan(target_pos.y())) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "[ModernORCAPlanner] NaN detected in target_pos for robot %d: target_pos(%f, %f), using "
      "current position as fallback",
      static_cast<int>(raw_command.robot_id), target_pos.x(), target_pos.y());
    return Vector2(0.0, 0.0);  // 現在位置を維持（速度0）
  }

  if (std::isnan(current_position.x()) || std::isnan(current_position.y())) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "[ModernORCAPlanner] NaN detected in current_pos for robot %d: current_pos(%f, %f), skipping",
      static_cast<int>(raw_command.robot_id), current_position.x(), current_position.y());
    return Vector2(0.0, 0.0);  // 速度0を返す
  }

  Point position_diff = target_pos - current_position;

  // Check if within position tolerance first
  if (isWithinPositionTolerance(command, current_position)) {
    return Vector2(0.0, 0.0);
  }

  // Calculate target velocity direction
  Point target_vel = position_diff.normalized() * MAX_VEL;

  // Apply square root velocity scaling for deceleration
  // v = sqrt(2 * a * x) for each axis
  double max_acc = resolveMaxAccelerationFactors(command, ACCELERATION);

  // Get previous velocity for acceleration limiting
  double pre_vel = getPreviousVelocity(command.robot_id);

  // Velocity limit by deceleration distance
  double max_vel_by_decel = std::sqrt(2.0 * max_acc * position_diff.norm());
  command.local_planner_config.max_velocity_factors.emplace_back(
    crane_msgs::msg::NamedFloat().set__name("max_vel_by_decel").set__value(max_vel_by_decel));

  // Velocity limit by acceleration constraint
  double max_vel_by_acc = pre_vel + max_acc * ORCA_TIME_STEP;
  command.local_planner_config.max_velocity_factors.emplace_back(
    crane_msgs::msg::NamedFloat().set__name("max_vel_by_acc").set__value(max_vel_by_acc));

  // Combine all velocity limits
  double max_vel = resolveMaxVelocityFactors(command, MAX_VEL);

  // Apply referee command velocity limits
  if (
    world_model && world_model->getMsg().play_situation.referee_raw.command.value ==
                     robocup_ssl_msgs::msg::RefereeCommand::STOP) {
    max_vel = std::min(max_vel, STOP_STATE_MAX_VELOCITY);
  }

  // Store final planned values (these will be set in the calling function)
  final_planned_acceleration_ = max_acc;
  final_planned_max_velocity_ = max_vel;

  // Normalize and scale to maximum velocity
  target_vel = target_vel.normalized() * max_vel;

  // Apply terminal velocity constraint
  if (target_vel.norm() < command.local_planner_config.terminal_velocity) {
    target_vel = target_vel.normalized() * command.local_planner_config.terminal_velocity;
  }

  return Vector2(target_vel.x(), target_vel.y());
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
  if (
    command.control_mode != crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE ||
    command.position_target_mode.empty()) {
    return false;
  }

  const auto & target_config = command.position_target_mode.front();
  double distance = std::hypot(
    target_config.target_x - current_position.x(), target_config.target_y - current_position.y());

  // 明示的な位置許容範囲をチェック
  if (distance < target_config.position_tolerance) {
    return true;
  }

  // 終端速度が0の場合のデフォルト許容範囲
  if (
    command.local_planner_config.terminal_velocity == 0.0 &&
    target_config.position_tolerance == 0.0 && distance < 0.03) {
    return true;
  }

  return false;
}

void ModernORCAPlanner::applyConstraintFlags(const crane_msgs::msg::LocalPlannerConfig & config)
{
  // local_planner_configから制約無効化フラグを適用
  // LocalPlannerConfigフラグをSSL制約タイプにマップ
  ssl_constraint_manager_->setConstraintEnabled(
    modern_orca::SSLConstraintType::BALL_AVOIDANCE, !config.disable_ball_avoidance);

  ssl_constraint_manager_->setConstraintEnabled(
    modern_orca::SSLConstraintType::PENALTY_AREA_AVOIDANCE, !config.disable_goal_area_avoidance);

  ssl_constraint_manager_->setConstraintEnabled(
    modern_orca::SSLConstraintType::BALL_PLACEMENT_AVOIDANCE, !config.disable_placement_avoidance);

  // Note: disable_collision_avoidance is handled separately in generateCommandsFromORCA
  // to control ORCA constraints directly
}

void ModernORCAPlanner::setConstraintLineup(
  const modern_orca::SSLConstraintManagerForCircularAgent::ConstraintLineup & lineup)
{
  ssl_constraint_manager_->setConstraintLineup(lineup);
  RCLCPP_INFO(
    node_.get_logger(),
    "ModernORCAPlanner: Updated constraint lineup - Ball: %s, PenaltyArea: %s, Placement: %s, "
    "Referee: %s, Collision: %s",
    lineup.ball_avoidance_enabled ? "ON" : "OFF",
    lineup.penalty_area_avoidance_enabled ? "ON" : "OFF",
    lineup.ball_placement_avoidance_enabled ? "ON" : "OFF",
    lineup.referee_command_enabled ? "ON" : "OFF", lineup.robot_collision_enabled ? "ON" : "OFF");
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
    Vector2 normal = constraint.normal;
    Vector2 point = constraint.point;

    // Create perpendicular direction for line visualization
    Vector2 tangent(-normal.y(), normal.x());
    Point line_start = point + tangent * 2.0;  // 2m line length
    Point line_end = point - tangent * 2.0;

    // Color code constraints: SSL constraints in blue, ORCA constraints in green
    std::string color = (i < 10) ? "blue" : "green";  // Rough heuristic

    visualizer->drawLine(line_start, line_end, color, 5, 0.3);

    // 制約法線方向を表示
    visualizer->drawLine(point, point + normal * 0.5, color, 8, 0.7);
  }

  // 制約数を表示
  std::stringstream ss;
  ss << "Constraints: " << constraints.size();
  visualizer->drawText(robot->pose.pos + Point(0.2, 0.2), ss.str(), "blue", 40, "start");
}

void ModernORCAPlanner::visualizeORCALines(
  uint32_t robot_id, const std::vector<modern_orca::HalfPlane> & orca_constraints)
{
  if (!visualizer || !world_model) return;

  auto robot = world_model->getOurRobot(robot_id);
  if (!robot) return;

  // ORCA線を具体的に可視化
  for (const auto & constraint : orca_constraints) {
    Vector2 normal = constraint.normal;
    Vector2 point = constraint.point;

    // ORCA線可視化のための垂直方向を作成
    Vector2 tangent(-normal.y(), normal.x());
    Point line_start = point + tangent * 1.5;  // 1.5m ORCA線の長さ
    Point line_end = point - tangent * 1.5;

    visualizer->drawLine(line_start, line_end, "orange", 6, 0.6);

    // 速度空間制約方向を表示
    visualizer->drawLine(point, point + normal * 0.3, "red", 10, 0.8);
  }

  // ORCA制約数を表示
  std::stringstream ss;
  ss << "ORCA: " << orca_constraints.size();
  visualizer->drawText(robot->pose.pos + Point(0.2, -0.1), ss.str(), "orange", 35, "start");
}

void ModernORCAPlanner::visualizePerformanceMetrics()
{
  if (!visualizer || !debug_show_performance_metrics_) return;

  // フィールドの角にパフォーマンスメトリクスを表示
  Point metrics_pos(-4.0, 3.0);  // 左上角

  std::stringstream ss;
  ss << "ModernORCA Performance:\n";
  ss << "Agents: " << agents_.size() << "\n";
  ss << "Time Step: " << ORCA_TIME_STEP << "s\n";
  ss << "Max Speed: " << ORCA_MAX_SPEED << "m/s\n";
  ss << "Time Horizon: " << ORCA_TIME_HORIZON << "s\n";

  // タイミングメトリクスを表示（利用可能な場合）
  if (solve_time_ms_ > 0.0) {
    ss << "Solve Time: " << std::fixed << std::setprecision(2) << solve_time_ms_ << "ms\n";
    ss << "Constraints/Agent: "
       << (total_constraints_ / std::max(1.0, static_cast<double>(agents_.size()))) << "\n";
  }

  auto lineup = getConstraintLineup();
  ss << "Constraints Active:\n";
  ss << "  Ball: " << (lineup.ball_avoidance_enabled ? "ON" : "OFF") << "\n";
  ss << "  PenaltyArea: " << (lineup.penalty_area_avoidance_enabled ? "ON" : "OFF") << "\n";
  ss << "  Placement: " << (lineup.ball_placement_avoidance_enabled ? "ON" : "OFF") << "\n";
  ss << "  Collision: " << (lineup.robot_collision_enabled ? "ON" : "OFF");

  visualizer->drawText(metrics_pos, ss.str(), "white", 30, "start");
}

Point ModernORCAPlanner::getCurrentPosition(const crane_msgs::msg::RobotCommand & command) const
{
  // ロボットフィードバック位置が利用可能な場合は使用、そうでなければコマンド位置にフォールバック
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
