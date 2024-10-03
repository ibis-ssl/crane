// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/rvo2_planner.hpp"

#include <boost/stacktrace.hpp>

// cspell: ignore OBST

namespace crane
{
RVO2Planner::RVO2Planner(rclcpp::Node & node) : LocalPlannerBase("rvo2_local_planner", node)
{
  node.declare_parameter("rvo_time_step", RVO_TIME_STEP);
  RVO_TIME_STEP = node.get_parameter("rvo_time_step").as_double();
  node.declare_parameter("rvo_neighbor_dist", RVO_NEIGHBOR_DIST);
  RVO_NEIGHBOR_DIST = node.get_parameter("rvo_neighbor_dist").as_double();
  node.declare_parameter("rvo_max_neighbors", RVO_MAX_NEIGHBORS);
  RVO_MAX_NEIGHBORS = node.get_parameter("rvo_max_neighbors").as_int();
  node.declare_parameter("rvo_time_horizon", RVO_TIME_HORIZON);
  RVO_TIME_HORIZON = node.get_parameter("rvo_time_horizon").as_double();
  node.declare_parameter("rvo_time_horizon_obst", RVO_TIME_HORIZON_OBST);
  RVO_TIME_HORIZON_OBST = node.get_parameter("rvo_time_horizon_obst").as_double();
  node.declare_parameter("rvo_radius", RVO_RADIUS);
  RVO_RADIUS = node.get_parameter("rvo_radius").as_double();
  node.declare_parameter("rvo_max_speed", RVO_MAX_SPEED);
  RVO_MAX_SPEED = node.get_parameter("rvo_max_speed").as_double();
  node.declare_parameter("rvo_trapezoidal_max_acc", RVO_TRAPEZOIDAL_MAX_ACC);
  RVO_TRAPEZOIDAL_MAX_ACC = node.get_parameter("rvo_trapezoidal_max_acc").as_double();
  node.declare_parameter("rvo_trapezoidal_frame_rate", RVO_TRAPEZOIDAL_FRAME_RATE);
  RVO_TRAPEZOIDAL_FRAME_RATE = node.get_parameter("rvo_trapezoidal_frame_rate").as_double();
  node.declare_parameter("rvo_trapezoidal_max_speed", RVO_TRAPEZOIDAL_MAX_SPEED);
  RVO_TRAPEZOIDAL_MAX_SPEED = node.get_parameter("rvo_trapezoidal_max_speed").as_double();

  node.declare_parameter("max_vel", MAX_VEL);
  MAX_VEL = node.get_parameter("max_vel").as_double();

  node.declare_parameter("max_acc", ACCELERATION);
  ACCELERATION = node.get_parameter("max_acc").as_double();

  node.declare_parameter("max_decel", DECELERATION);
  DECELERATION = node.get_parameter("max_decel").as_double();

  rvo_sim = std::make_unique<RVO::RVOSimulator>(
    RVO_TIME_STEP, RVO_NEIGHBOR_DIST, RVO_MAX_NEIGHBORS, RVO_TIME_HORIZON, RVO_TIME_HORIZON_OBST,
    RVO_RADIUS, RVO_MAX_SPEED);

  // friend robots -> 0~19
  // enemy robots -> 20~39
  // ball 40
  for (int i = 0; i < 41; i++) {
    rvo_sim->addAgent(RVO::Vector2(20.0f, 20.0f));
  }
}

void RVO2Planner::reflectWorldToRVOSim(const crane_msgs::msg::RobotCommands & msg)
{
  bool add_ball = true;
  // 味方ロボット：RVO内の位置・速度（＝進みたい方向）の更新
  for (const auto & command : msg.robot_commands) {
    rvo_sim->setAgentPosition(
      command.robot_id, RVO::Vector2(command.current_pose.x, command.current_pose.y));
    rvo_sim->setAgentPrefVelocity(command.robot_id, RVO::Vector2(0.f, 0.f));

    auto robot = world_model->getOurRobot(command.robot_id);
    if (robot->available && command.local_planner_config.disable_collision_avoidance) {
      add_ball = false;
    }

    switch (command.control_mode) {
      case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE: {
        Velocity target_vel;
        target_vel << (command.position_target_mode.front().target_x - command.current_pose.x),
          command.position_target_mode.front().target_y - command.current_pose.y;

        double pre_vel = [&]() {
          if (auto it = std::find_if(
                pre_commands.robot_commands.begin(), pre_commands.robot_commands.end(),
                [&](const auto & c) { return c.robot_id == command.robot_id; });
              it != pre_commands.robot_commands.end()) {
            return static_cast<double>(std::hypot(
              it->simple_velocity_target_mode.front().target_vx,
              it->simple_velocity_target_mode.front().target_vy));
          } else {
            // 履歴が見つからなければ0
            return 0.0;
          }
        }();

        double deceleration = std::min(
          DECELERATION, static_cast<double>(command.local_planner_config.max_deceleration));
        double acceleration = std::min(
          ACCELERATION, static_cast<double>(command.local_planner_config.max_acceleration));

        // v^2 - v0^2 = 2ax
        // v = sqrt(v0^2 + 2ax)
        // v0 = 0, x = diff(=target_vel)
        // v = sqrt(2ax)
        double max_vel_by_decel = std::sqrt(2.0 * deceleration * target_vel.norm());

        // v = v0 + at
        double max_vel_by_acc = pre_vel + acceleration * RVO_TIME_STEP;

        double max_vel =
          std::min(static_cast<double>(command.local_planner_config.max_velocity), MAX_VEL);
        max_vel = std::min(max_vel, max_vel_by_decel);
        max_vel = std::min(max_vel, max_vel_by_acc);

        target_vel = target_vel.normalized() * max_vel;

        if (target_vel.norm() < command.local_planner_config.terminal_velocity) {
          target_vel = target_vel.normalized() * command.local_planner_config.terminal_velocity;
        }
        rvo_sim->setAgentPrefVelocity(command.robot_id, toRVO(target_vel));
        break;
      }
      case crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE: {
        rvo_sim->setAgentPrefVelocity(
          command.robot_id, RVO::Vector2(
                              command.simple_velocity_target_mode.front().target_vx,
                              command.simple_velocity_target_mode.front().target_vy));
        break;
      }
      default: {
        std::stringstream what;
        what << "Unsupported control mode: " << command.control_mode;
        what << ", expected: POSITION_TARGET_MODE, SIMPLE_VELOCITY_TARGET_MODE";
        throw std::runtime_error(what.str());
      }
    }
  }

  if (add_ball) {
    rvo_sim->setAgentPosition(40, toRVO(world_model->ball.pos));
    rvo_sim->setAgentPrefVelocity(40, RVO::Vector2(0.f, 0.f));
  } else {
    rvo_sim->setAgentPosition(40, RVO::Vector2(20.0f, 20.0f));
    rvo_sim->setAgentPrefVelocity(40, RVO::Vector2(0.f, 0.f));
  }

  for (const auto & enemy_robot : world_model->theirs.robots) {
    if (enemy_robot->available) {
      const auto & pos = enemy_robot->pose.pos;
      const auto & vel = enemy_robot->vel.linear;
      rvo_sim->setAgentPosition(enemy_robot->id + 20, toRVO(pos));
      rvo_sim->setAgentPrefVelocity(enemy_robot->id + 20, toRVO(vel));
    } else {
      rvo_sim->setAgentPosition(enemy_robot->id + 20, RVO::Vector2(20.f, 20.f));
      rvo_sim->setAgentPrefVelocity(enemy_robot->id + 20, RVO::Vector2(0.f, 0.f));
    }
  }
}
crane_msgs::msg::RobotCommands RVO2Planner::extractRobotCommandsFromRVOSim(
  const crane_msgs::msg::RobotCommands & msg)
{
  crane_msgs::msg::RobotCommands commands;
  for (const auto & original_command : msg.robot_commands) {
    const auto & robot = world_model->getOurRobot(original_command.robot_id);
    crane_msgs::msg::RobotCommand command = original_command;
    // RVOシミュレータの出力をコピーする
    // NOTE: RVOシミュレータは角度を扱わないので角度はそのまま

    command.control_mode = crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE;
    command.simple_velocity_target_mode.clear();
    command.simple_velocity_target_mode.reserve(1);

    crane_msgs::msg::SimpleVelocityTargetMode target;
    auto vel = toPoint(rvo_sim->getAgentVelocity(original_command.robot_id));
    target.target_vx = vel.x();
    target.target_vy = vel.y();
    command.simple_velocity_target_mode.push_back(target);

    commands.robot_commands.emplace_back(command);
  }

  pre_commands = commands;
  return commands;
}
crane_msgs::msg::RobotCommands RVO2Planner::calculateRobotCommand(
  const crane_msgs::msg::RobotCommands & msg)
{
  crane_msgs::msg::RobotCommands commands = msg;
  overrideTargetPosition(commands);
  reflectWorldToRVOSim(commands);
  // RVOシミュレータ更新
  rvo_sim->doStep();
  return extractRobotCommandsFromRVOSim(commands);
}

void RVO2Planner::overrideTargetPosition(crane_msgs::msg::RobotCommands & msg)
{
  for (auto & command : msg.robot_commands) {
    if (command.control_mode == crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE) {
      Point target_pos;
      target_pos << command.position_target_mode.front().target_x,
        command.position_target_mode.front().target_y;
      if (not command.local_planner_config.disable_goal_area_avoidance) {
        bool is_in_our_penalty_area = isInBox(world_model->getOurPenaltyArea(), target_pos);
        bool is_in_their_penalty_area = isInBox(world_model->getTheirPenaltyArea(), target_pos);
        if (is_in_our_penalty_area or is_in_their_penalty_area) {
          // ペナルティエリア内にいる場合は、ペナルティエリアの外に出るようにする
          const Point goal_pos = [&]() {
            if (is_in_our_penalty_area) {
              return world_model->getOurGoalCenter();
            } else {
              return world_model->getTheirGoalCenter();
            }
          }();

          // ゴールの後ろに回り込んだ場合は、ゴールの前に出るようにする
          if (std::abs(target_pos.x()) > world_model->field_size.x() / 2.0) {
            target_pos.x() = std::copysign(world_model->field_size.x() / 2.0, target_pos.x());
          }

          // 目標点をペナルティエリアの外に出るようにする
          while ([&]() {
            if (is_in_our_penalty_area) {
              return isInBox(world_model->getOurPenaltyArea(), target_pos);
            } else {
              return isInBox(world_model->getTheirPenaltyArea(), target_pos);
            }
          }()) {
            target_pos +=
              (target_pos - goal_pos).normalized() * 0.05;  // ゴールから5cmずつ離れていく
          }
        }
      }

      command.position_target_mode.front().target_x = target_pos.x();
      command.position_target_mode.front().target_y = target_pos.y();
    }
  }
}
}  // namespace crane
