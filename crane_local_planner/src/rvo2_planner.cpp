// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/rvo2_planner.hpp"

// cspell: ignore OBST

namespace crane
{
RVO2Planner::RVO2Planner(rclcpp::Node & node)
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

  visualizer = std::make_shared<ConsaiVisualizerWrapper>(node, "rvo2_local_planner");

  rvo_sim = std::make_unique<RVO::RVOSimulator>(
    RVO_TIME_STEP, RVO_NEIGHBOR_DIST, RVO_MAX_NEIGHBORS, RVO_TIME_HORIZON, RVO_TIME_HORIZON_OBST,
    RVO_RADIUS, RVO_MAX_SPEED);

  // TODO(HansRobo): add goal area as obstacles

  // TODO(HansRobo): add external area as obstacles
  // friend robots -> 0~19
  // enemy robots -> 20~39
  // ball 40
  for (int i = 0; i < 41; i++) {
    rvo_sim->addAgent(RVO::Vector2(20.0f, 20.0f));
  }
}
void RVO2Planner::reflectWorldToRVOSim(
  const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model)
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

        target_vel *= command.local_planner_config.max_acceleration;

        double max_vel =
          std::min(command.local_planner_config.max_velocity, static_cast<float>(MAX_VEL));
        if (target_vel.norm() > max_vel) {
          target_vel = target_vel.normalized() * max_vel;
        }
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
  const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model)
{
  crane_msgs::msg::RobotCommands commands = msg;
  for (size_t i = 0; i < msg.robot_commands.size(); i++) {
    const auto & original_command = msg.robot_commands.at(i);
    const auto & robot = world_model->getOurRobot(original_command.robot_id);
    crane_msgs::msg::RobotCommand command = original_command;
    // RVOシミュレータの出力をコピーする
    // NOTE: RVOシミュレータは角度を扱わないので角度はそのまま

    if (command.control_mode != crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE) {
      command.control_mode = crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE;
      command.simple_velocity_target_mode.clear();
    }

    crane_msgs::msg::SimpleVelocityTargetMode target;
    auto vel = toPoint(rvo_sim->getAgentVelocity(original_command.robot_id));
    // if (original_command.robot_id == 3) {
    //   //      std::cout << "robot_id " << int(original_command.robot_id) << std::endl;
    //   //      std::cout << "vel : " << vel.x() << " " << vel.y() << std::endl;
    // }
    target.target_vx = vel.x();
    target.target_vy = vel.y();
    command.simple_velocity_target_mode.push_back(target);

    commands.robot_commands.at(i) = command;
  }
  return commands;
}
crane_msgs::msg::RobotCommands RVO2Planner::calculateRobotCommand(
  const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model)
{
  reflectWorldToRVOSim(msg, world_model);
  // RVOシミュレータ更新
  rvo_sim->doStep();
  return extractRobotCommandsFromRVOSim(msg, world_model);
}
}  // namespace crane
