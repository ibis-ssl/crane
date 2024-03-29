// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/rvo_planner.hpp"

// cspell: ignore OBST

namespace crane
{
RVOPlanner::RVOPlanner(rclcpp::Node & node)
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
void RVOPlanner::reflectWorldToRVOSim(
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

    float target_x = command.target_x.front();
    float target_y = command.target_y.front();

    if (command.motion_mode_enable) {
      // 速度制御モードの場合：速度司令をそのままRVOのpreferred velocityとして設定する
      rvo_sim->setAgentPrefVelocity(command.robot_id, RVO::Vector2(target_x, target_y));
    } else {
      // 位置制御モードの場合：目標位置方向に移動する速度ベクトルを
      // RVOのpreferred velocityとして設定する
      const auto pos = robot->pose.pos;
      auto diff_pos = Point(target_x - pos.x(), target_y - pos.y());

      // 台形加速制御
      // 2ax = v^2 - v0^2
      // v^2 - 2ax = v0^2
      // v0 = sqrt(v^2 - 2ax)
      double max_speed_for_stop =
        std::sqrt(0 * 0 - 2.0 * (-RVO_TRAPEZOIDAL_MAX_ACC) * diff_pos.norm());
      double max_speed_for_acc =
        robot->vel.linear.norm() + RVO_TRAPEZOIDAL_MAX_ACC / RVO_TRAPEZOIDAL_FRAME_RATE;

      double target_speed = std::min(
        {max_speed_for_acc, max_speed_for_stop, static_cast<double>(RVO_TRAPEZOIDAL_MAX_SPEED)});
      Velocity vel;
      double dist = diff_pos.norm();
      vel << target_speed / dist * diff_pos.x(), target_speed / dist * diff_pos.y();

      if (command.robot_id == 3) {
        std::cout << "current_robot: " << int(command.robot_id) << std::endl;
        std::cout << "from: " << pos.x() << ", " << pos.y() << std::endl;
        std::cout << "to: " << target_x << ", " << target_y << std::endl;
        std::cout << "diff_pos: " << diff_pos.x() << ", " << diff_pos.y() << std::endl;
        std::cout << "target_speed: " << target_speed << std::endl;
        std::cout << "target_vel: " << std::fixed << std::setprecision(5) << vel.x() << ", "
                  << vel.y() << std::endl;
      }
      rvo_sim->setAgentPrefVelocity(command.robot_id, RVO::Vector2(vel.x(), vel.y()));
    }
  }

  if (add_ball) {
    rvo_sim->setAgentPosition(
      40, RVO::Vector2(world_model->ball.pos.x(), world_model->ball.pos.y()));
    rvo_sim->setAgentPrefVelocity(40, RVO::Vector2(0.f, 0.f));
  } else {
    rvo_sim->setAgentPosition(40, RVO::Vector2(20.0f, 20.0f));
    rvo_sim->setAgentPrefVelocity(40, RVO::Vector2(0.f, 0.f));
  }

  for (const auto & enemy_robot : world_model->theirs.robots) {
    if (enemy_robot->available) {
      const auto & pos = enemy_robot->pose.pos;
      const auto & vel = enemy_robot->vel.linear;
      rvo_sim->setAgentPosition(enemy_robot->id + 20, RVO::Vector2(pos.x(), pos.y()));
      rvo_sim->setAgentPrefVelocity(enemy_robot->id + 20, RVO::Vector2(vel.x(), vel.y()));
    } else {
      rvo_sim->setAgentPosition(enemy_robot->id + 20, RVO::Vector2(20.f, 20.f));
      rvo_sim->setAgentPrefVelocity(enemy_robot->id + 20, RVO::Vector2(0.f, 0.f));
    }
  }
}
crane_msgs::msg::RobotCommands RVOPlanner::extractRobotCommandsFromRVOSim(
  const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model)
{
  crane_msgs::msg::RobotCommands commands = msg;
  for (size_t i = 0; i < msg.robot_commands.size(); i++) {
    const auto & original_command = msg.robot_commands.at(i);
    const auto & robot = world_model->getOurRobot(original_command.robot_id);
    crane_msgs::msg::RobotCommand command = original_command;
    // RVOシミュレータの出力をコピーする
    // NOTE: RVOシミュレータは角度を扱わないので角度はそのまま

    // RVOシミュレータの出力は速度なので，速度制御モードにする
    command.motion_mode_enable = true;
    auto vel = rvo_sim->getAgentVelocity(original_command.robot_id);
    if (original_command.robot_id == 3) {
      //      std::cout << "robot_id " << int(original_command.robot_id) << std::endl;
      //      std::cout << "vel : " << vel.x() << " " << vel.y() << std::endl;
    }
    command.target_velocity.x = vel.x();
    command.target_velocity.y = vel.y();
    command.target_x.clear();
    command.target_y.clear();

    commands.robot_commands.at(i) = command;
  }
  return commands;
}
crane_msgs::msg::RobotCommands RVOPlanner::calculateRobotCommand(
  const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model)
{
  reflectWorldToRVOSim(msg, world_model);
  // RVOシミュレータ更新
  rvo_sim->doStep();
  return extractRobotCommandsFromRVOSim(msg, world_model);
}
}  // namespace crane
