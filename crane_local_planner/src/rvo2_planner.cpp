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
RVO2Planner::RVO2Planner(rclcpp::Node & node)
: LocalPlannerBase("rvo2_local_planner", node),
  deceleration_factor("deceleration_factor", node, 1.5),
  p_gain("p_gain", node, 4.0),
  i_gain("i_gain", node, 0.0),
  d_gain("d_gain", node, 0.0)
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

  p_gain.callback = [&](double value) {
    for (auto & controller : vx_controllers) {
      controller.setGain(value, i_gain.getValue(), d_gain.getValue());
    }
    for (auto & controller : vy_controllers) {
      controller.setGain(value, i_gain.getValue(), d_gain.getValue());
    }
  };

  i_gain.callback = [&](double value) {
    for (auto & controller : vx_controllers) {
      controller.setGain(p_gain.getValue(), value, d_gain.getValue());
    }
    for (auto & controller : vy_controllers) {
      controller.setGain(p_gain.getValue(), value, d_gain.getValue());
    }
  };

  d_gain.callback = [&](double value) {
    for (auto & controller : vx_controllers) {
      controller.setGain(p_gain.getValue(), i_gain.getValue(), value);
    }
    for (auto & controller : vy_controllers) {
      controller.setGain(p_gain.getValue(), i_gain.getValue(), value);
    }
  };

  node.declare_parameter("i_saturation", I_SATURATION);
  I_SATURATION = node.get_parameter("i_saturation").as_double();

  for (auto & controller : vx_controllers) {
    controller.setGain(p_gain.getValue(), i_gain.getValue(), d_gain.getValue(), I_SATURATION);
  }

  for (auto & controller : vy_controllers) {
    controller.setGain(p_gain.getValue(), i_gain.getValue(), d_gain.getValue(), I_SATURATION);
  }

  rvo_sim = std::make_unique<RVO::RVOSimulator>(
    RVO_TIME_STEP, RVO_NEIGHBOR_DIST, RVO_MAX_NEIGHBORS, RVO_TIME_HORIZON, RVO_TIME_HORIZON_OBST,
    RVO_RADIUS, RVO_MAX_SPEED);

  // friend robots -> 0~19
  // enemy robots -> 20~39
  for (int i = 0; i < 40; i++) {
    rvo_sim->addAgent(RVO::Vector2(20.0f, 20.0f));
  }
}

void RVO2Planner::reflectWorldToRVOSim(const crane_msgs::msg::RobotCommands & msg)
{
  if (world_model->play_situation.getSituationCommandID() == crane_msgs::msg::PlaySituation::STOP) {
    // 1.5m/sだとたまに超えるので1.0m/sにしておく
    for (int i = 0; i < 40; i++) {
      rvo_sim->setAgentMaxSpeed(i, 1.0f);
    }
  } else {
    for (int i = 0; i < 40; i++) {
      rvo_sim->setAgentMaxSpeed(i, RVO_MAX_SPEED);
    }
  }
  // 味方ロボット：RVO内の位置・速度（＝進みたい方向）の更新
  for (const auto & command : msg.robot_commands) {
    rvo_sim->setAgentPosition(
      command.robot_id, RVO::Vector2(command.current_pose.x, command.current_pose.y));
    rvo_sim->setAgentPrefVelocity(command.robot_id, RVO::Vector2(0.f, 0.f));

    auto robot = world_model->getOurRobot(command.robot_id);

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

        double acceleration = std::min(
          ACCELERATION, static_cast<double>(command.local_planner_config.max_acceleration));
        double deceleration = acceleration * deceleration_factor.getValue();

        // v^2 - v0^2 = 2ax
        // v = sqrt(v0^2 + 2ax)
        // v0 = 0, x = diff(=target_vel)
        // v = sqrt(2ax)
        // double max_vel_by_decel = std::sqrt(2.0 * deceleration * target_vel.norm());
        // PIDによる速度制限（減速のみ）
        double max_vel_by_decel = [&]() {
          double pid_vx = vx_controllers[command.robot_id].update(target_vel.x(), 1. / 30.);
          double pid_vy = vy_controllers[command.robot_id].update(target_vel.y(), 1. / 30.);
          return std::hypot(pid_vx, pid_vy);
        }();

        // v = v0 + at
        double max_vel_by_acc = pre_vel + acceleration * RVO_TIME_STEP;

        double max_vel =
          std::min(static_cast<double>(command.local_planner_config.max_velocity), MAX_VEL);
        max_vel = std::min(max_vel, max_vel_by_decel);
        max_vel = std::min(max_vel, max_vel_by_acc);
        if (
          world_model->play_situation.getSituationCommandID() ==
          crane_msgs::msg::PlaySituation::STOP) {
          // 1.5m/sだとたまに超えるので1.0m/sにしておく
          max_vel = std::min(max_vel, 1.0);
        }

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

    // 障害物回避を無効にする場合、目標速度をそのまま使う
    if (command.local_planner_config.disable_collision_avoidance) {
      vel = toPoint(rvo_sim->getAgentPrefVelocity(original_command.robot_id));
    }

    // 位置目標が許容誤差以下の場合、速度目標を0にする
    if (original_command.control_mode == crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE) {
      double distance = std::hypot(
        original_command.position_target_mode.front().target_x - robot->pose.pos.x(),
        original_command.position_target_mode.front().target_y - robot->pose.pos.y());
      if (distance < original_command.position_target_mode.front().position_tolerance) {
        vel = Velocity::Zero();
      }
    }

    target.target_vx = vel.x();
    target.target_vy = vel.y();

    command.simple_velocity_target_mode.push_back(target);

    if (std::hypot(command.current_velocity.x, command.current_velocity.y) > vel.norm()) {
      // 減速中は減速度制限をmax_accelerationに代入
      command.local_planner_config.max_acceleration *= deceleration_factor.getValue();
    }

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

      bool is_near_our_penalty_area =
        (std::signbit(world_model->getOurGoalCenter().x()) == std::signbit(command.current_pose.x));
      Box penalty_area = [&]() {
        if (is_near_our_penalty_area) {
          return world_model->getOurPenaltyArea();
        } else {
          return world_model->getTheirPenaltyArea();
        }
      }();
      const Point goal_pos = [&]() {
        if (is_near_our_penalty_area) {
          return world_model->getOurGoalCenter();
        } else {
          return world_model->getTheirGoalCenter();
        }
      }();
      if (not command.local_planner_config.disable_goal_area_avoidance) {
        bool is_in_penalty_area = isInBox(penalty_area, target_pos, 0.2);
        double SURROUNDING_OFFSET = 0.3;
        double PENALTY_AREA_OFFSET = 0.1;
        if(world_model->play_situation.getSituationCommandID()== crane_msgs::msg::PlaySituation::STOP) {
          PENALTY_AREA_OFFSET = 0.5;
          SURROUNDING_OFFSET = 0.6;
        }
        if (isInBox(
              penalty_area, Point(command.current_pose.x, command.current_pose.y),
              PENALTY_AREA_OFFSET)) {
          // 目標点をペナルティエリアの外に出るようにする
          target_pos = Point(command.current_pose.x, command.current_pose.y);
          while (isInBox(penalty_area, target_pos, PENALTY_AREA_OFFSET)) {
            target_pos +=
              (target_pos - goal_pos).normalized() * 0.05;  // ゴールから5cmずつ離れていく
          }
          command.position_target_mode.front().position_tolerance = 0.0;
        } else if (isInBox(penalty_area, target_pos, PENALTY_AREA_OFFSET)) {
          // ペナルティエリア内にいる場合は、ペナルティエリアの外に出るようにする
          // ゴールの後ろに回り込んだ場合は、ゴールの前に出るようにする
          if (std::abs(target_pos.x()) > world_model->field_size.x() / 2.0) {
            target_pos.x() = std::copysign(world_model->field_size.x() / 2.0, target_pos.x());
          }

          // 目標点をペナルティエリアの外に出るようにする
          while (isInBox(penalty_area, target_pos, PENALTY_AREA_OFFSET)) {
            target_pos +=
              (target_pos - goal_pos).normalized() * 0.05;  // ゴールから5cmずつ離れていく
          }
          // toleranceがゆるく設定されていると、ペナルティエリアの外に出られないことがあるので、toleranceを0にする
          command.position_target_mode.front().position_tolerance = 0.0;
        }
        // ペナルティエリアを通り抜ける場合は、一旦角に
        const Point current_pos = Point(command.current_pose.x, command.current_pose.y);
        Segment move_line(current_pos, target_pos);
        if (bg::intersects(move_line, penalty_area)) {
          command.position_target_mode.front().position_tolerance = 0.0;
          const auto penalty_area_size = world_model->penalty_area_size;
          Point corner_1 = goal_pos + Point(
                                        std::copysign(penalty_area_size.x(), -goal_pos.x()),
                                        world_model->penalty_area_size.y() * 0.5);
          Point around_corner_1 =
            goal_pos + Point(
                         std::copysign(penalty_area_size.x() + SURROUNDING_OFFSET, -goal_pos.x()),
                         world_model->penalty_area_size.y() * 0.5 + SURROUNDING_OFFSET);

          Point corner_2 = goal_pos + Point(
                                        std::copysign(penalty_area_size.x(), -goal_pos.x()),
                                        -world_model->penalty_area_size.y() * 0.5);
          Point around_corner_2 =
            goal_pos + Point(
                         std::copysign(penalty_area_size.x() + SURROUNDING_OFFSET, -goal_pos.x()),
                         -world_model->penalty_area_size.y() * 0.5 - SURROUNDING_OFFSET);

          auto [distance_1, closest_point_1] = getClosestPointAndDistance(corner_1, move_line);
          auto [distance_2, closest_point_2] = getClosestPointAndDistance(corner_2, move_line);

          const double penalty_area_min_x = world_model->field_size.x() * 0.5 -
                                            world_model->penalty_area_size.x() -
                                            PENALTY_AREA_OFFSET;
          if (
            std::abs(closest_point_1.x()) > penalty_area_min_x &&
            std::abs(closest_point_2.x()) > penalty_area_min_x) {
            // 横切る場合は、近い方の角に向かう
            if (bg::distance(corner_1, current_pos) < bg::distance(corner_2, current_pos)) {
              target_pos = around_corner_1;
            } else {
              target_pos = around_corner_2;
            }
          } else if (isInBox(penalty_area, closest_point_1, PENALTY_AREA_OFFSET)) {
            target_pos = around_corner_1;
          } else if (isInBox(penalty_area, closest_point_2, PENALTY_AREA_OFFSET)) {
            target_pos = around_corner_2;
          } else {
            std::stringstream what;
            what << "Failed to find a target position outside the penalty area.";
            what << " current_pos: " << current_pos.x() << ", " << current_pos.y();
            what << " target_pos: " << target_pos.x() << ", " << target_pos.y();
            what << " closest_point_1: " << closest_point_1.x() << ", " << closest_point_1.y();
            what << " closest_point_2: " << closest_point_2.x() << ", " << closest_point_2.y();
            throw std::runtime_error(what.str());
          }
        }
      }

      if (not command.local_planner_config.disable_ball_avoidance) {
        const Point current_pos = Point(command.current_pose.x, command.current_pose.y);
        const auto & ball_pos = world_model->ball.pos;
        const double MIN_BALL_DISTANCE = 0.2;
        if ((target_pos - ball_pos).norm() < MIN_BALL_DISTANCE) {
          target_pos = ball_pos + (target_pos - ball_pos).normalized() * MIN_BALL_DISTANCE;
        }
        if ((current_pos - ball_pos).norm() < MIN_BALL_DISTANCE) {
          // 現在位置が近い場合は、最優先で離れる
          target_pos =
            ball_pos + (current_pos - ball_pos).normalized() * (MIN_BALL_DISTANCE + 0.05);
        } else {
          Segment move_line(current_pos, target_pos);
          auto [distance, closest_point] = getClosestPointAndDistance(ball_pos, move_line);
          if (
            closest_point != ball_pos && closest_point != target_pos &&
            distance < MIN_BALL_DISTANCE) {
            // 少しずらす
            target_pos = ball_pos + (closest_point - ball_pos).normalized() * MIN_BALL_DISTANCE;
          }
        }
      }

      command.position_target_mode.front().target_x = target_pos.x();
      command.position_target_mode.front().target_y = target_pos.y();
      visualizer->addLine(
        target_pos, Point(command.current_pose.x, command.current_pose.y), 1, "yellow");
    }
  }
}
}  // namespace crane
