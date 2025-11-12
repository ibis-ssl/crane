// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/rvo2_planner.hpp"

#include <boost/stacktrace.hpp>
#include <robocup_ssl_msgs/msg/referee.hpp>

// cspell: ignore OBST

namespace crane
{
RVO2Planner::RVO2Planner(rclcpp::Node & node)
: LocalPlannerBase("rvo2_local_planner", node),
  acceleration_factor("acceleration_factor", node, 1.5)
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

  node.declare_parameter("max_vel", MAX_VEL);
  MAX_VEL = node.get_parameter("max_vel").as_double();

  node.declare_parameter("max_acc", ACCELERATION);
  ACCELERATION = node.get_parameter("max_acc").as_double();

  node.declare_parameter("stop_state_max_velocity", STOP_STATE_MAX_VELOCITY);
  STOP_STATE_MAX_VELOCITY = node.get_parameter("stop_state_max_velocity").as_double();

  rvo_sim = std::make_unique<RVO::RVOSimulator>(
    RVO_TIME_STEP, RVO_NEIGHBOR_DIST, RVO_MAX_NEIGHBORS, RVO_TIME_HORIZON, RVO_TIME_HORIZON_OBST,
    RVO_RADIUS, RVO_MAX_SPEED);

  // friend robots -> 0~19
  // enemy robots -> 20~39
  for (int i = 0; i < 40; i++) {
    rvo_sim->addAgent(RVO::Vector2(20.0f, 20.0f));
  }

  sub_feedback_array = node.create_subscription<crane_msgs::msg::RobotFeedbackArray>(
    "/robot_feedback", 1,
    [this](const crane_msgs::msg::RobotFeedbackArray & msg) { latest_feedback = msg; });
}

auto RVO2Planner::reflectWorldToRVOSim(crane_msgs::msg::RobotCommands & msg) -> void
{
  if (
    world_model->getMsg().play_situation.referee_raw.command.value ==
    robocup_ssl_msgs::msg::RefereeCommand::STOP) {
    for (int i = 0; i < 40; i++) {
      rvo_sim->setAgentMaxSpeed(i, STOP_STATE_MAX_VELOCITY);
    }
  } else {
    for (int i = 0; i < 40; i++) {
      rvo_sim->setAgentMaxSpeed(i, RVO_MAX_SPEED);
    }
  }
  // 味方ロボット：RVO内の位置・速度（＝進みたい方向）の更新
  for (auto & command : msg.robot_commands) {
    rvo_sim->setAgentPosition(
      command.robot_id, RVO::Vector2(command.current_pose.x, command.current_pose.y));
    rvo_sim->setAgentPrefVelocity(command.robot_id, RVO::Vector2(0.f, 0.f));
    auto vel = std::hypot(command.current_velocity.x, command.current_velocity.y);
    double radius = 0.05f + vel * 0.1f;
    rvo_sim->setAgentRadius(command.robot_id, radius);

    auto robot = world_model->getOurRobot(command.robot_id);

    visualizer->circle()
      .radius(radius)
      .center(robot->pose.pos)
      .stroke("yellow", 0.2)
      .strokeWidth(10)
      .build();
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << vel << "m/s";
    visualizer->text()
      .text(ss.str())
      .fontSize(50)
      .position(robot->pose.pos + Point(0, radius + 0.07))
      .textAnchor("middle")
      .fill("yellow", 0.5)
      .build();

    // feedback情報があればそちらの現在位置を参照する
    Point current_position = [&]() -> Point {
      if (auto feedback = ranges::find_if(
            latest_feedback.feedback,
            [&](const auto & f) { return f.robot_id == command.robot_id; });
          feedback != latest_feedback.feedback.end()) {
        return Point(feedback->odom[0], feedback->odom[1]);
      } else {
        return Point(command.current_pose.x, command.current_pose.y);
      }
    }();

    switch (command.control_mode) {
      case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE: {
        Vector2 position_diff;
        position_diff << command.position_target_mode.front().target_x - current_position.x(),
          command.position_target_mode.front().target_y - current_position.y();

        if (command.position_target_mode.empty()) {
          throw std::runtime_error("POSITION_TARGET_MODEだがcommand.position_target_mode.empty()");
        }

        double pre_vel = [&]() {
          if (auto it = std::find_if(
                pre_commands.robot_commands.begin(), pre_commands.robot_commands.end(),
                [&](const auto & c) { return c.robot_id == command.robot_id; });
              it != pre_commands.robot_commands.end()) {
            if (it->simple_velocity_target_mode.size() > 0) {
              return static_cast<double>(std::hypot(
                it->simple_velocity_target_mode.front().target_vx,
                it->simple_velocity_target_mode.front().target_vy));
            } else if (it->polar_velocity_target_mode.size() > 0) {
              return static_cast<double>(it->polar_velocity_target_mode.front().target_velocity_r);
            } else {
              return 0.0;
            }
          } else {
            // 履歴が見つからなければ0
            return 0.0;
          }
        }();

        // 加速度
        command.local_planner_config.max_acceleration_factors.emplace_back(
          crane_msgs::msg::NamedFloat()
            .set__name("RVO2Planner::max_acc from parameter")
            .set__value(ACCELERATION));
        double max_acc = resolveMaxAccelerationFactors(command, ACCELERATION);

        // 速度
        // v^2 - v0^2 = 2ax
        // v = sqrt(v0^2 + 2ax)
        // v0 = 0, x = diff(=target_vel)
        // v = sqrt(2ax)
        double max_vel_by_decel = std::sqrt(2.0 * max_acc * position_diff.norm());
        command.local_planner_config.max_velocity_factors.emplace_back(
          crane_msgs::msg::NamedFloat()
            .set__name("RVO2Planner::max_vel_by_decel")
            .set__value(max_vel_by_decel));

        command.local_planner_config.max_velocity_factors.emplace_back(
          crane_msgs::msg::NamedFloat()
            .set__name("RVO2Planner::max_vel from parameter")
            .set__value(MAX_VEL));
        if (
          world_model->getMsg().play_situation.referee_raw.command.value ==
          robocup_ssl_msgs::msg::RefereeCommand::STOP) {
          command.local_planner_config.max_velocity_factors.emplace_back(
            crane_msgs::msg::NamedFloat()
              .set__name("RVO2Planner STOP制限")
              .set__value(STOP_STATE_MAX_VELOCITY));
        }

        double max_vel = resolveMaxVelocityFactors(command, MAX_VEL);

        Velocity target_vel;
        target_vel << (command.position_target_mode.front().target_x - current_position.x()),
          command.position_target_mode.front().target_y - current_position.y();

        target_vel.x() =
          std::copysign(std::sqrt(2.0 * max_acc * std::abs(target_vel.x())), target_vel.x());
        target_vel.y() =
          std::copysign(std::sqrt(2.0 * max_acc * std::abs(target_vel.y())), target_vel.y());

        // Avoid NaN when already on target
        if (target_vel.norm() > 1e-6) {
          target_vel = target_vel.normalized() * max_vel;

          if (target_vel.norm() < command.local_planner_config.terminal_velocity) {
            target_vel = target_vel.normalized() * command.local_planner_config.terminal_velocity;
          }
        } else {
          target_vel.setZero();
        }
        rvo_sim->setAgentPrefVelocity(command.robot_id, toRVO(target_vel));
        rvo_sim->setAgentMaxSpeed(command.robot_id, max_vel);
        break;
      }
      case crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE: {
        Velocity target_vel;
        target_vel << command.simple_velocity_target_mode.front().target_vx,
          command.simple_velocity_target_mode.front().target_vy;
        rvo_sim->setAgentPrefVelocity(
          command.robot_id, RVO::Vector2(target_vel.x(), target_vel.y()));
        rvo_sim->setAgentMaxSpeed(command.robot_id, target_vel.norm());
        break;
      }
      case crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE: {
        if (command.polar_velocity_target_mode.empty()) {
          throw std::runtime_error(
            "POLAR_VELOCITY_TARGET_MODEだがcommand.polar_velocity_target_mode.empty()");
        }
        double v_r = command.polar_velocity_target_mode.front().target_velocity_r;
        double v_theta = command.polar_velocity_target_mode.front().target_velocity_theta;
        rvo_sim->setAgentPrefVelocity(
          command.robot_id, RVO::Vector2(v_r * cos(v_theta), v_r * sin(v_theta)));
        rvo_sim->setAgentMaxSpeed(command.robot_id, v_r);
        break;
      }
      default: {
        std::stringstream what;
        what << "Unsupported control mode: " << command.control_mode;
        what << ", expected: POSITION_TARGET_MODE, SIMPLE_VELOCITY_TARGET_MODE, "
                "POLAR_VELOCITY_TARGET_MODE";
        throw std::runtime_error(what.str());
      }
    }
  }

  for (const auto & enemy_robot : world_model->theirs().robots) {
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

auto RVO2Planner::extractRobotCommandsFromRVOSim(
  const crane_msgs::msg::RobotCommands & msg, double theta_offset) -> crane_msgs::msg::RobotCommands
{
  crane_msgs::msg::RobotCommands commands;
  for (const auto & original_command : msg.robot_commands) {
    const auto & robot = world_model->getOurRobot(original_command.robot_id);
    crane_msgs::msg::RobotCommand command = original_command;
    // RVOシミュレータの出力をコピーする
    // NOTE: RVOシミュレータは角度を扱わないので角度はそのまま

    command.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
    command.polar_velocity_target_mode.clear();
    command.polar_velocity_target_mode.reserve(1);

    crane_msgs::msg::PolarVelocityTargetMode target;
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
      } else if (
        original_command.local_planner_config.terminal_velocity == 0. &&
        original_command.position_target_mode.front().position_tolerance == 0. && distance < 0.03) {
        // terminal_velocityが0のときはデフォルトで3cmのトレランス
        vel = Velocity::Zero();
      }
    }

    target.target_velocity_r = vel.norm();
    target.target_velocity_theta = std::atan2(vel.y(), vel.x()) + theta_offset;

    command.polar_velocity_target_mode.push_back(target);

    // if (std::hypot(command.current_velocity.x, command.current_velocity.y) < vel.norm()) {
    //  // 減速中は減速度制限をmax_accelerationに代入
    //  command.local_planner_config.max_acceleration *= acceleration_factor.getValue();
    //}

    commands.robot_commands.emplace_back(command);
  }

  pre_commands = commands;
  return commands;
}

auto RVO2Planner::calculateRobotCommand(
  const crane_msgs::msg::RobotCommands & msg, double theta_offset) -> crane_msgs::msg::RobotCommands
{
  crane_msgs::msg::RobotCommands commands = msg;
  if (
    world_model->getMsg().play_situation.referee_raw.command.value !=
    robocup_ssl_msgs::msg::RefereeCommand::HALT) {
    overrideTargetPosition(commands);
  }
  reflectWorldToRVOSim(commands);
  // RVOシミュレータ更新
  rvo_sim->doStep();
  return extractRobotCommandsFromRVOSim(commands, theta_offset);
}

auto RVO2Planner::overrideTargetPosition(crane_msgs::msg::RobotCommands & msg) -> void
{
  for (auto & command : msg.robot_commands) {
    if (command.control_mode == crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE) {
      Point target_pos;
      target_pos << command.position_target_mode.front().target_x,
        command.position_target_mode.front().target_y;

      // NaN値検証とフォールバック処理
      const Point current_pos(command.current_pose.x, command.current_pose.y);
      if (std::isnan(target_pos.x()) || std::isnan(target_pos.y())) {
        std::cout << "[RVO2Planner] NaN detected in target_pos for robot "
                  << static_cast<int>(command.robot_id) << ": target_pos(" << target_pos.x() << ", "
                  << target_pos.y() << "), using current position as fallback" << std::endl;
        to_block_style_yaml(command, std::cout);
        target_pos = current_pos;  // フォールバック: 現在位置に設定
        command.position_target_mode.front().target_x = target_pos.x();
        command.position_target_mode.front().target_y = target_pos.y();
        continue;  // この時点で早期リターン、ペナルティエリア処理をスキップ
      }

      if (std::isnan(current_pos.x()) || std::isnan(current_pos.y())) {
        std::cout << "[RVO2Planner] NaN detected in current_pos for robot "
                  << static_cast<int>(command.robot_id) << ": current_pos(" << current_pos.x()
                  << ", " << current_pos.y() << "), skipping robot" << std::endl;
        continue;  // この場合は処理をスキップ
      }

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
        double SURROUNDING_OFFSET = 0.3;
        double PENALTY_AREA_OFFSET = 0.05;

        // 離れないといけないのは敵ペナルティエリアのみ
        if (not is_near_our_penalty_area) {
          switch (world_model->getMsg().play_situation.referee_raw.command.value) {
            case robocup_ssl_msgs::msg::RefereeCommand::STOP:
            case robocup_ssl_msgs::msg::RefereeCommand::DIRECT_FREE_BLUE:
            case robocup_ssl_msgs::msg::RefereeCommand::DIRECT_FREE_YELLOW:
              PENALTY_AREA_OFFSET = 0.5;
              SURROUNDING_OFFSET = 0.6;
              break;
            default:
              PENALTY_AREA_OFFSET = 0.1;
              SURROUNDING_OFFSET = 0.3;
          }
        }
        if (isInBox(
              penalty_area, Point(command.current_pose.x, command.current_pose.y),
              PENALTY_AREA_OFFSET)) {
          // 目標点をペナルティエリアの外に出るようにする (二番目の条件は無限ループ防止)
          target_pos = Point(command.current_pose.x, command.current_pose.y);
          while (isInBox(penalty_area, target_pos, PENALTY_AREA_OFFSET) and
                 target_pos != goal_pos) {
            target_pos +=
              (target_pos - goal_pos).normalized() * 0.05;  // ゴールから5cmずつ離れていく
          }
          command.position_target_mode.front().position_tolerance = 0.0;
        } else if (isInBox(penalty_area, target_pos, PENALTY_AREA_OFFSET)) {
          // ペナルティエリア内にいる場合は、ペナルティエリアの外に出るようにする
          // ゴールの後ろに回り込んだ場合は、ゴールの前に出るようにする
          if (std::abs(target_pos.x()) > world_model->fieldSize().x() / 2.0) {
            target_pos.x() = std::copysign(world_model->fieldSize().x() / 2.0, target_pos.x());
          }
          // 目標点をペナルティエリアの外に出るようにする (二番目の条件は無限ループ防止)
          while (isInBox(penalty_area, target_pos, PENALTY_AREA_OFFSET) and
                 target_pos != goal_pos) {
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
          const auto penalty_area_size = world_model->penaltyAreaSize();
          Point corner_1 = goal_pos + Point(
                                        std::copysign(penalty_area_size.x(), -goal_pos.x()),
                                        world_model->penaltyAreaSize().y() * 0.5);
          Point around_corner_1 =
            goal_pos + Point(
                         std::copysign(penalty_area_size.x() + SURROUNDING_OFFSET, -goal_pos.x()),
                         world_model->penaltyAreaSize().y() * 0.5 + SURROUNDING_OFFSET);

          Point corner_2 = goal_pos + Point(
                                        std::copysign(penalty_area_size.x(), -goal_pos.x()),
                                        -world_model->penaltyAreaSize().y() * 0.5);
          Point around_corner_2 =
            goal_pos + Point(
                         std::copysign(penalty_area_size.x() + SURROUNDING_OFFSET, -goal_pos.x()),
                         -world_model->penaltyAreaSize().y() * 0.5 - SURROUNDING_OFFSET);

          auto [distance_1, closest_point_1] = getClosestPointAndDistance(corner_1, move_line);
          auto [distance_2, closest_point_2] = getClosestPointAndDistance(corner_2, move_line);

          const double penalty_area_min_x = world_model->fieldSize().x() * 0.5 -
                                            world_model->penaltyAreaSize().x() -
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
        const auto & ball_pos = world_model->ball().pos;
        const double MIN_BALL_DISTANCE = [&]() {
          switch (world_model->getMsg().play_situation.command.value) {
            case crane_msgs::msg::PlaySituation::THEIR_DIRECT_FREE:
              return 0.7;
            case crane_msgs::msg::PlaySituation::STOP:
              return 0.5;
            default:
              return 0.2;
          }
        }();
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

      if (
        not command.local_planner_config.disable_placement_avoidance &&
        world_model->getBallPlacementTarget().has_value()) {
        auto isInPlacementArea = [this](const Point & point, double offset) {
          if (auto placement_area = world_model->getBallPlacementArea(); placement_area) {
            return bg::distance(point, placement_area.value()) <=
                   placement_area.value().radius + offset;
          } else {
            return false;
          }
        };

        if (isInPlacementArea(current_pos, 0.2)) {
          auto [distance, closest_point] = getClosestPointAndDistance(
            world_model->getBallPlacementArea().value().segment, current_pos);
          // 0.6m離れる
          Point target_position = closest_point + (current_pos - closest_point).normalized() * 0.8;
          if (not world_model->point_checker.isFieldInside(target_position, 0.2)) {
            // 一番近いフィールド外のポイントがだめなので逆方向に0.6m離れる
            target_position = closest_point + (closest_point - current_pos).normalized() * 0.8;

            if (auto segment = world_model->getBallPlacementArea().value().segment;
                (closest_point == segment.first || closest_point == segment.second)) {
              // 一番近い点が端点の場合は単純に反対側の点を選択するだけではだめなので、
              // 垂直方向に0.6m離れた点を複数選択して、フィールド外かつ配置エリア外の点を選択する
              std::vector<Point> target_candidates;
              Vector2 vertical_vec =
                getVerticalVec((segment.second - segment.first).normalized()) * 0.8;
              target_candidates.push_back(closest_point + vertical_vec);
              target_candidates.push_back(closest_point - vertical_vec);

              if (auto target = std::ranges::find_if(
                    target_candidates,
                    [&](const auto & target_candidate) {
                      return (
                        not world_model->point_checker.isFieldInside(target_candidate, 0.2) &&
                        not isInPlacementArea(target_candidate, 0.1));
                    });
                  target != target_candidates.end()) {
                target_pos = *target;
              } else {
                // どの候補もだめな場合は移動しない
                target_pos = current_pos;
              }
            } else {
              target_pos = target_position;
            }
          } else {
            target_pos = target_position;
          }
        }
      }

      command.position_target_mode.front().target_x = target_pos.x();
      command.position_target_mode.front().target_y = target_pos.y();
    }
  }
}
}  // namespace crane
