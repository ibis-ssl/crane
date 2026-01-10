// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/rvo2_planner.hpp"

#include <boost/stacktrace.hpp>
#include <crane_local_planner/visualization_helpers.hpp>
#include <range/v3/algorithm/find_if.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/referee.hpp>
#include <sstream>

#include "crane_physics/bang_bang_trajectory.hpp"

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

auto RVO2Planner::reflectWorldToRVOSim(crane_msgs::msg::PositionCommands & msg) -> void
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
    drawRobotRadiusWithSpeed(visualizer, robot->pose.pos, radius, vel, "yellow");

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

    // 位置指令の処理
    Vector2 position_diff;
    position_diff << command.target_x - current_position.x(),
      command.target_y - current_position.y();

    double pre_vel = [&]() {
      if (auto it = ranges::find_if(
            pre_commands.robot_commands,
            [&](const auto & c) { return c.robot_id == command.robot_id; });
          it != ranges::end(pre_commands.robot_commands)) {
        return std::hypot(
                 it->target_x - current_position.x(), it->target_y - current_position.y()) > 0.01
                 ? static_cast<double>(
                     std::hypot(command.current_velocity.x, command.current_velocity.y))
                 : 0.0;
      } else {
        return 0.0;
      }
    }();

    // 減速計算用の減速度を選択（現在速度に応じて高速域・低速域を選択）
    double deceleration_for_planning;
    if (pre_vel >= planning_deceleration_velocity_threshold) {
      deceleration_for_planning = planning_deceleration_high_speed;
    } else {
      deceleration_for_planning = planning_deceleration_low_speed;
    }

    // 加速度（互換性のため）
    command.local_planner_config.max_acceleration_factors.emplace_back(
      crane_msgs::msg::NamedFloat()
        .set__name("RVO2Planner::max_acc from parameter")
        .set__value(deceleration_for_planning));
    double max_acc = resolveMaxAccelerationFactors(command, deceleration_for_planning);

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
    target_vel << (command.target_x - current_position.x()),
      command.target_y - current_position.y();

    BangBangTrajectory2D trajectory;
    trajectory.generate(
      Eigen::Vector2d(current_position.x(), current_position.y()),
      Eigen::Vector2d(command.target_x, command.target_y),
      Eigen::Vector2d(command.current_velocity.x, command.current_velocity.y), max_vel, max_acc);

    // BangBangTrajectoryから速度を取得
    // Vision遅延（~100ms）を考慮してlookahead時間を設定
    const double lookahead_time = 0.1;
    Eigen::Vector2d next_vel = trajectory.getVelocity(lookahead_time);

    // 目標との距離を計算
    double distance_to_target = std::hypot(
      command.target_x - current_position.x(), command.target_y - current_position.y());

    // 疑似I項：低速かつ目標から離れている場合に補正
    // terminal_velocity（0の場合はフォールバック値0.3 m/sを使用）
    const double terminal_vel = command.local_planner_config.terminal_velocity > 0
                                  ? command.local_planner_config.terminal_velocity
                                  : 0.3;
    const double min_distance =
      std::max(static_cast<double>(command.position_tolerance) * 2, 0.03);  // 最低3cm

    if (next_vel.norm() < terminal_vel && distance_to_target > min_distance) {
      // 低速かつ目標から離れている場合、目標方向への terminal_velocity を設定
      Eigen::Vector2d direction =
        (Eigen::Vector2d(command.target_x, command.target_y) - current_position).normalized();
      target_vel = direction * terminal_vel;
    } else if (next_vel.norm() < 1e-6) {
      // 目標到達済み
      target_vel.setZero();
    } else {
      target_vel << next_vel.x(), next_vel.y();
    }
    rvo_sim->setAgentPrefVelocity(command.robot_id, toRVO(target_vel));
    rvo_sim->setAgentMaxSpeed(command.robot_id, max_vel);
  }

  for (const auto & enemy_robot : world_model->theirs().robots) {
    if (enemy_robot->available()) {
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

auto RVO2Planner::extractVelocityCommandsFromRVOSim(
  const crane_msgs::msg::PositionCommands & msg, double theta_offset)
  -> crane_msgs::msg::VelocityCommands
{
  crane_msgs::msg::VelocityCommands commands;
  for (const auto & original_command : msg.robot_commands) {
    const auto & robot = world_model->getOurRobot(original_command.robot_id);

    // VelocityCommandを構築
    crane_msgs::msg::VelocityCommand command;
    command.robot_id = original_command.robot_id;
    command.target_theta = original_command.target_theta;
    command.omega_limit = original_command.omega_limit;
    command.chip_enable = original_command.chip_enable;
    command.kick_power = original_command.kick_power;
    command.dribble_power = original_command.dribble_power;
    command.stop_flag = original_command.stop_flag;
    command.current_pose = original_command.current_pose;
    command.current_velocity = original_command.current_velocity;
    command.state_factors = original_command.state_factors;
    command.planner_name = original_command.planner_name;
    command.delay_checkpoints = original_command.delay_checkpoints;
    command.local_planner_config = original_command.local_planner_config;

    auto vel = toPoint(rvo_sim->getAgentVelocity(original_command.robot_id));

    // 障害物回避を無効にする場合、目標速度をそのまま使う
    if (command.local_planner_config.disable_collision_avoidance) {
      vel = toPoint(rvo_sim->getAgentPrefVelocity(original_command.robot_id));
    }

    // 位置目標が許容誤差以下の場合、速度目標を0にする
    double distance = std::hypot(
      original_command.target_x - robot->pose.pos.x(),
      original_command.target_y - robot->pose.pos.y());
    if (distance < original_command.position_tolerance) {
      vel = Velocity::Zero();
    } else if (
      original_command.local_planner_config.terminal_velocity == 0. &&
      original_command.position_tolerance == 0. && distance < 0.03) {
      // terminal_velocityが0のときはデフォルトで3cmのトレランス
      vel = Velocity::Zero();
    }

    command.target_velocity_r = vel.norm();
    // 座標系の設計について：
    // - target_velocity_thetaはtheta_offsetを含む（half_court_practice_mode対応）
    // - vel.x/y（RVOの出力）はフィールド座標系のまま（theta_offset未適用）
    // - sim_senderで velocity_theta = target_velocity_theta - current_theta により
    //   ロボットローカル座標系に変換される
    command.target_velocity_theta = std::atan2(vel.y(), vel.x()) + theta_offset;

    // 解決済みの速度・加速度制限を設定
    command.max_velocity = original_command.local_planner_config.final_planned_max_velocity.value;
    command.max_acceleration =
      original_command.local_planner_config.final_planned_max_acceleration.value;

    // 効率的な加速のための回転制御
    if (command.local_planner_config.enable_rotation_stop_on_accel) {
      double move_angle = std::atan2(vel.y(), vel.x());
      double angle_diff = getAngleDiff(robot->pose.theta, move_angle);

      constexpr double ANGLE_THRESHOLD = 15.0 * M_PI / 180.0;  // 15度
      bool is_forward_or_backward =
        (std::abs(angle_diff) <= ANGLE_THRESHOLD) ||                 // 前方
        (std::abs(std::abs(angle_diff) - M_PI) <= ANGLE_THRESHOLD);  // 後方

      double current_speed = robot->vel.linear.norm();
      double target_speed = vel.norm();
      double max_speed = rvo_sim->getAgentMaxSpeed(original_command.robot_id);

      bool is_accelerating = current_speed < target_speed;
      bool is_low_speed = current_speed <= max_speed * 0.5;

      // 加速初期段階かつ前後方向に向いている場合、回転を停止
      if (is_forward_or_backward && is_low_speed && is_accelerating && target_speed > 0.01) {
        command.omega_limit = 0.0;
      }
    }

    commands.robot_commands.emplace_back(command);
  }

  pre_commands = msg;
  return commands;
}

auto RVO2Planner::calculateRobotCommand(
  const crane_msgs::msg::PositionCommands & msg, double theta_offset)
  -> crane_msgs::msg::VelocityCommands
{
  crane_msgs::msg::PositionCommands commands = msg;
  if (
    world_model->getMsg().play_situation.referee_raw.command.value !=
    robocup_ssl_msgs::msg::RefereeCommand::HALT) {
    overrideTargetPosition(commands);
  }
  reflectWorldToRVOSim(commands);
  // RVOシミュレータ更新
  rvo_sim->doStep();
  return extractVelocityCommandsFromRVOSim(commands, theta_offset);
}

auto RVO2Planner::overrideTargetPosition(crane_msgs::msg::PositionCommands & msg) -> void
{
  for (auto & command : msg.robot_commands) {
    Point target_pos;
    target_pos << command.target_x, command.target_y;

    // NaN値検証とフォールバック処理
    const Point current_pos(command.current_pose.x, command.current_pose.y);
    if (std::isnan(target_pos.x()) || std::isnan(target_pos.y())) {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("rvo2_local_planner"),
        "[RVO2Planner] NaN detected in target_pos for robot "
          << static_cast<int>(command.robot_id) << ": target_pos(" << target_pos.x() << ", "
          << target_pos.y() << "), using current position as fallback\n"
          << crane_msgs::msg::to_yaml(command));
      target_pos = current_pos;  // フォールバック: 現在位置に設定
      command.target_x = target_pos.x();
      command.target_y = target_pos.y();
      continue;  // この時点で早期リターン、ペナルティエリア処理をスキップ
    }

    if (std::isnan(current_pos.x()) || std::isnan(current_pos.y())) {
      RCLCPP_WARN(
        rclcpp::get_logger("rvo2_local_planner"),
        "[RVO2Planner] NaN detected in current_pos for robot %d: current_pos(%f, %f), skipping "
        "robot",
        static_cast<int>(command.robot_id), current_pos.x(), current_pos.y());
      continue;  // この場合は処理をスキップ
    }

    // 3つの独立した回避ロジックを適用
    adjustForPenaltyAreaAvoidance(target_pos, current_pos, command);
    adjustForBallAvoidance(target_pos, current_pos, command);
    adjustForPlacementAvoidance(target_pos, current_pos, command);

    command.target_x = target_pos.x();
    command.target_y = target_pos.y();
  }
}

auto RVO2Planner::adjustForPenaltyAreaAvoidance(
  Point & target_pos, const Point & current_pos,
  const crane_msgs::msg::PositionCommand & command) const -> void
{
  if (not command.local_planner_config.disable_goal_area_avoidance) {
    constexpr double SURROUNDING_OFFSET = 0.2;
    constexpr double PENALTY_AREA_OFFSET = 0.1;

    auto avoidPenaltyArea = [&](const Box & penalty_area, const Point & goal_pos) {
      if (isInBox(penalty_area, current_pos, PENALTY_AREA_OFFSET)) {
        if (std::abs(current_pos.x()) > world_model->fieldSize().x() / 2.0) {
          target_pos.x() = std::copysign(world_model->fieldSize().x() / 2.0, target_pos.x());
        }
        // 目標点をペナルティエリアの外に出るようにする (二番目の条件は無限ループ防止)
        while (isInBox(penalty_area, target_pos, PENALTY_AREA_OFFSET) and
               target_pos != current_pos) {
          target_pos += (target_pos - current_pos).normalized() * 0.05;  // 5cmずつ離れていく
        }
      } else if (isInBox(penalty_area, target_pos, PENALTY_AREA_OFFSET)) {
        // ペナルティエリア内にいる場合は、ペナルティエリアの外に出るようにする
        if (std::abs(target_pos.x()) > world_model->fieldSize().x() / 2.0) {
          target_pos.x() = std::copysign(world_model->fieldSize().x() / 2.0, target_pos.x());
        }
        // 目標点をペナルティエリアの外に出るようにする
        while (isInBox(penalty_area, target_pos, PENALTY_AREA_OFFSET) and target_pos != goal_pos) {
          target_pos += (target_pos - goal_pos).normalized() * 0.05;
        }
      }
      // ペナルティエリアを通り抜ける場合は、一旦角に
      Segment move_line(current_pos, target_pos);
      if (bg::intersects(move_line, penalty_area)) {
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
                                          world_model->penaltyAreaSize().x() - PENALTY_AREA_OFFSET;
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
    };

    // 自陣と敵陣の両方のペナルティエリアを回避
    avoidPenaltyArea(world_model->getOurPenaltyArea(), world_model->getOurGoalCenter());
    avoidPenaltyArea(world_model->getTheirPenaltyArea(), world_model->getTheirGoalCenter());
  }
}

auto RVO2Planner::adjustForBallAvoidance(
  Point & target_pos, const Point & current_pos,
  const crane_msgs::msg::PositionCommand & command) const -> void
{
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
      target_pos = ball_pos + (current_pos - ball_pos).normalized() * (MIN_BALL_DISTANCE + 0.05);
    } else {
      Segment move_line(current_pos, target_pos);
      auto [distance, closest_point] = getClosestPointAndDistance(ball_pos, move_line);
      if (
        closest_point != ball_pos && closest_point != target_pos && distance < MIN_BALL_DISTANCE) {
        // 少しずらす
        target_pos = ball_pos + (closest_point - ball_pos).normalized() * MIN_BALL_DISTANCE;
      }
    }
  }
}

auto RVO2Planner::adjustForPlacementAvoidance(
  Point & target_pos, const Point & current_pos,
  const crane_msgs::msg::PositionCommand & command) const -> void
{
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
}
}  // namespace crane
