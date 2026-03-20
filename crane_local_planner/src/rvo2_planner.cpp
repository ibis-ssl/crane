// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/rvo2_planner.hpp"

#include <algorithm>
#include <boost/stacktrace.hpp>
#include <crane_local_planner/visualization_helpers.hpp>
#include <crane_msg_wrappers/command_wrapper_base.hpp>
#include <range/v3/algorithm/find_if.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/referee.hpp>

// cspell: ignore OBST

namespace crane
{
namespace
{
auto pointChanged(const Point & before, const Point & after, double epsilon = 1e-4) -> bool
{
  return (before - after).norm() > epsilon;
}
}  // namespace

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

  node.declare_parameter("field_boundary_offset", FIELD_BOUNDARY_OFFSET);
  FIELD_BOUNDARY_OFFSET = node.get_parameter("field_boundary_offset").as_double();

  node.declare_parameter("crash_speed_limit", CRASH_SPEED_LIMIT);
  CRASH_SPEED_LIMIT = node.get_parameter("crash_speed_limit").as_double();
  node.declare_parameter("crash_safety_margin", CRASH_SAFETY_MARGIN);
  CRASH_SAFETY_MARGIN = node.get_parameter("crash_safety_margin").as_double();
  node.declare_parameter("crash_avoidance_distance", CRASH_AVOIDANCE_DISTANCE);
  CRASH_AVOIDANCE_DISTANCE = node.get_parameter("crash_avoidance_distance").as_double();
  node.declare_parameter("crash_avoidance_decel_distance", CRASH_AVOIDANCE_DECEL_DISTANCE);
  CRASH_AVOIDANCE_DECEL_DISTANCE = node.get_parameter("crash_avoidance_decel_distance").as_double();
  if (CRASH_AVOIDANCE_DISTANCE <= CRASH_AVOIDANCE_DECEL_DISTANCE) {
    RCLCPP_ERROR(
      node.get_logger(),
      "crash_avoidance_distance (%.2f) must be > crash_avoidance_decel_distance (%.2f). "
      "Resetting to defaults.",
      CRASH_AVOIDANCE_DISTANCE, CRASH_AVOIDANCE_DECEL_DISTANCE);
    CRASH_AVOIDANCE_DISTANCE = 1.0;
    CRASH_AVOIDANCE_DECEL_DISTANCE = 0.5;
  }

  node.declare_parameter("penalty_area_offset", PENALTY_AREA_OFFSET);
  PENALTY_AREA_OFFSET = node.get_parameter("penalty_area_offset").as_double();
  node.declare_parameter("penalty_area_time_horizon_obst", PENALTY_AREA_TIME_HORIZON_OBST);
  PENALTY_AREA_TIME_HORIZON_OBST =
    static_cast<float>(node.get_parameter("penalty_area_time_horizon_obst").as_double());

  node.declare_parameter("enable_velocity_plan_trace", false);
  enable_velocity_plan_trace = node.get_parameter("enable_velocity_plan_trace").as_bool();

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

auto RVO2Planner::initializePenaltyAreaObstacles() -> void
{
  // ペナルティエリアをRVO2のポリゴン障害物として登録する。
  // processObstacles()呼び出し後は変更不可なので、フィールド情報確定後に一度だけ呼ぶ。
  // 頂点は反時計回り（RVO2の仕様）で指定する。
  const Box our_area = world_model->getOurPenaltyArea();
  const Box their_area = world_model->getTheirPenaltyArea();
  auto addBoxObstacle = [&](const Box & box) {
    const float xmin = static_cast<float>(box.min_corner().x());
    const float xmax = static_cast<float>(box.max_corner().x());
    const float ymin = static_cast<float>(box.min_corner().y());
    const float ymax = static_cast<float>(box.max_corner().y());
    // 反時計回り: 左下 → 右下 → 右上 → 左上
    rvo_sim->addObstacle({{xmin, ymin}, {xmax, ymin}, {xmax, ymax}, {xmin, ymax}});
  };
  addBoxObstacle(our_area);
  addBoxObstacle(their_area);
  rvo_sim->processObstacles();
  penalty_area_obstacles_initialized = true;
  RCLCPP_INFO(
    rclcpp::get_logger("rvo2_local_planner"),
    "Penalty area obstacles initialized (our: [%.2f,%.2f]x[%.2f,%.2f], "
    "their: [%.2f,%.2f]x[%.2f,%.2f])",
    our_area.min_corner().x(), our_area.max_corner().x(), our_area.min_corner().y(),
    our_area.max_corner().y(), their_area.min_corner().x(), their_area.max_corner().x(),
    their_area.min_corner().y(), their_area.max_corner().y());
}

auto RVO2Planner::reflectWorldToRVOSim(crane_msgs::msg::RobotCommands & msg) -> void
{
  // ペナルティエリアObstacleの遅延初期化（フィールド情報が確定してから一度だけ）
  if (!penalty_area_obstacles_initialized && world_model->fieldSize().squaredNorm() > 0.01) {
    initializePenaltyAreaObstacles();
  }

  const auto referee_command = world_model->getMsg().play_situation.referee_raw.command.value;
  if (referee_command == robocup_ssl_msgs::msg::RefereeCommand::STOP) {
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
    if (command.position_target_mode.empty()) {
      RCLCPP_WARN(
        rclcpp::get_logger("rvo2_local_planner"),
        "robot_id=%d has no position_target_mode. skipping.", static_cast<int>(command.robot_id));
      continue;
    }
    auto & pos_mode = command.position_target_mode.front();

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
      if (
        auto feedback = ranges::find_if(
          latest_feedback.feedback, [&](const auto & f) { return f.robot_id == command.robot_id; });
        feedback != latest_feedback.feedback.end()) {
        return Point(feedback->odom[0], feedback->odom[1]);
      } else {
        return Point(command.current_pose.x, command.current_pose.y);
      }
    }();

    // 位置指令の処理
    Vector2 position_diff;
    position_diff << pos_mode.target_x - current_position.x(),
      pos_mode.target_y - current_position.y();

    double pre_vel = [&]() {
      if (
        auto it = ranges::find_if(
          pre_commands.robot_commands,
          [&](const auto & c) { return c.robot_id == command.robot_id; });
        it != ranges::end(pre_commands.robot_commands)) {
        if (it->position_target_mode.empty()) {
          return 0.0;
        }
        return std::hypot(
                 it->position_target_mode.front().target_x - current_position.x(),
                 it->position_target_mode.front().target_y - current_position.y()) > 0.01
                 ? vel
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

    // max_brk: 停止のための減速度（planning_decelerationの値を使用）
    double max_brk = deceleration_for_planning;

    command.local_planner_config.max_velocity_factors.emplace_back(
      crane_msgs::msg::NamedFloat()
        .set__name("RVO2Planner::max_vel from parameter")
        .set__value(MAX_VEL));
    if (referee_command == robocup_ssl_msgs::msg::RefereeCommand::STOP) {
      command.local_planner_config.max_velocity_factors.emplace_back(
        crane_msgs::msg::NamedFloat()
          .set__name("RVO2Planner STOP制限")
          .set__value(STOP_STATE_MAX_VELOCITY));
    }

    double max_vel = resolveMaxVelocityFactors(command, MAX_VEL);

    Velocity target_vel;
    target_vel << (pos_mode.target_x - current_position.x()),
      pos_mode.target_y - current_position.y();

    // 速度超過クランプ: Vision観測誤差でmax_velをわずかに超えた速度を正規化（Sumatra adaptVel相当）
    constexpr double MAX_VEL_TOLERANCE = 0.2;
    Eigen::Vector2d v0(command.current_velocity.x, command.current_velocity.y);
    if (vel > max_vel && vel < max_vel + MAX_VEL_TOLERANCE) {
      v0 = v0 * (max_vel / vel);
    }

    // 目標との距離を計算
    const double dx = pos_mode.target_x - current_position.x();
    const double dy = pos_mode.target_y - current_position.y();
    const double distance_to_target = std::hypot(dx, dy);

    // 各軸独立に減速制約を計算: v = sign(d) * sqrt(2 * max_brk * |d|)
    // これにより現在速度方向と目標方向が異なる場合（横方向の慣性）も正しく扱える
    auto brk_vel = [max_brk](double d) -> double {
      const double abs_d = std::abs(d);
      return (abs_d > 1e-9) ? std::copysign(std::sqrt(2.0 * max_brk * abs_d), d) : 0.0;
    };
    Eigen::Vector2d next_vel(brk_vel(dx), brk_vel(dy));
    // 合成速度がmax_velを超えた場合はスケールダウン
    const double next_vel_norm = next_vel.norm();
    if (next_vel_norm > max_vel) {
      next_vel *= max_vel / next_vel_norm;
    }

    // terminal_velocity: スキルが明示的に設定した場合のみ疑似I項として適用する
    // 0（デフォルト・未指定）のときはBangBang軌道の出力に従い、目標で停止する
    const double terminal_vel = command.local_planner_config.terminal_velocity;
    const double min_distance =
      std::max(static_cast<double>(pos_mode.position_tolerance) * 2, 0.03);  // 最低3cm

    if (terminal_vel > 0 && next_vel.norm() < terminal_vel && distance_to_target > min_distance) {
      // terminal_velocityが指定されており、低速かつ目標から離れている場合に補正
      Eigen::Vector2d direction =
        (Eigen::Vector2d(pos_mode.target_x, pos_mode.target_y) - current_position).normalized();
      target_vel = direction * terminal_vel;
    } else if (next_vel.norm() < 1e-6) {
      // 目標到達済み
      target_vel.setZero();
    } else {
      target_vel << next_vel.x(), next_vel.y();
    }
    // 衝突ファール (crashing) 回避:
    // SSLルールでは衝突時の速度ベクトル差をロボット間直線に射影した値が
    // 1.5 m/s を超えるとファール。敵ロボットへの接近方向成分を制限する。
    if (!command.local_planner_config.disable_crash_avoidance) {
      const Point our_pos(command.current_pose.x, command.current_pose.y);
      for (const auto & enemy_robot : world_model->theirs().robotsWhere().available().get()) {
        const Vector2 to_enemy = enemy_robot->pose.pos - our_pos;
        const double dist = to_enemy.norm();

        if (dist < CRASH_AVOIDANCE_DISTANCE && dist > 0.01) {
          const Vector2 dir = to_enemy.normalized();

          // 敵ロボットの自ロボットへの接近速度成分（敵が向かってくる方向を正）
          const double enemy_approach = -enemy_robot->vel.linear.dot(dir);

          // 安全な接近速度 = ルール閾値 - 敵の接近速度 - 安全マージン
          double safe_approach =
            CRASH_SPEED_LIMIT - std::max(0.0, enemy_approach) - CRASH_SAFETY_MARGIN;
          safe_approach = std::max(safe_approach, 0.0);

          // 距離に応じた線形補間: 近い(factor=1)ほど制限を強く適用
          const double factor =
            1.0 - std::clamp(
                    (dist - CRASH_AVOIDANCE_DECEL_DISTANCE) /
                      (CRASH_AVOIDANCE_DISTANCE - CRASH_AVOIDANCE_DECEL_DISTANCE),
                    0.0, 1.0);

          // target_velの敵方向への射影成分を取得
          const double approach_component = target_vel.dot(dir);

          // factor補間した制限値を計算（factor=0なら制限なし）
          const double max_approach = approach_component * (1.0 - factor) + safe_approach * factor;

          // 接近方向成分が制限を超えており、かつ敵に向かっている場合のみ制限
          if (approach_component > max_approach && approach_component > 0.0) {
            // 横方向成分を維持しながら接近方向成分のみを制限
            const Vector2 lateral = target_vel - approach_component * dir;
            target_vel = lateral + max_approach * dir;
            addOrUpdatePlanningFactor(
              command, "CrashAvoidance",
              "robot" + std::to_string(enemy_robot->id) + ":" + formatPlanningDouble(max_approach));
          }
        }
      }
    }

    // ペナルティエリア物理ブレーキング制約
    // ORCA（速度空間制約）はコマンド速度を制限するが、ロボットの物理慣性は考慮できない。
    // 境界までの距離に基づく「物理的に停止可能な最大接近速度」を計算してprefVelocityを制限する。
    //   max_approach_vel = sqrt(2 * planning_deceleration_high_speed * dist_to_boundary)
    if (!command.local_planner_config.disable_goal_area_avoidance) {
      auto applyPhysicalBrakingConstraint = [&](const Box & area) {
        const double xmin = area.min_corner().x() - PENALTY_AREA_OFFSET;
        const double xmax = area.max_corner().x() + PENALTY_AREA_OFFSET;
        const double ymin = area.min_corner().y() - PENALTY_AREA_OFFSET;
        const double ymax = area.max_corner().y() + PENALTY_AREA_OFFSET;
        // ロボットがエリア内にいる場合はグローバル回避に任せる
        if (
          current_position.x() >= xmin && current_position.x() <= xmax &&
          current_position.y() >= ymin && current_position.y() <= ymax) {
          return;
        }

        // 各軸方向の境界からの距離（外側から境界への距離、外側で正）
        const double dx_left = xmin - current_position.x();   // 左側にある場合 > 0
        const double dx_right = current_position.x() - xmax;  // 右側にある場合 > 0
        const double dy_below = ymin - current_position.y();  // 下側にある場合 > 0
        const double dy_above = current_position.y() - ymax;  // 上側にある場合 > 0

        if ((dx_left > 0.0 || dx_right > 0.0) && (dy_below > 0.0 || dy_above > 0.0)) {
          // 角の外側: 最近傍角頂点までのユークリッド距離でブレーキング制約を計算
          // 各軸独立制約より緩やかになり、角を回り込む際の接線方向の速度が維持される
          const double corner_x = (dx_left > 0.0) ? xmin : xmax;
          const double corner_y = (dy_below > 0.0) ? ymin : ymax;
          const double cdx = corner_x - current_position.x();
          const double cdy = corner_y - current_position.y();
          const double dist_to_corner = std::hypot(cdx, cdy);
          if (dist_to_corner > 1e-9) {
            const Eigen::Vector2d corner_dir(cdx / dist_to_corner, cdy / dist_to_corner);
            const double approach = target_vel.dot(corner_dir);
            if (approach > 0.0) {
              const double v_max =
                std::sqrt(2.0 * planning_deceleration_high_speed * dist_to_corner);
              if (approach > v_max) {
                target_vel -= (approach - v_max) * corner_dir;
              }
            }
          }
        } else {
          // 面の外側: 各境界面への距離を計算し、接近方向速度成分を物理制動距離内に制限する
          // 左面: ロボットが左(x < xmin)にいてxmin方向に接近中
          if (dx_left > 0.0 && target_vel.x() > 0.0) {
            const double v_max = std::sqrt(2.0 * planning_deceleration_high_speed * dx_left);
            target_vel.x() = std::min(target_vel.x(), v_max);
          }
          // 右面: ロボットが右(x > xmax)にいてxmax方向に接近中
          if (dx_right > 0.0 && target_vel.x() < 0.0) {
            const double v_max = std::sqrt(2.0 * planning_deceleration_high_speed * dx_right);
            target_vel.x() = std::max(target_vel.x(), -v_max);
          }
          // 下面: ロボットが下(y < ymin)にいてymin方向に接近中
          if (dy_below > 0.0 && target_vel.y() > 0.0) {
            const double v_max = std::sqrt(2.0 * planning_deceleration_high_speed * dy_below);
            target_vel.y() = std::min(target_vel.y(), v_max);
          }
          // 上面: ロボットが上(y > ymax)にいてymax方向に接近中
          if (dy_above > 0.0 && target_vel.y() < 0.0) {
            const double v_max = std::sqrt(2.0 * planning_deceleration_high_speed * dy_above);
            target_vel.y() = std::max(target_vel.y(), -v_max);
          }
        }
      };
      applyPhysicalBrakingConstraint(world_model->getOurPenaltyArea());
      applyPhysicalBrakingConstraint(world_model->getTheirPenaltyArea());
    }

    // ペナルティエリア障害物回避のtimeHorizonObstをフラグに応じて設定
    // disable_goal_area_avoidance=trueのロボット（GKなど）は障害物回避を無効化
    rvo_sim->setAgentTimeHorizonObst(
      command.robot_id, command.local_planner_config.disable_goal_area_avoidance
                          ? 0.0f
                          : PENALTY_AREA_TIME_HORIZON_OBST);

    rvo_sim->setAgentPrefVelocity(command.robot_id, toRVO(target_vel));
    rvo_sim->setAgentMaxSpeed(command.robot_id, max_vel);

    // 速度計画トレースに計画点を追加
    if (enable_velocity_plan_trace && !command.velocity_plan_trace.empty()) {
      // 現在時刻から100ms後の予測位置・速度を記録
      constexpr double TRACE_LOOKAHEAD_S = 0.1;
      const int32_t target_time_us = static_cast<int32_t>(TRACE_LOOKAHEAD_S * 1e6);  // 100000us
      Eigen::Vector2d predicted_pos = current_position + next_vel * TRACE_LOOKAHEAD_S;
      Eigen::Vector2d predicted_vel = next_vel;

      VelocityPlanTracker::addPlanPoint(
        command.velocity_plan_trace[0], "local_planner", predicted_pos, predicted_vel,
        target_time_us,
        0  // estimated_arrival_time_us（後で実装可能）
      );
    }
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
  const crane_msgs::msg::RobotCommands & msg, double theta_offset) -> crane_msgs::msg::RobotCommands
{
  crane_msgs::msg::RobotCommands commands;
  for (const auto & original_command : msg.robot_commands) {
    const auto & robot = world_model->getOurRobot(original_command.robot_id);
    if (original_command.position_target_mode.empty()) {
      continue;
    }
    const auto & original_pos_mode = original_command.position_target_mode.front();

    // 元コマンドを保持したまま下流向け情報を追記する
    crane_msgs::msg::RobotCommand command = original_command;
    command.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
    if (command.polar_velocity_target_mode.empty()) {
      command.polar_velocity_target_mode.emplace_back();
    }

    auto pref_vel = toPoint(rvo_sim->getAgentPrefVelocity(original_command.robot_id));
    auto vel = toPoint(rvo_sim->getAgentVelocity(original_command.robot_id));
    addOrUpdatePlanningFactor(command, "RVO2PrefSpeed", formatPlanningDouble(pref_vel.norm()));

    // 速度修正をトレースに記録（RVO2による修正）
    if (enable_velocity_plan_trace && !command.velocity_plan_trace.empty()) {
      // RVO2による修正を記録
      if ((vel - pref_vel).norm() > 0.01) {  // 1cm/s以上の差がある場合のみ記録
        VelocityPlanTracker::addCorrection(command.velocity_plan_trace[0], "rvo2", pref_vel, vel);
      }
    }

    // 障害物回避を無効にする場合、目標速度をそのまま使う
    if (command.local_planner_config.disable_collision_avoidance) {
      vel = pref_vel;
      if (vel.norm() > rvo_sim->getAgentMaxSpeed(original_command.robot_id)) {
        vel = vel.normalized() * rvo_sim->getAgentMaxSpeed(original_command.robot_id);
      }
      addOrUpdatePlanningFactor(command, "RVO2CollisionAvoidance", "DISABLED");
    } else {
      addOrUpdatePlanningFactor(command, "RVO2CollisionAvoidance", "ENABLED");
    }

    // 位置目標が許容誤差以下の場合、速度目標を0にする
    double distance = std::hypot(
      original_pos_mode.target_x - robot->pose.pos.x(),
      original_pos_mode.target_y - robot->pose.pos.y());
    addOrUpdatePlanningFactor(command, "RVO2DistanceToTarget", formatPlanningDouble(distance));
    addOrUpdatePlanningFactor(
      command, "RVO2PositionTolerance", formatPlanningDouble(original_pos_mode.position_tolerance));
    ZeroVelocityReason zero_velocity_reason = ZeroVelocityReason::NONE;
    if (distance < original_pos_mode.position_tolerance) {
      vel = Velocity::Zero();
      zero_velocity_reason = ZeroVelocityReason::POSITION_TOLERANCE;
    }
    if (zero_velocity_reason == ZeroVelocityReason::NONE && vel.norm() < 1e-4) {
      zero_velocity_reason = (pref_vel.norm() > 1e-3)
                               ? ZeroVelocityReason::RVO_COLLISION_OR_CONSTRAINT
                               : ZeroVelocityReason::PREF_VELOCITY_ZERO;
    }
    addOrUpdatePlanningFactor(command, "RVO2OutputSpeed", formatPlanningDouble(vel.norm()));
    addOrUpdatePlanningFactor(command, "RVO2ZeroVelocityReason", toString(zero_velocity_reason));

    // 座標系の設計について：
    // - target_velocity_thetaはtheta_offsetを含む（half_court_practice_mode対応）
    // - vel.x/y（RVOの出力）はフィールド座標系のまま（theta_offset未適用）
    // - sim_senderで velocity_theta = target_velocity_theta - current_theta により
    //   ロボットローカル座標系に変換される
    command.polar_velocity_target_mode.front().target_velocity_r = vel.norm();
    command.polar_velocity_target_mode.front().target_velocity_theta =
      std::atan2(vel.y(), vel.x()) + theta_offset;

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
  const crane_msgs::msg::RobotCommands & msg, double theta_offset) -> crane_msgs::msg::RobotCommands
{
  crane_msgs::msg::RobotCommands commands = msg;
  const auto referee_command = world_model->getMsg().play_situation.referee_raw.command.value;
  if (referee_command != robocup_ssl_msgs::msg::RefereeCommand::HALT) {
    overrideTargetPosition(commands);
  }
  reflectWorldToRVOSim(commands);
  // RVOシミュレータ更新
  rvo_sim->doStep();
  return extractVelocityCommandsFromRVOSim(commands, theta_offset);
}

auto RVO2Planner::overrideTargetPosition(crane_msgs::msg::RobotCommands & msg) -> void
{
  for (auto & command : msg.robot_commands) {
    if (command.position_target_mode.empty()) {
      continue;
    }
    auto & pos_mode = command.position_target_mode.front();

    Point target_pos;
    target_pos << pos_mode.target_x, pos_mode.target_y;
    const Point original_target_pos = target_pos;
    addOrUpdatePlanningFactor(command, "RVO2AdjustFieldBoundary", "0");
    addOrUpdatePlanningFactor(command, "RVO2AdjustPenaltyArea", "0");
    addOrUpdatePlanningFactor(command, "RVO2AdjustBallAvoidance", "0");
    addOrUpdatePlanningFactor(command, "RVO2AdjustPlacementAvoidance", "0");
    addOrUpdatePlanningFactor(command, "RVO2TargetFallback", "NONE");

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
      pos_mode.target_x = target_pos.x();
      pos_mode.target_y = target_pos.y();
      addOrUpdatePlanningFactor(command, "RVO2TargetFallback", "NAN_TARGET");
      continue;  // この時点で早期リターン、ペナルティエリア処理をスキップ
    }

    if (std::isnan(current_pos.x()) || std::isnan(current_pos.y())) {
      RCLCPP_WARN(
        rclcpp::get_logger("rvo2_local_planner"),
        "[RVO2Planner] NaN detected in current_pos for robot %d: current_pos(%f, %f), skipping "
        "robot",
        static_cast<int>(command.robot_id), current_pos.x(), current_pos.y());
      addOrUpdatePlanningFactor(command, "RVO2TargetFallback", "NAN_CURRENT");
      continue;  // この場合は処理をスキップ
    }

    // 4つの独立した回避ロジックを適用
    Point before = target_pos;
    adjustForFieldBoundary(target_pos, current_pos, command);
    if (pointChanged(before, target_pos)) {
      addOrUpdatePlanningFactor(command, "RVO2AdjustFieldBoundary", "1");
    }

    before = target_pos;
    adjustForPenaltyAreaAvoidance(target_pos, current_pos, command);
    if (pointChanged(before, target_pos)) {
      addOrUpdatePlanningFactor(command, "RVO2AdjustPenaltyArea", "1");
    }

    before = target_pos;
    adjustForBallAvoidance(target_pos, current_pos, command);
    if (pointChanged(before, target_pos)) {
      addOrUpdatePlanningFactor(command, "RVO2AdjustBallAvoidance", "1");
    }

    before = target_pos;
    adjustForPlacementAvoidance(target_pos, current_pos, command);
    if (pointChanged(before, target_pos)) {
      addOrUpdatePlanningFactor(command, "RVO2AdjustPlacementAvoidance", "1");
    }

    pos_mode.target_x = target_pos.x();
    pos_mode.target_y = target_pos.y();
    addOrUpdatePlanningFactor(
      command, "RVO2TargetAdjustedDistance",
      formatPlanningDouble((target_pos - original_target_pos).norm()));
  }
}

auto RVO2Planner::adjustForFieldBoundary(
  Point & target_pos, const Point & current_pos,
  const crane_msgs::msg::RobotCommand & command) const -> void
{
  if (command.local_planner_config.disable_field_boundary) {
    return;
  }
  const double max_x = world_model->fieldSize().x() / 2.0 + FIELD_BOUNDARY_OFFSET;
  const double max_y = world_model->fieldSize().y() / 2.0 + FIELD_BOUNDARY_OFFSET;

  // フィールド境界のBox
  Box field_box;
  field_box.min_corner() << -max_x, -max_y;
  field_box.max_corner() << max_x, max_y;

  // 目標位置がフィールド内ならそのまま
  if (isInBox(field_box, target_pos)) {
    return;
  }

  // 現在位置から目標位置への線分
  Segment move_line(current_pos, target_pos);

  // フィールド境界の4辺
  Segment top_edge(Point(-max_x, max_y), Point(max_x, max_y));
  Segment bottom_edge(Point(-max_x, -max_y), Point(max_x, -max_y));
  Segment right_edge(Point(max_x, -max_y), Point(max_x, max_y));
  Segment left_edge(Point(-max_x, -max_y), Point(-max_x, max_y));

  // 各辺との交点を計算
  std::vector<Point> all_intersections;
  for (const auto & edge : {top_edge, bottom_edge, right_edge, left_edge}) {
    auto intersections = getIntersections(move_line, edge);
    all_intersections.insert(all_intersections.end(), intersections.begin(), intersections.end());
  }

  // 現在位置に最も近い交点を選択（最初に交差する点）
  if (!all_intersections.empty()) {
    auto closest = std::min_element(
      all_intersections.begin(), all_intersections.end(),
      [&current_pos](const Point & a, const Point & b) {
        return bg::distance(a, current_pos) < bg::distance(b, current_pos);
      });
    target_pos = *closest;
  } else {
    // 交点がない場合（現在位置がフィールド外など）、単純なクランプにフォールバック
    target_pos.x() = std::clamp(target_pos.x(), -max_x, max_x);
    target_pos.y() = std::clamp(target_pos.y(), -max_y, max_y);
  }
}

auto RVO2Planner::adjustForPenaltyAreaAvoidance(
  Point & target_pos, const Point & current_pos,
  const crane_msgs::msg::RobotCommand & command) const -> void
{
  if (not command.local_planner_config.disable_goal_area_avoidance) {
    constexpr double SURROUNDING_OFFSET = 0.2;

    auto avoidPenaltyArea = [&](const Box & penalty_area, const Point & goal_pos) {
      constexpr int MAX_ITERATIONS = 100;
      if (isInBox(penalty_area, current_pos, PENALTY_AREA_OFFSET)) {
        if (std::abs(current_pos.x()) > world_model->fieldSize().x() / 2.0) {
          target_pos.x() = std::copysign(world_model->fieldSize().x() / 2.0, target_pos.x());
        }
        // 目標点をペナルティエリアの外に出るようにする (反復上限で無限ループ防止)
        for (int iter = 0;
             iter < MAX_ITERATIONS && isInBox(penalty_area, target_pos, PENALTY_AREA_OFFSET) &&
             target_pos != current_pos;
             ++iter) {
          target_pos += (target_pos - current_pos).normalized() * 0.05;  // 5cmずつ離れていく
        }
      } else if (isInBox(penalty_area, target_pos, PENALTY_AREA_OFFSET)) {
        // ペナルティエリア内にいる場合は、ペナルティエリアの外に出るようにする
        if (std::abs(target_pos.x()) > world_model->fieldSize().x() / 2.0) {
          target_pos.x() = std::copysign(world_model->fieldSize().x() / 2.0, target_pos.x());
        }
        // 目標点をペナルティエリアの外に出るようにする
        for (int iter = 0;
             iter < MAX_ITERATIONS && isInBox(penalty_area, target_pos, PENALTY_AREA_OFFSET) &&
             target_pos != goal_pos;
             ++iter) {
          target_pos += (target_pos - goal_pos).normalized() * 0.05;
        }
      }
      // ペナルティエリアを通り抜ける場合は、総移動距離が短い方の角を経由
      // estimatePenaltyAwareDistance と同じ選択ロジックで一貫性を保つ
      Segment move_line(current_pos, target_pos);
      if (bg::intersects(move_line, penalty_area)) {
        const auto penalty_area_size = world_model->penaltyAreaSize();
        const double x_offset =
          std::copysign(penalty_area_size.x() + SURROUNDING_OFFSET, -goal_pos.x());
        const double half_height = penalty_area_size.y() * 0.5 + SURROUNDING_OFFSET;
        const Point around_corner_1 = goal_pos + Point(x_offset, half_height);
        const Point around_corner_2 = goal_pos + Point(x_offset, -half_height);

        const double dist_via_1 =
          (around_corner_1 - current_pos).norm() + (target_pos - around_corner_1).norm();
        const double dist_via_2 =
          (around_corner_2 - current_pos).norm() + (target_pos - around_corner_2).norm();
        const Point & chosen_corner =
          (dist_via_1 <= dist_via_2) ? around_corner_1 : around_corner_2;
        // 角に十分近い場合はリダイレクトせず通過（RVO2が侵入防止を担当）
        // 角ウェイポイントへのBangBang減速を防ぎ、通過速度を維持する
        constexpr double CORNER_PASS_THROUGH_DISTANCE = 0.5;
        if ((current_pos - chosen_corner).norm() > CORNER_PASS_THROUGH_DISTANCE) {
          target_pos = chosen_corner;
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
  const crane_msgs::msg::RobotCommand & command) const -> void
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
  const crane_msgs::msg::RobotCommand & command) const -> void
{
  if (
    not command.local_planner_config.disable_placement_avoidance &&
    world_model->getBallPlacementTarget().has_value()) {
    const auto placement_area_opt = world_model->getBallPlacementArea();
    if (!placement_area_opt) return;
    const auto & placement_area = placement_area_opt.value();

    auto isInPlacementArea = [&placement_area](const Point & point, double offset) {
      return bg::distance(point, placement_area) <= placement_area.radius + offset;
    };

    if (isInPlacementArea(current_pos, 0.2)) {
      auto [distance, closest_point] =
        getClosestPointAndDistance(placement_area.segment, current_pos);
      // 0.6m離れる
      Point target_position = closest_point + (current_pos - closest_point).normalized() * 0.8;
      if (not world_model->point_checker.isFieldInside(target_position, 0.2)) {
        // 一番近いフィールド外のポイントがだめなので逆方向に0.6m離れる
        target_position = closest_point + (closest_point - current_pos).normalized() * 0.8;

        if (
          const auto & segment = placement_area.segment;
          (closest_point == segment.first || closest_point == segment.second)) {
          // 一番近い点が端点の場合は単純に反対側の点を選択するだけではだめなので、
          // 垂直方向に0.6m離れた点を複数選択して、フィールド内かつ配置エリア外の点を選択する
          Vector2 vertical_vec =
            getVerticalVec((segment.second - segment.first).normalized()) * 0.8;
          std::array<Point, 2> target_candidates = {
            closest_point + vertical_vec, closest_point - vertical_vec};

          if (
            auto target = std::ranges::find_if(
              target_candidates,
              [&](const auto & target_candidate) {
                return (
                  world_model->point_checker.isFieldInside(target_candidate, 0.2) &&
                  not isInPlacementArea(target_candidate, 0.1));
              });
            target != target_candidates.end()) {
            target_pos = *target;
          } else {
            // 垂直方向の2候補もだめな場合は放射状8方向で有効点を探索する
            std::array<Point, 8> radial_candidates;
            for (int i = 0; i < 8; i++) {
              double angle = i * M_PI / 4.0;
              radial_candidates[i] = closest_point + Point(std::cos(angle), std::sin(angle)) * 0.8;
            }
            auto valid = std::ranges::find_if(radial_candidates, [&](const auto & c) {
              return world_model->point_checker.isFieldInside(c, 0.2) &&
                     not isInPlacementArea(c, 0.1);
            });
            if (valid != radial_candidates.end()) {
              target_pos = *valid;
            } else {
              // 最終フォールバック: 移動しない
              target_pos = current_pos;
            }
          }
        } else {
          target_pos = target_position;
        }
      } else {
        target_pos = target_position;
      }
      // 安全チェック: 結果がフィールド外にならないようにクランプ
      if (not world_model->point_checker.isFieldInside(target_pos, 0.2)) {
        const double max_x = world_model->fieldSize().x() / 2.0 + FIELD_BOUNDARY_OFFSET;
        const double max_y = world_model->fieldSize().y() / 2.0 + FIELD_BOUNDARY_OFFSET;
        target_pos.x() = std::clamp(target_pos.x(), -max_x, max_x);
        target_pos.y() = std::clamp(target_pos.y(), -max_y, max_y);
      }
    }
  }
}
}  // namespace crane
