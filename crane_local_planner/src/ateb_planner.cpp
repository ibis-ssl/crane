// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/ateb_planner.hpp"

#include <algorithm>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_msg_wrappers/command_wrapper_base.hpp>
#include <crane_msgs/msg/play_situation.hpp>
#include <robocup_ssl_msgs/msg/referee.hpp>
#include <robocup_ssl_msgs/msg/referee_command.hpp>

namespace crane
{

ATEBPlanner::ATEBPlanner(rclcpp::Node & node) : LocalPlannerBase("ateb_planner", node)
{
  node.declare_parameter("max_vel", MAX_VEL);
  MAX_VEL = node.get_parameter("max_vel").as_double();

  node.declare_parameter("stop_state_max_velocity", STOP_STATE_MAX_VELOCITY);
  STOP_STATE_MAX_VELOCITY = node.get_parameter("stop_state_max_velocity").as_double();

  node.declare_parameter("field_boundary_offset", FIELD_BOUNDARY_OFFSET);
  FIELD_BOUNDARY_OFFSET = node.get_parameter("field_boundary_offset").as_double();

  node.declare_parameter("penalty_area_offset", PENALTY_AREA_OFFSET);
  PENALTY_AREA_OFFSET = node.get_parameter("penalty_area_offset").as_double();

  node.declare_parameter("crash_speed_limit", CRASH_SPEED_LIMIT);
  CRASH_SPEED_LIMIT = node.get_parameter("crash_speed_limit").as_double();

  node.declare_parameter("crash_safety_margin", CRASH_SAFETY_MARGIN);
  CRASH_SAFETY_MARGIN = node.get_parameter("crash_safety_margin").as_double();

  node.declare_parameter("crash_avoidance_distance", CRASH_AVOIDANCE_DISTANCE);
  CRASH_AVOIDANCE_DISTANCE = node.get_parameter("crash_avoidance_distance").as_double();

  node.declare_parameter("crash_avoidance_decel_distance", CRASH_AVOIDANCE_DECEL_DISTANCE);
  CRASH_AVOIDANCE_DECEL_DISTANCE = node.get_parameter("crash_avoidance_decel_distance").as_double();

  // 可視グラフの設定
  double ateb_inflation = 0.090;
  node.declare_parameter("ateb_obstacle_inflation", ateb_inflation);
  ateb_inflation = node.get_parameter("ateb_obstacle_inflation").as_double();

  int ateb_max_homotopy = 3;
  node.declare_parameter("ateb_max_homotopy_classes", ateb_max_homotopy);
  ateb_max_homotopy = node.get_parameter("ateb_max_homotopy_classes").as_int();

  ateb::VisibilityGraph::Config vg_cfg;
  vg_cfg.inflation_radius = ateb_inflation;
  vg_cfg.max_homotopy_classes = ateb_max_homotopy;
  visibility_graph_.configure(vg_cfg);

  // 空間最適化器の設定
  int ateb_band_nodes = 15;
  node.declare_parameter("ateb_band_node_count", ateb_band_nodes);
  ateb_band_nodes = node.get_parameter("ateb_band_node_count").as_int();

  int ateb_spatial_iter = 5;
  node.declare_parameter("ateb_spatial_iterations", ateb_spatial_iter);
  ateb_spatial_iter = node.get_parameter("ateb_spatial_iterations").as_int();

  double ateb_smooth_w = 1.0;
  node.declare_parameter("ateb_smoothness_weight", ateb_smooth_w);
  ateb_smooth_w = node.get_parameter("ateb_smoothness_weight").as_double();

  double ateb_obs_w = 10.0;
  node.declare_parameter("ateb_obstacle_weight", ateb_obs_w);
  ateb_obs_w = node.get_parameter("ateb_obstacle_weight").as_double();

  double ateb_len_w = 0.3;
  node.declare_parameter("ateb_path_length_weight", ateb_len_w);
  ateb_len_w = node.get_parameter("ateb_path_length_weight").as_double();

  ateb::SpatialOptimizer::Config so_cfg;
  so_cfg.band_node_count = ateb_band_nodes;
  so_cfg.max_iterations = ateb_spatial_iter;
  so_cfg.smoothness_weight = ateb_smooth_w;
  so_cfg.obstacle_weight = ateb_obs_w;
  so_cfg.path_length_weight = ateb_len_w;
  spatial_optimizer_.configure(so_cfg);

  // 時間パラメータ化の設定
  int ateb_time_samples = 50;
  node.declare_parameter("ateb_time_sample_count", ateb_time_samples);
  ateb_time_samples = node.get_parameter("ateb_time_sample_count").as_int();

  ateb::TimeParameterizer::Config tp_cfg;
  tp_cfg.sample_count = ateb_time_samples;
  tp_cfg.max_velocity = MAX_VEL;
  tp_cfg.max_acceleration = planning_acceleration;
  tp_cfg.max_deceleration = planning_deceleration;
  time_parameterizer_.configure(tp_cfg);

  // CBFフィルタの設定
  double ateb_cbf_alpha = 1.0;
  node.declare_parameter("ateb_cbf_alpha", ateb_cbf_alpha);
  ateb_cbf_alpha = node.get_parameter("ateb_cbf_alpha").as_double();

  double ateb_cbf_margin = 0.03;
  node.declare_parameter("ateb_cbf_safety_margin", ateb_cbf_margin);
  ateb_cbf_margin = node.get_parameter("ateb_cbf_safety_margin").as_double();

  ateb::CBFFilter::Config cbf_cfg;
  cbf_cfg.alpha = ateb_cbf_alpha;
  cbf_cfg.safety_margin = ateb_cbf_margin;
  cbf_filter_.configure(cbf_cfg);

  node.declare_parameter("path_tracking_lookahead", PATH_TRACKING_LOOKAHEAD);
  PATH_TRACKING_LOOKAHEAD = node.get_parameter("path_tracking_lookahead").as_double();

  node.declare_parameter("path_tracking_replan_dist", PATH_TRACKING_REPLAN_DIST);
  PATH_TRACKING_REPLAN_DIST = node.get_parameter("path_tracking_replan_dist").as_double();

  // ロボット状態初期化
  for (uint8_t i = 0; i < 20; ++i) {
    robot_states_[i].robot_id = i;
  }
}

ATEBPlanner::BandProjection ATEBPlanner::projectOnBand(
  const Point & pos, const ateb::ElasticBand & band)
{
  BandProjection result;
  if (band.nodes.size() < 2) {
    result.projected_pos = pos;
    result.tangent = Vector2(1.0, 0.0);
    result.arc_length = 0.0;
    result.total_arc = 0.0;
    return result;
  }

  double best_dist = std::numeric_limits<double>::max();
  double cum_arc = 0.0;

  for (size_t i = 0; i + 1 < band.nodes.size(); ++i) {
    const Point & a = band.nodes[i].pos;
    const Point & b = band.nodes[i + 1].pos;
    const Vector2 ab = b - a;
    const double seg_len = ab.norm();

    if (seg_len < 1e-9) {
      cum_arc += seg_len;
      continue;
    }

    const Vector2 tangent = ab / seg_len;
    const double t = std::clamp((pos - a).dot(ab) / (seg_len * seg_len), 0.0, 1.0);
    const Point proj = a + t * ab;
    const double dist = (pos - proj).norm();

    if (dist < best_dist) {
      best_dist = dist;
      result.projected_pos = proj;
      result.tangent = tangent;
      result.arc_length = cum_arc + t * seg_len;
    }
    cum_arc += seg_len;
  }

  result.total_arc = cum_arc;
  return result;
}

Point ATEBPlanner::getLookaheadPoint(
  const ateb::ElasticBand & band, double arc_from, double lookahead_dist)
{
  const double target_arc = arc_from + lookahead_dist;
  double cum_arc = 0.0;

  for (size_t i = 0; i + 1 < band.nodes.size(); ++i) {
    const Point & a = band.nodes[i].pos;
    const Point & b = band.nodes[i + 1].pos;
    const double seg_len = (b - a).norm();

    if (seg_len < 1e-9) {
      cum_arc += seg_len;
      continue;
    }

    if (cum_arc + seg_len >= target_arc) {
      const double t = (target_arc - cum_arc) / seg_len;
      return a + t * (b - a);
    }
    cum_arc += seg_len;
  }

  return band.nodes.back().pos;
}

double ATEBPlanner::speedAtArcLength(const ateb::TimeOptimalTrajectory & traj, double arc_s)
{
  if (traj.points.size() < 2) {
    return traj.points.empty() ? 0.0 : traj.points.front().vel.norm();
  }

  double cum_arc = 0.0;
  for (size_t i = 1; i < traj.points.size(); ++i) {
    const double ds = (traj.points[i].pos - traj.points[i - 1].pos).norm();
    if (cum_arc + ds >= arc_s) {
      const double alpha = (ds > 1e-9) ? (arc_s - cum_arc) / ds : 0.0;
      return traj.points[i - 1].vel.norm() * (1.0 - alpha) + traj.points[i].vel.norm() * alpha;
    }
    cum_arc += ds;
  }

  return traj.points.back().vel.norm();
}

auto ATEBPlanner::needsExpandedPenaltyAreaOffset(uint8_t cmd) -> bool
{
  using PS = crane_msgs::msg::PlaySituation;
  switch (cmd) {
    case PS::INPLAY:
    case PS::HALT:
    case PS::HALF_TIME:
    case PS::POST_GAME:
      return false;
    default:
      return true;
  }
}

auto ATEBPlanner::buildObstacles(uint8_t ego_id, const crane_msgs::msg::RobotCommand & cmd) const
  -> std::vector<ateb::Obstacle>
{
  const bool include_robots = !cmd.local_planner_config.disable_collision_avoidance;
  const bool include_penalty = !cmd.local_planner_config.disable_goal_area_avoidance;
  const bool include_ball = !cmd.local_planner_config.disable_ball_avoidance;
  const bool include_placement = !cmd.local_planner_config.disable_placement_avoidance;
  const bool include_boundary = !cmd.local_planner_config.disable_field_boundary;

  const int est_size = (include_robots ? 22 : 0) + (include_penalty ? 2 : 0) +
                       (include_ball ? 1 : 0) + (include_placement ? 1 : 0) +
                       (include_boundary ? 4 : 0);
  std::vector<ateb::Obstacle> obstacles;
  obstacles.reserve(est_size);

  if (include_robots) {
    for (const auto & robot : world_model->ours().robots) {
      if (!robot->available() || robot->id == ego_id) continue;
      obstacles.push_back(ateb::Obstacle::makeCircle(robot->pose.pos, robot->geometry().radius));
    }
    for (const auto & robot : world_model->theirs().robots) {
      if (!robot->available()) continue;
      obstacles.push_back(ateb::Obstacle::makeCircle(robot->pose.pos, robot->geometry().radius));
    }
  }

  // ペナルティエリア
  if (include_penalty) {
    const double base_offset =
      needsExpandedPenaltyAreaOffset(world_model->getMsg().play_situation.command.value)
        ? PENALTY_AREA_OFFSET_STOP
        : PENALTY_AREA_OFFSET;
    const double penalty_offset = base_offset - cmd.local_planner_config.penalty_area_contraction;
    const auto our_pa = world_model->getOurPenaltyArea();
    const auto their_pa = world_model->getTheirPenaltyArea();

    // ペナルティエリアをoffset分だけ膨張
    Box our_inflated = our_pa;
    our_inflated.min_corner().x() -= penalty_offset;
    our_inflated.min_corner().y() -= penalty_offset;
    our_inflated.max_corner().x() += penalty_offset;
    our_inflated.max_corner().y() += penalty_offset;

    Box their_inflated = their_pa;
    their_inflated.min_corner().x() -= penalty_offset;
    their_inflated.min_corner().y() -= penalty_offset;
    their_inflated.max_corner().x() += penalty_offset;
    their_inflated.max_corner().y() += penalty_offset;

    // ゴールライン外方向（フィールド外）に5m延長して、PA内からフィールド外側への脱出経路を塞ぐ
    // 各PAのX方向中心を見て、フィールド外側（ゴール側）の辺を延長する
    constexpr double PA_FIELD_EXTENSION = 5.0;
    const double our_center_x =
      (our_inflated.min_corner().x() + our_inflated.max_corner().x()) / 2.0;
    if (our_center_x > 0.0) {
      our_inflated.max_corner().x() += PA_FIELD_EXTENSION;
    } else {
      our_inflated.min_corner().x() -= PA_FIELD_EXTENSION;
    }

    const double their_center_x =
      (their_inflated.min_corner().x() + their_inflated.max_corner().x()) / 2.0;
    if (their_center_x > 0.0) {
      their_inflated.max_corner().x() += PA_FIELD_EXTENSION;
    } else {
      their_inflated.min_corner().x() -= PA_FIELD_EXTENSION;
    }

    // buildObstacles内で既にpenalty_offset分膨張済みのため、VG内での再膨張をスキップする
    auto our_pa_obs = ateb::Obstacle::makeBox(our_inflated);
    our_pa_obs.skip_inflation = true;
    obstacles.push_back(our_pa_obs);

    auto their_pa_obs = ateb::Obstacle::makeBox(their_inflated);
    their_pa_obs.skip_inflation = true;
    obstacles.push_back(their_pa_obs);
  }

  // ボール回避
  if (include_ball) {
    const auto & ball_pos = world_model->ball().pos;
    const double ball_radius = [&]() {
      switch (world_model->getMsg().play_situation.command.value) {
        case crane_msgs::msg::PlaySituation::THEIR_DIRECT_FREE:
          return 0.7;
        case crane_msgs::msg::PlaySituation::STOP:
          return 0.5;
        default:
          return 0.2;
      }
    }();
    obstacles.push_back(ateb::Obstacle::makeCircle(ball_pos, ball_radius));
  }

  // ボール配置エリア
  if (include_placement && world_model->getBallPlacementTarget().has_value()) {
    if (const auto placement_opt = world_model->getBallPlacementArea(); placement_opt.has_value()) {
      obstacles.push_back(ateb::Obstacle::makeCapsule(placement_opt.value()));
    }
  }

  // フィールド境界壁（4辺を太い矩形として追加）
  // バンド中間ノードがフィールド外に最適化されないようにするため
  if (include_boundary) {
    const double hw = world_model->fieldSize().x() / 2.0 + FIELD_BOUNDARY_OFFSET;
    const double hh = world_model->fieldSize().y() / 2.0 + FIELD_BOUNDARY_OFFSET;
    constexpr double kFar = 20.0;

    Box top_wall, bottom_wall, right_wall, left_wall;
    top_wall.min_corner() << -kFar, hh;
    top_wall.max_corner() << kFar, kFar;
    bottom_wall.min_corner() << -kFar, -kFar;
    bottom_wall.max_corner() << kFar, -hh;
    right_wall.min_corner() << hw, -kFar;
    right_wall.max_corner() << kFar, kFar;
    left_wall.min_corner() << -kFar, -kFar;
    left_wall.max_corner() << -hw, kFar;

    obstacles.push_back(ateb::Obstacle::makeBox(top_wall));
    obstacles.push_back(ateb::Obstacle::makeBox(bottom_wall));
    obstacles.push_back(ateb::Obstacle::makeBox(right_wall));
    obstacles.push_back(ateb::Obstacle::makeBox(left_wall));
  }

  return obstacles;
}

auto ATEBPlanner::planSingleRobot(const crane_msgs::msg::RobotCommand & cmd, double theta_offset)
  -> crane_msgs::msg::RobotCommand
{
  crane_msgs::msg::RobotCommand result = cmd;
  result.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
  if (result.polar_velocity_target_mode.empty()) {
    result.polar_velocity_target_mode.emplace_back();
  }

  if (cmd.position_target_mode.empty()) {
    result.polar_velocity_target_mode.front().target_velocity_r = 0.0;
    result.polar_velocity_target_mode.front().target_velocity_theta = theta_offset;
    return result;
  }

  const auto & pos_mode = cmd.position_target_mode.front();
  const Point current_pos(cmd.current_pose.x, cmd.current_pose.y);
  const Point target_pos(pos_mode.target_x, pos_mode.target_y);
  const Vector2 current_vel(cmd.current_velocity.x, cmd.current_velocity.y);

  // 調整後の目標位置をデバッグ用に記録
  addOrUpdatePlanningFactor(
    result, "ATEBTarget",
    "(" + formatPlanningDouble(target_pos.x()) + "," + formatPlanningDouble(target_pos.y()) + ")");

  // 位置許容誤差内なら停止
  const double dist = (target_pos - current_pos).norm();
  if (dist < pos_mode.position_tolerance) {
    result.polar_velocity_target_mode.front().target_velocity_r = 0.0;
    result.polar_velocity_target_mode.front().target_velocity_theta = theta_offset;
    addOrUpdatePlanningFactor(result, "ATEBStatus", "AT_GOAL");
    return result;
  }

  // 速度上限を解決
  result.local_planner_config.max_velocity_factors.emplace_back(
    crane_msgs::msg::NamedFloat()
      .set__name("ATEBPlanner::max_vel")
      .set__value(static_cast<float>(MAX_VEL)));

  const auto referee_command = world_model->getMsg().play_situation.referee_raw.command.value;
  if (referee_command == robocup_ssl_msgs::msg::RefereeCommand::STOP) {
    result.local_planner_config.max_velocity_factors.emplace_back(
      crane_msgs::msg::NamedFloat()
        .set__name("ATEBPlanner STOP制限")
        .set__value(static_cast<float>(STOP_STATE_MAX_VELOCITY)));
  }

  const double max_vel = resolveMaxVelocityFactors(result, static_cast<float>(MAX_VEL));
  const double max_acc =
    resolveMaxAccelerationFactors(result, static_cast<float>(planning_acceleration));

  const auto obstacles = buildObstacles(cmd.robot_id, cmd);

  auto & state = robot_states_[cmd.robot_id];

  // ウォームスタート判定: 目標位置が大きく変化した場合は再計画
  const bool goal_changed = (target_pos - state.cached_goal).norm() > REPLAN_THRESHOLD;
  if (goal_changed) {
    state.warm_start_valid = false;
    state.cached_goal = target_pos;
  }

  Vector2 output_vel;

  {
    // Phase 1: 可視グラフでホモトピークラスを抽出
    // disable_collision_avoidance=true でもPA等静的障害物は回避する
    // （ロボット障害物はbuildObstacles()で除外済み）
    std::vector<ateb::HomotopyClass> homotopies;
    if (!state.warm_start_valid) {
      homotopies = visibility_graph_.extract(current_pos, target_pos, obstacles);
      state.cached_homotopies = homotopies;
    } else {
      homotopies = state.cached_homotopies;
    }

    // Phase 2: 各ホモトピークラスのバンドを最適化して最良を選ぶ
    ateb::ElasticBand best_band;
    if (state.warm_start_valid && state.best_band.isValid()) {
      // ウォームスタート時: 現在位置をバンドに射影して始点とする
      // （現在位置をそのまま始点にすると経路がロボットを追いかけてしまうため）
      const auto proj = projectOnBand(current_pos, state.best_band);
      const double cross_track_dist = (current_pos - proj.projected_pos).norm();

      if (cross_track_dist > PATH_TRACKING_REPLAN_DIST) {
        // 横偏差が閾値超え → ウォームスタート破棄して再計画
        state.warm_start_valid = false;
        for (const auto & homotopy : homotopies) {
          ateb::ElasticBand band =
            spatial_optimizer_.optimize(homotopy, current_pos, target_pos, obstacles);
          if (band.total_cost < best_band.total_cost) {
            best_band = band;
          }
        }
      } else {
        // 射影点を始点に設定してreoptimize（経路を現在位置に引き寄せない）
        state.best_band.nodes.front().pos = proj.projected_pos;
        state.best_band.nodes.back().pos = target_pos;
        best_band = spatial_optimizer_.reoptimize(state.best_band, obstacles);
      }
    } else {
      for (const auto & homotopy : homotopies) {
        ateb::ElasticBand band =
          spatial_optimizer_.optimize(homotopy, current_pos, target_pos, obstacles);
        if (band.total_cost < best_band.total_cost) {
          best_band = band;
        }
      }
    }

    state.best_band = best_band;
    state.warm_start_valid = true;

    // Phase 3: 経路追従制御（Pure Pursuit + 時間最適速度プロファイル）
    //
    // 1. 現在位置をバンドに射影して弧長を求める
    // 2. TimeParameterizerで速度プロファイルを生成し、射影点の弧長での速度を得る
    // 3. 先読み点（lookahead）へのPure Pursuitで速度方向を決定
    //    → 接線方向追従と横偏差補正を両立する
    {
      if (best_band.nodes.size() >= 2) {
        // バンドへの射影
        const auto proj = projectOnBand(current_pos, best_band);

        // 速度プロファイル生成（初速: 現在速度、終速: 0）
        const auto trajectory = time_parameterizer_.parameterize(
          best_band, current_vel, Vector2::Zero(), max_vel, max_acc, planning_deceleration);

        // 先読み点の弧長位置で速度を取得する
        // （射影点（≈現在位置）の速度は停止時に0になり、ロボットが動き出せないため）
        const double lookahead_arc =
          std::min(proj.arc_length + PATH_TRACKING_LOOKAHEAD, proj.total_arc);
        const double remaining_dist = proj.total_arc - proj.arc_length;

        // 速度の大きさを2つのソースから決定:
        // 1. 時間最適プロファイル: 曲率・加速度制約を反映した上限速度
        // 2. 減速則: 残り距離に基づく滑らかな減速 sqrt(2*a*d)
        double profile_speed = max_vel;
        if (trajectory.isValid() && lookahead_arc < proj.total_arc) {
          // 先読み点がバンド内にある場合のみプロファイル速度を参照
          // （先読みが終端を超えると終端速度=0が返り、早期停止の原因になるため）
          profile_speed = speedAtArcLength(trajectory, lookahead_arc);
        }
        const double decel_speed =
          std::min(max_vel, std::sqrt(2.0 * planning_deceleration * remaining_dist));
        const double speed = std::min(profile_speed, decel_speed);

        // Pure Pursuit: 先読み点へのベクトルを速度方向とする
        const Point lookahead =
          getLookaheadPoint(best_band, proj.arc_length, PATH_TRACKING_LOOKAHEAD);
        const Vector2 to_lookahead = lookahead - current_pos;
        const double d_lookahead = to_lookahead.norm();
        const Vector2 direction =
          (d_lookahead > 1e-6) ? Vector2(to_lookahead / d_lookahead) : proj.tangent;

        output_vel = direction * speed;
      } else {
        const double v = std::min(max_vel, std::sqrt(2.0 * planning_deceleration * dist));
        const Vector2 diff = target_pos - current_pos;
        const double d = diff.norm();
        output_vel = (d > 1e-6) ? Vector2(diff.normalized() * v) : Vector2::Zero();
      }
    }

    // Phase 4: CBF安全フィルタ
    if (!cmd.local_planner_config.disable_crash_avoidance) {
      std::vector<std::pair<Point, Vector2>> dynamic_obs;
      for (const auto & robot : world_model->theirs().robots) {
        if (robot->available()) {
          dynamic_obs.emplace_back(robot->pose.pos, robot->vel.linear);
        }
      }
      for (const auto & robot : world_model->ours().robots) {
        if (robot->available() && robot->id != cmd.robot_id) {
          dynamic_obs.emplace_back(robot->pose.pos, robot->vel.linear);
        }
      }

      // 静的障害物はペナルティエリアのみ（ロボット動的障害物はdynamic_obsで渡す）
      // フィールド境界壁（kFar=20のBOX）はCBFに不要なため除外する
      std::vector<ateb::Obstacle> static_obs;
      if (!cmd.local_planner_config.disable_goal_area_avoidance) {
        const double hw = world_model->fieldSize().x() / 2.0 + FIELD_BOUNDARY_OFFSET;
        const double hh = world_model->fieldSize().y() / 2.0 + FIELD_BOUNDARY_OFFSET;
        for (const auto & obs : obstacles) {
          if (obs.type != ateb::Obstacle::Type::BOX) continue;
          // フィールド境界壁は max_corner が非常に大きい（kFar=20）ので除外
          const double xmax = std::abs(obs.box.max_corner().x());
          const double ymax = std::abs(obs.box.max_corner().y());
          if (xmax > hw + 1.0 || ymax > hh + 1.0) continue;
          static_obs.push_back(obs);
        }
      }

      output_vel = cbf_filter_.filter(current_pos, output_vel, dynamic_obs, static_obs);
    }

    addOrUpdatePlanningFactor(result, "ATEBStatus", "OK");
    addOrUpdatePlanningFactor(result, "ATEBOutputSpeed", formatPlanningDouble(output_vel.norm()));
    addOrUpdatePlanningFactor(
      result, "ATEBObstacles", std::to_string(static_cast<int>(obstacles.size())));
    if (best_band.nodes.size() >= 2) {
      const auto & n1 = best_band.nodes[1];
      addOrUpdatePlanningFactor(
        result, "ATEBBand1stNode",
        "(" + formatPlanningDouble(n1.pos.x()) + "," + formatPlanningDouble(n1.pos.y()) + ")");
    }

    // --- 可視化 ---
    // ホモトピー候補経路（薄いグレーのポリライン）
    for (const auto & homotopy : homotopies) {
      if (homotopy.waypoints.size() >= 2) {
        visualizer->drawPolyline(homotopy.waypoints, "gray", 0.4, 8.0);
      }
    }

    // 弾性バンド経路（シアンのポリライン）
    if (best_band.nodes.size() >= 2) {
      std::vector<Point> band_pts;
      band_pts.reserve(best_band.nodes.size());
      for (const auto & node : best_band.nodes) {
        band_pts.push_back(node.pos);
      }
      visualizer->drawPolyline(band_pts, "cyan", 0.8, 12.0);

      // 中間ノードを小円で描画
      for (size_t i = 1; i + 1 < best_band.nodes.size(); ++i) {
        visualizer->drawFilledCircle(best_band.nodes[i].pos, 0.03, "cyan", 0.7);
      }
    }
  }

  // 速度をクランプ
  const double vel_norm = output_vel.norm();
  if (vel_norm > max_vel) {
    output_vel *= max_vel / vel_norm;
  }

  // クラッシュ回避（SSLルール準拠）
  if (!cmd.local_planner_config.disable_crash_avoidance) {
    const Point our_pos = current_pos;
    for (const auto & enemy_robot : world_model->theirs().robotsWhere().available().get()) {
      const Vector2 to_enemy = enemy_robot->pose.pos - our_pos;
      const double d = to_enemy.norm();

      if (d < CRASH_AVOIDANCE_DISTANCE && d > 0.01) {
        const Vector2 dir = to_enemy.normalized();
        const double enemy_approach = -enemy_robot->vel.linear.dot(dir);
        double safe_approach =
          CRASH_SPEED_LIMIT - std::max(0.0, enemy_approach) - CRASH_SAFETY_MARGIN;
        safe_approach = std::max(safe_approach, 0.0);

        const double factor = 1.0 - std::clamp(
                                      (d - CRASH_AVOIDANCE_DECEL_DISTANCE) /
                                        (CRASH_AVOIDANCE_DISTANCE - CRASH_AVOIDANCE_DECEL_DISTANCE),
                                      0.0, 1.0);

        const double approach_component = output_vel.dot(dir);
        const double max_approach = approach_component * (1.0 - factor) + safe_approach * factor;

        if (approach_component > max_approach && approach_component > 0.0) {
          const Vector2 lateral = output_vel - approach_component * dir;
          output_vel = lateral + max_approach * dir;
        }
      }
    }
  }

  // 回転制御（加速初期の回転停止）
  if (cmd.local_planner_config.enable_rotation_stop_on_accel) {
    const double move_angle = std::atan2(output_vel.y(), output_vel.x());
    const double angle_diff = normalizeAngle(cmd.current_pose.theta - move_angle);

    constexpr double ANGLE_THRESHOLD = 15.0 * M_PI / 180.0;
    const bool is_forward_or_backward = (std::abs(angle_diff) <= ANGLE_THRESHOLD) ||
                                        (std::abs(std::abs(angle_diff) - M_PI) <= ANGLE_THRESHOLD);

    const double current_speed = current_vel.norm();
    const double target_speed = output_vel.norm();
    const bool is_accelerating = current_speed < target_speed;
    const bool is_low_speed = current_speed <= max_vel * 0.5;

    if (is_forward_or_backward && is_low_speed && is_accelerating && target_speed > 0.01) {
      result.omega_limit = 0.0;
    }
  }

  result.polar_velocity_target_mode.front().target_velocity_r = output_vel.norm();
  result.polar_velocity_target_mode.front().target_velocity_theta =
    std::atan2(output_vel.y(), output_vel.x()) + theta_offset;

  // 速度ベクトル（黄色矢印）と目標位置（白円）を描画
  visualizer->velocityArrow(current_pos, output_vel, "yellow");
  visualizer->drawCircle(target_pos, 0.06, "white", 8.0, 0.7);

  // 経路追従デバッグ: バンドへの射影点（緑）と先読み点（オレンジ）を描画
  if (state.best_band.isValid()) {
    const auto proj = projectOnBand(current_pos, state.best_band);
    visualizer->drawFilledCircle(proj.projected_pos, 0.04, "green", 0.8);
    const Point lookahead =
      getLookaheadPoint(state.best_band, proj.arc_length, PATH_TRACKING_LOOKAHEAD);
    visualizer->drawFilledCircle(lookahead, 0.04, "orange", 0.8);
  }

  return result;
}

auto ATEBPlanner::calculateRobotCommand(
  const crane_msgs::msg::RobotCommands & msg, double theta_offset) -> crane_msgs::msg::RobotCommands
{
  crane_msgs::msg::RobotCommands commands = msg;

  const auto referee_command = world_model->getMsg().play_situation.referee_raw.command.value;
  if (referee_command != robocup_ssl_msgs::msg::RefereeCommand::HALT) {
    overrideTargetPosition(commands);
  }

  crane_msgs::msg::RobotCommands result;
  result.header = msg.header;
  result.on_positive_half = msg.on_positive_half;
  result.is_yellow = msg.is_yellow;

  for (const auto & cmd : commands.robot_commands) {
    result.robot_commands.push_back(planSingleRobot(cmd, theta_offset));
  }

  return result;
}

namespace
{
auto pointChanged(const Point & before, const Point & after, double epsilon = 1e-4) -> bool
{
  return (before - after).norm() > epsilon;
}
}  // namespace

auto ATEBPlanner::overrideTargetPosition(crane_msgs::msg::RobotCommands & msg) -> void
{
  for (auto & command : msg.robot_commands) {
    if (command.position_target_mode.empty()) {
      continue;
    }
    auto & pos_mode = command.position_target_mode.front();

    Point target_pos;
    target_pos << pos_mode.target_x, pos_mode.target_y;

    const Point current_pos(command.current_pose.x, command.current_pose.y);
    if (std::isnan(target_pos.x()) || std::isnan(target_pos.y())) {
      target_pos = current_pos;
      pos_mode.target_x = target_pos.x();
      pos_mode.target_y = target_pos.y();
      addOrUpdatePlanningFactor(command, "ATEBTargetFallback", "NAN_TARGET");
      continue;
    }

    if (std::isnan(current_pos.x()) || std::isnan(current_pos.y())) {
      addOrUpdatePlanningFactor(command, "ATEBTargetFallback", "NAN_CURRENT");
      continue;
    }

    Point before = target_pos;
    adjustForFieldBoundary(target_pos, current_pos, command);
    if (pointChanged(before, target_pos)) {
      addOrUpdatePlanningFactor(command, "ATEBAdjustFieldBoundary", "1");
    }

    // 衝突回避が有効な場合はVGがPA（BOX障害物）を自律的に回避するため、
    // アドホックなコーナーウェイポイントロジックは不要
    if (command.local_planner_config.disable_collision_avoidance) {
      before = target_pos;
      adjustForPenaltyAreaAvoidance(target_pos, current_pos, command);
      if (pointChanged(before, target_pos)) {
        addOrUpdatePlanningFactor(command, "ATEBAdjustPenaltyArea", "1");
      }
    }

    before = target_pos;
    adjustForBallAvoidance(target_pos, current_pos, command);
    if (pointChanged(before, target_pos)) {
      addOrUpdatePlanningFactor(command, "ATEBAdjustBallAvoidance", "1");
    }

    before = target_pos;
    adjustForPlacementAvoidance(target_pos, current_pos, command);
    if (pointChanged(before, target_pos)) {
      addOrUpdatePlanningFactor(command, "ATEBAdjustPlacementAvoidance", "1");
    }

    pos_mode.target_x = target_pos.x();
    pos_mode.target_y = target_pos.y();
  }
}

auto ATEBPlanner::adjustForFieldBoundary(
  Point & target_pos, const Point & current_pos,
  const crane_msgs::msg::RobotCommand & command) const -> void
{
  if (command.local_planner_config.disable_field_boundary) {
    return;
  }
  const double max_x = world_model->fieldSize().x() / 2.0 + FIELD_BOUNDARY_OFFSET;
  const double max_y = world_model->fieldSize().y() / 2.0 + FIELD_BOUNDARY_OFFSET;

  Box field_box;
  field_box.min_corner() << -max_x, -max_y;
  field_box.max_corner() << max_x, max_y;

  if (isInBox(field_box, target_pos)) {
    return;
  }

  Segment move_line(current_pos, target_pos);
  Segment top_edge(Point(-max_x, max_y), Point(max_x, max_y));
  Segment bottom_edge(Point(-max_x, -max_y), Point(max_x, -max_y));
  Segment right_edge(Point(max_x, -max_y), Point(max_x, max_y));
  Segment left_edge(Point(-max_x, -max_y), Point(-max_x, max_y));

  std::vector<Point> all_intersections;
  for (const auto & edge : {top_edge, bottom_edge, right_edge, left_edge}) {
    auto intersections = getIntersections(move_line, edge);
    all_intersections.insert(all_intersections.end(), intersections.begin(), intersections.end());
  }

  if (!all_intersections.empty()) {
    auto closest = std::min_element(
      all_intersections.begin(), all_intersections.end(),
      [&current_pos](const Point & a, const Point & b) {
        return bg::distance(a, current_pos) < bg::distance(b, current_pos);
      });
    target_pos = *closest;
  } else {
    target_pos.x() = std::clamp(target_pos.x(), -max_x, max_x);
    target_pos.y() = std::clamp(target_pos.y(), -max_y, max_y);
  }
}

auto ATEBPlanner::adjustForPenaltyAreaAvoidance(
  Point & target_pos, const Point & current_pos,
  const crane_msgs::msg::RobotCommand & command) const -> void
{
  if (not command.local_planner_config.disable_goal_area_avoidance) {
    constexpr double SURROUNDING_OFFSET = 0.2;
    const double penalty_area_offset =
      needsExpandedPenaltyAreaOffset(world_model->getMsg().play_situation.command.value)
        ? PENALTY_AREA_OFFSET_STOP
        : PENALTY_AREA_OFFSET;

    auto avoidPenaltyArea = [&](const Box & penalty_area, const Point & goal_pos) {
      constexpr int MAX_ITERATIONS = 100;
      if (isInBox(penalty_area, current_pos, penalty_area_offset)) {
        if (std::abs(current_pos.x()) > world_model->fieldSize().x() / 2.0) {
          target_pos.x() = std::copysign(world_model->fieldSize().x() / 2.0, target_pos.x());
        }
        if ((target_pos - current_pos).norm() < 1e-6) {
          target_pos = current_pos + (current_pos - goal_pos).normalized() * 0.05;
        }
        for (int iter = 0;
             iter < MAX_ITERATIONS && isInBox(penalty_area, target_pos, penalty_area_offset);
             ++iter) {
          target_pos += (target_pos - current_pos).normalized() * 0.05;
        }
      } else if (isInBox(penalty_area, target_pos, penalty_area_offset)) {
        if (std::abs(target_pos.x()) > world_model->fieldSize().x() / 2.0) {
          target_pos.x() = std::copysign(world_model->fieldSize().x() / 2.0, target_pos.x());
        }
        for (int iter = 0;
             iter < MAX_ITERATIONS && isInBox(penalty_area, target_pos, penalty_area_offset) &&
             target_pos != goal_pos;
             ++iter) {
          target_pos += (target_pos - goal_pos).normalized() * 0.05;
        }
      }
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

        // 現在位置→コーナーの経路がPAを通過しないかつ未通過のコーナーを選ぶ
        const bool c1_accessible =
          !bg::intersects(Segment(current_pos, around_corner_1), penalty_area);
        const bool c2_accessible =
          !bg::intersects(Segment(current_pos, around_corner_2), penalty_area);
        // コーナー通過判定: ロボットがコーナーをターゲット方向に抜けているかどうか
        // ドット積 > 0 → ロボットがコーナーよりもターゲット側にいる（通過済み）
        // 距離閾値による判定では、コーナー手前0.5mで「通過済み」と誤判定し
        // 反対側コーナーへのターゲット切り替えが発生して振動が生じるため使用しない
        const bool c1_passed =
          (target_pos - around_corner_1).dot(current_pos - around_corner_1) > 0;
        const bool c2_passed =
          (target_pos - around_corner_2).dot(current_pos - around_corner_2) > 0;

        bool use_c1 = c1_accessible && !c1_passed;
        bool use_c2 = c2_accessible && !c2_passed;
        if (use_c1 && use_c2) {
          // 両方有効な場合は経路長の短い方
          if (dist_via_1 <= dist_via_2) {
            use_c2 = false;
          } else {
            use_c1 = false;
          }
        }

        if (use_c1) {
          target_pos = around_corner_1;
        } else if (use_c2) {
          target_pos = around_corner_2;
        }
        // どちらも無効な場合はそのまま（PAから抜け出るまで待つ）
      }
    };

    avoidPenaltyArea(world_model->getOurPenaltyArea(), world_model->getOurGoalCenter());
    avoidPenaltyArea(world_model->getTheirPenaltyArea(), world_model->getTheirGoalCenter());
  }
}

auto ATEBPlanner::adjustForBallAvoidance(
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
      target_pos = ball_pos + (current_pos - ball_pos).normalized() * (MIN_BALL_DISTANCE + 0.05);
    } else {
      Segment move_line(current_pos, target_pos);
      auto [distance, closest_point] = getClosestPointAndDistance(ball_pos, move_line);
      if (
        closest_point != ball_pos && closest_point != target_pos && distance < MIN_BALL_DISTANCE) {
        target_pos = ball_pos + (closest_point - ball_pos).normalized() * MIN_BALL_DISTANCE;
      }
    }
  }
}

auto ATEBPlanner::adjustForPlacementAvoidance(
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
      Point target_position = closest_point + (current_pos - closest_point).normalized() * 0.8;
      if (not world_model->point_checker.isFieldInside(target_position, 0.2)) {
        target_position = closest_point + (closest_point - current_pos).normalized() * 0.8;

        if (
          const auto & segment = placement_area.segment;
          (closest_point == segment.first || closest_point == segment.second)) {
          Vector2 vertical_vec =
            getVerticalVec((segment.second - segment.first).normalized()) * 0.8;
          std::array<Point, 2> target_candidates = {
            closest_point + vertical_vec, closest_point - vertical_vec};

          if (
            auto target = std::ranges::find_if(
              target_candidates,
              [&](const auto & tc) {
                return (
                  world_model->point_checker.isFieldInside(tc, 0.2) &&
                  not isInPlacementArea(tc, 0.1));
              });
            target != target_candidates.end()) {
            target_pos = *target;
          } else {
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
              target_pos = current_pos;
            }
          }
        } else {
          target_pos = target_position;
        }
      } else {
        target_pos = target_position;
      }
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
