// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_physics/ball_physics_model.hpp>
#include <crane_physics/kicker_model.hpp>
#include <crane_utils/package.hpp>
#include <crane_utils/parameter.hpp>
#include <filesystem>
#include <format>
#include <rclcpp/rclcpp.hpp>

#include "crane_game_analyzer/game_analyzer.hpp"
#include "crane_game_analyzer/metrics/attacker_metrics.hpp"
#include "crane_game_analyzer/metrics/ball_horizon_metric.hpp"
#include "crane_game_analyzer/metrics/ongoing_kick_metric.hpp"
#include "crane_game_analyzer/metrics/pass_plan_metrics.hpp"
#include "crane_game_analyzer/metrics/pass_target_metrics.hpp"
#include "crane_game_analyzer/metrics/slack_metrics.hpp"
#include "crane_game_analyzer/metrics/sub_attacker_metrics.hpp"
#include "crane_game_analyzer/metrics/threat_metrics.hpp"
#include "crane_game_analyzer/threat_evaluator.hpp"

namespace crane
{

GameAnalyzerComponent::GameAnalyzerComponent(const rclcpp::NodeOptions & options)
: Node("crane_game_analyzer", options),
  visualizer(std::make_shared<VisualizerMessageBuilder>("analyzer"))
{
  RCLCPP_INFO(get_logger(), "GameAnalyzer is constructed.");

  // パラメータの設定と読み込み
  config.ball_idle.threshold_duration = rclcpp::Duration::from_seconds(
    crane::get_or_declare_parameter(this, "ball_idle.threshold_duration", 5.0));
  config.ball_idle.move_distance_threshold_meter =
    crane::get_or_declare_parameter(this, "ball_idle.move_distance_threshold_meter", 0.05);
  config.robot_collision.velocity_threshold =
    crane::get_or_declare_parameter(this, "robot_collision.velocity_threshold", 1.0);
  config.robot_collision.distance_threshold =
    crane::get_or_declare_parameter(this, "robot_collision.distance_threshold", 0.2);
  config.robot_collision.time_window =
    crane::get_or_declare_parameter(this, "robot_collision.time_window", 0.5);
  RCLCPP_DEBUG(
    get_logger(), "  - Velocity threshold: %.2f m/s", config.robot_collision.velocity_threshold);
  RCLCPP_DEBUG(
    get_logger(), "  - Distance threshold: %.2f m", config.robot_collision.distance_threshold);
  RCLCPP_DEBUG(get_logger(), "  - Time window: %.2f s", config.robot_collision.time_window);

  CraneVisualizerBuffer::activate(*this);

  world_model = std::make_unique<WorldModelWrapper>(*this);
  kick_event_detector_ = std::make_unique<KickEventDetector>();

  // キック予測モデル初期化（ongoing_kickの予測トレース生成に使用）
  std::string ball_physics_config_path =
    crane::get_or_declare_parameter(this, "ball_physics_config_path", "");
  std::shared_ptr<BallPhysicsModel> ball_physics_model;
  if (!ball_physics_config_path.empty()) {
    auto full_config_path = crane::resolve_package_path(
      get_logger(), "crane_world_model_publisher", ball_physics_config_path);
    try {
      ball_physics_model = BallPhysicsModelFactory::createWithYAMLConfig(full_config_path);
      RCLCPP_INFO(get_logger(), "ボール物理設定を読み込みました: %s", full_config_path.c_str());
    } catch (const std::exception & ex) {
      RCLCPP_WARN(
        get_logger(), "ボール物理設定の読み込みに失敗 (%s): %s。デフォルト設定を使用します",
        full_config_path.c_str(), ex.what());
      ball_physics_model = BallPhysicsModelFactory::getInstance();
    }
  } else {
    ball_physics_model = BallPhysicsModelFactory::getInstance();
  }

  std::string kicker_physics_config_path =
    crane::get_or_declare_parameter(this, "kicker_physics_config_path", "");
  std::shared_ptr<KickerModel> kicker_model;
  if (!kicker_physics_config_path.empty()) {
    auto full_config_path = crane::resolve_package_path(
      get_logger(), "crane_world_model_publisher", kicker_physics_config_path);
    try {
      kicker_model = createIntegratedKickerModel(full_config_path, ball_physics_model);
      RCLCPP_INFO(get_logger(), "キッカー物理設定を読み込みました: %s", full_config_path.c_str());
    } catch (const std::exception & ex) {
      RCLCPP_WARN(
        get_logger(), "キッカー物理設定の読み込みに失敗 (%s): %s。デフォルト設定を使用します",
        full_config_path.c_str(), ex.what());
      kicker_model = std::make_shared<KickerModel>();
      kicker_model->setBallPhysicsModel(ball_physics_model);
    }
  } else {
    kicker_model = std::make_shared<KickerModel>();
    kicker_model->setBallPhysicsModel(ball_physics_model);
  }

  kick_event_detector_->setKickerModel(kicker_model);
  sub_robot_commands_ = create_subscription<crane_msgs::msg::RobotCommands>(
    "/robot_commands", 10, [this](const crane_msgs::msg::RobotCommands::SharedPtr msg) {
      kick_event_detector_->updateRobotCommands(*msg);
    });

  // 脅威評価結果のパブリッシャー
  game_analysis_pub_ = create_publisher<crane_msgs::msg::GameAnalysis>("game_analysis", 10);

  // キック予実トレース（実績記録済みの完了トレースのみ）のパブリッシャー
  kick_prediction_trace_pub_ =
    create_publisher<crane_msgs::msg::KickPredictionTrace>("kick_prediction_traces", 10);

  // メトリクス計算エンジンの初期化
  metric_engine_ = std::make_unique<metrics::MetricEngine>(get_logger());

  // 基礎メトリクス
  metric_engine_->registerMetric(std::make_shared<metrics::BallHorizonMetric>());
  metric_engine_->registerMetric(std::make_shared<metrics::OurSlackMetric>());
  metric_engine_->registerMetric(std::make_shared<metrics::TheirSlackMetric>());
  metric_engine_->registerMetric(std::make_shared<metrics::OngoingKickMetric>());

  // 脅威評価メトリクス（共有ThreatEvaluatorインスタンス）
  auto shared_threat_evaluator = std::make_shared<ThreatEvaluator>(ThreatEvaluatorConfig{});

  auto ball_threat_metric = std::make_shared<metrics::BallThreatMetric>(shared_threat_evaluator);
  metric_engine_->registerMetric(ball_threat_metric);

  auto robot_threats_metric =
    std::make_shared<metrics::RobotThreatsMetric>(ball_threat_metric, shared_threat_evaluator);
  metric_engine_->registerMetric(robot_threats_metric);

  metric_engine_->registerMetric(
    std::make_shared<metrics::RecommendedDefendersMetric>(
      ball_threat_metric, robot_threats_metric, shared_threat_evaluator));

  // 役割決定メトリクス（新規）
  auto attacker_metric = std::make_shared<metrics::AttackerCandidateMetric>();
  metric_engine_->registerMetric(attacker_metric);

  auto sub_attacker_position_metric = std::make_shared<metrics::SubAttackerPositionMetric>();
  metric_engine_->registerMetric(sub_attacker_position_metric);

  // パスターゲット選定メトリクス
  auto pass_target_metric = std::make_shared<metrics::PassTargetMetric>();
  // パラメータ設定
  double min_hold = crane::get_or_declare_parameter(this, "pass_target.min_hold_duration_sec", 0.5);
  double min_improve =
    crane::get_or_declare_parameter(this, "pass_target.min_improvement_margin", 0.2);
  pass_target_metric->setHysteresisParams(min_hold, min_improve);
  metric_engine_->registerMetric(pass_target_metric);

  // 通常プレーのパス計画（出し手・受け手で共有）
  auto pass_plan_metric = std::make_shared<metrics::PassPlanMetric>();
  declare_parameter("pass_plan.recompute_interval_sec", 0.1);
  // 旧経路（Attacker の MIN_PASS_SCORE_ATTACKER）と同じ実効閾値にそろえる。
  // スコア尺度は combinePassScore で共通なので、生産側だけ 0.5 に上げると
  // 実質的にパス受理基準を 0.2 から 0.5 へ引き上げたことになる。
  // 実測では構造の異なる3配置すべてで最良スコアが 0.13〜0.46 に留まり、
  // 0.5 では計画がまず成立しなかった。
  // 到達しにくい主因は score の構成にある:
  //   score = (距離 + ゴール角 − 自ゴール) × 敵ゴール × 迎撃 × 遮蔽
  // 自ゴールペナルティは pass_rating.cpp が遮蔽ロボットを空リストで評価するため
  // 純粋な幾何値で、自ゴールから約6.8m以内は常に上限 0.5 に張り付く。
  declare_parameter("pass_plan.min_pass_score", 0.2);
  // 計画保持中の下限緩和率（シュミットトリガ）。閾値ぎわでの明滅を防ぐ。
  declare_parameter("pass_plan.min_pass_score_release_ratio", 0.6);
  // 受領点に要求するフィールド境界からの余裕 [m]
  declare_parameter("pass_plan.receive_point_field_margin", 0.5);
  declare_parameter("pass_plan.max_candidates", 800);
  declare_parameter("pass_plan.dpps_r_resolution", 0.3);
  declare_parameter("pass_plan.dpps_r_max", 2.5);
  declare_parameter("pass_plan.dpps_theta_div", 16);
  // 受け手の運動能力は local planner が INPLAY 中に解決する上限（planning_acceleration
  // 5.0 / max_vel 5.0）と揃える。3.0/4.0 では自軍の受け手を指令上限より 4 割遅く見積もり、
  // 敵の迎撃モデル（3.0/5.5・マージンなし）より不利な扱いになっていた。
  // 実測（パス距離 3.0m・飛行 1.49s）では受け手の到達半径 1.05m に対し敵 1.65m。
  // 揃えると受け手 1.76m となり、ようやく敵と同等以上になる。
  declare_parameter("pass_plan.receiver_max_acceleration", 5.0);
  declare_parameter("pass_plan.receiver_max_velocity", 5.0);
  // 受領点での到達速度。詳細な実測値は ReceiveFeasibilityParams のコメント参照。
  // 減速度が実測 0.36 m/s^2 と小さいため、1.5 では受領点通過後 3.1m 転がる。
  declare_parameter("pass_plan.desired_arrival_speed", 1.0);
  // キック初速の下限。
  //
  // この下限は短いパスで desired_arrival_speed の設計を上書きする。実測
  // （受領点まで 1.99m）では、到達速度 1.0 で届く初速 1.56 m/s が 2.00 へ
  // クランプされ、受領点での速度が 1.60 m/s になる。通過後 3.56m 転がるので、
  // 受け手が捕り損ねると場外まで出る。
  //
  // それでも 2.0 を維持する。1.5 に下げて 15 試行を測ったところ、到達速度は
  // 設計どおり 1.0 に乗った（検算 1.04/1.07/1.00）が、成功率は 7/15 から
  // 4/15 へ落ちた。原因は出し手による再接触で、SELF_TOUCH が 1 件から 6 件に
  // 増えた。5 m/s で動けるロボットにとって 1.7 m/s の減速するボールは追い
  // つける速さで、遅いパスは出し手自身に取り戻される。
  //
  // 転がり過ぎを減らすなら、初速を下げるのではなく BALL_IN_FLIGHT 中の
  // 出し手の振る舞いを直す方が筋が良い（AttackerSkillSession は計画が
  // BALL_IN_FLIGHT のときだけ停止するので、キックで計画が解除されると
  // そのまま追いかける）。
  declare_parameter("pass_plan.min_initial_speed", 2.0);
  declare_parameter("pass_plan.max_initial_speed", 5.5);
  declare_parameter("pass_plan.feasibility_margin", 0.3);
  pass_plan_metric->setRecomputeInterval(
    get_parameter("pass_plan.recompute_interval_sec").as_double());
  pass_plan_metric->setMinPassScore(get_parameter("pass_plan.min_pass_score").as_double());
  pass_plan_metric->setMinPassScoreReleaseRatio(
    get_parameter("pass_plan.min_pass_score_release_ratio").as_double());
  pass_plan_metric->setReceivePointFieldMargin(
    get_parameter("pass_plan.receive_point_field_margin").as_double());
  pass_plan_metric->setMaxCandidates(
    static_cast<int>(get_parameter("pass_plan.max_candidates").as_int()));
  pass_plan_metric->setDppsParams(
    get_parameter("pass_plan.dpps_r_resolution").as_double(),
    get_parameter("pass_plan.dpps_r_max").as_double(),
    static_cast<int>(get_parameter("pass_plan.dpps_theta_div").as_int()));
  {
    ReceiveFeasibilityParams fp;
    fp.receiver_max_acceleration = get_parameter("pass_plan.receiver_max_acceleration").as_double();
    fp.receiver_max_velocity = get_parameter("pass_plan.receiver_max_velocity").as_double();
    fp.desired_arrival_speed = get_parameter("pass_plan.desired_arrival_speed").as_double();
    fp.min_initial_speed = get_parameter("pass_plan.min_initial_speed").as_double();
    fp.max_initial_speed = get_parameter("pass_plan.max_initial_speed").as_double();
    fp.margin = get_parameter("pass_plan.feasibility_margin").as_double();
    pass_plan_metric->setFeasibilityParams(fp);
  }
  metric_engine_->registerMetric(pass_plan_metric);

  // メトリクスエンジン初期化（トポロジカルソート・循環依存検出）
  if (!metric_engine_->initialize()) {
    RCLCPP_FATAL(get_logger(), "Failed to initialize metric engine!");
    throw std::runtime_error("Metric engine initialization failed");
  }

  world_model->addCallback([&]() {
    auto robot_collision_info = getRobotCollisionInfo();

    if (robot_collision_info) {
      //          robot_collision_info->attack_robot.robot_id
      RCLCPP_DEBUG(
        get_logger(), "Collision Detected : ( %d, %d ) , %f [m/s]",
        robot_collision_info->attack_robot.id, robot_collision_info->attacked_robot.id,
        robot_collision_info->relative_velocity);
    }

    // ボール履歴を更新
    crane_msgs::msg::BallInfo ball_info_msg;
    world_model->ball().toMsg(ball_info_msg);
    ball_history_.push_front(ball_info_msg);
    if (ball_history_.size() > 100) {
      ball_history_.pop_back();
    }

    // メトリクス計算エンジンで脅威評価を実行
    crane_msgs::msg::GameAnalysis analysis;
    metrics::MetricContext ctx{
      .world_model = world_model.get(),
      .ball_history = &ball_history_,
      .clock = get_clock(),
      .kick_event_detector = kick_event_detector_.get(),
      .analysis = analysis};

    metric_engine_->computeAll(ctx);
    metric_engine_->visualizeAll(ctx, visualizer);

    game_analysis_pub_->publish(analysis);

    // キック終了時に完成した予実トレースを払い出して publish（bag に残して較正に使う）
    for (const auto & trace : kick_event_detector_->takeCompletedTraces()) {
      kick_prediction_trace_pub_->publish(trace);
    }

    visualizer->flush();
    CraneVisualizerBuffer::publish();
  });
}

}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crane::GameAnalyzerComponent)
