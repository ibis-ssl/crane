// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <crane_physics/ball_physics_model.hpp>
#include <crane_physics/kicker_model.hpp>
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

  // パラメータの設定
  declare_parameter("ball_idle.threshold_duration", 5.0);
  declare_parameter("ball_idle.move_distance_threshold_meter", 0.05);
  declare_parameter("robot_collision.velocity_threshold", 1.0);
  declare_parameter("robot_collision.distance_threshold", 0.2);
  declare_parameter("robot_collision.time_window", 0.5);
  declare_parameter("ball_physics_config_path", std::string(""));
  declare_parameter("kicker_physics_config_path", std::string(""));

  // パラメータの読み込み
  config.ball_idle.threshold_duration =
    rclcpp::Duration::from_seconds(get_parameter("ball_idle.threshold_duration").as_double());
  config.ball_idle.move_distance_threshold_meter =
    get_parameter("ball_idle.move_distance_threshold_meter").as_double();
  config.robot_collision.velocity_threshold =
    get_parameter("robot_collision.velocity_threshold").as_double();
  config.robot_collision.distance_threshold =
    get_parameter("robot_collision.distance_threshold").as_double();
  config.robot_collision.time_window = get_parameter("robot_collision.time_window").as_double();
  RCLCPP_DEBUG(
    get_logger(), "  - Velocity threshold: %.2f m/s", config.robot_collision.velocity_threshold);
  RCLCPP_DEBUG(
    get_logger(), "  - Distance threshold: %.2f m", config.robot_collision.distance_threshold);
  RCLCPP_DEBUG(get_logger(), "  - Time window: %.2f s", config.robot_collision.time_window);

  CraneVisualizerBuffer::activate(*this);

  world_model = std::make_unique<WorldModelWrapper>(*this);
  kick_event_detector_ = std::make_unique<KickEventDetector>();

  auto resolveConfigPath = [this](const std::string & config_path) -> std::string {
    if (config_path.empty() || std::filesystem::path(config_path).is_absolute()) {
      return config_path;
    }
    try {
      std::string package_share_dir =
        ament_index_cpp::get_package_share_directory("crane_world_model_publisher");
      return (std::filesystem::path(package_share_dir) / "config" / config_path).string();
    } catch (const std::exception & ex) {
      RCLCPP_WARN(
        get_logger(), "パッケージディレクトリ取得に失敗したため相対パスとして扱います: %s",
        ex.what());
      return config_path;
    }
  };

  // キック予測モデル初期化（ongoing_kickの予測トレース生成に使用）
  std::string ball_physics_config_path;
  get_parameter("ball_physics_config_path", ball_physics_config_path);
  std::shared_ptr<BallPhysicsModel> ball_physics_model;
  if (!ball_physics_config_path.empty()) {
    auto full_config_path = resolveConfigPath(ball_physics_config_path);
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

  std::string kicker_physics_config_path;
  get_parameter("kicker_physics_config_path", kicker_physics_config_path);
  std::shared_ptr<KickerModel> kicker_model;
  if (!kicker_physics_config_path.empty()) {
    auto full_config_path = resolveConfigPath(kicker_physics_config_path);
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
  declare_parameter("pass_target.min_hold_duration_sec", 0.5);
  declare_parameter("pass_target.min_improvement_margin", 0.2);
  double min_hold = 0.5, min_improve = 0.2;
  get_parameter("pass_target.min_hold_duration_sec", min_hold);
  get_parameter("pass_target.min_improvement_margin", min_improve);
  pass_target_metric->setHysteresisParams(min_hold, min_improve);
  metric_engine_->registerMetric(pass_target_metric);

  // パス計画メトリクス（M2: シャドー運用・消費者なし）
  auto pass_plan_metric = std::make_shared<metrics::PassPlanMetric>();
  declare_parameter("pass_plan.recompute_interval_sec", 0.1);
  declare_parameter("pass_plan.min_pass_score", 0.5);
  declare_parameter("pass_plan.max_candidates", 800);
  declare_parameter("pass_plan.dpps_r_resolution", 0.3);
  declare_parameter("pass_plan.dpps_r_max", 2.5);
  declare_parameter("pass_plan.dpps_theta_div", 16);
  declare_parameter("pass_plan.receiver_max_acceleration", 3.0);
  declare_parameter("pass_plan.receiver_max_velocity", 4.0);
  declare_parameter("pass_plan.desired_arrival_speed", 1.5);
  declare_parameter("pass_plan.feasibility_margin", 0.3);
  pass_plan_metric->setRecomputeInterval(
    get_parameter("pass_plan.recompute_interval_sec").as_double());
  pass_plan_metric->setMinPassScore(get_parameter("pass_plan.min_pass_score").as_double());
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
