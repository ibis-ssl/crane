// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <cmath>
#include <format>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "crane_game_analyzer/game_analyzer.hpp"

namespace crane
{

namespace
{
// 脅威度 (0.0-1.0) を RGB 色に変換（緑→黄→赤）
std::string threatToColor(double threat_rating)
{
  double t = std::clamp(threat_rating, 0.0, 1.0);
  int r, g;
  if (t < 0.5) {
    r = static_cast<int>(510 * t);           // 0 → 255
    g = 200 + static_cast<int>(55 * t * 2);  // 200 → 255
  } else {
    r = 255;
    g = static_cast<int>(255 * (1.0 - (t - 0.5) * 2));  // 255 → 0
  }
  return std::format("rgb({},{},0)", r, g);
}
}  // namespace

GameAnalyzerComponent::GameAnalyzerComponent(const rclcpp::NodeOptions & options)
: Node("crane_game_analyzer", options),
  visualizer(std::make_shared<VisualizerMessageBuilder>("game_analyzer"))
{
  RCLCPP_INFO(get_logger(), "GameAnalyzer is constructed.");

  // パラメータの設定
  declare_parameter("ball_idle.threshold_duration", 5.0);
  declare_parameter("ball_idle.move_distance_threshold_meter", 0.05);
  declare_parameter("robot_collision.velocity_threshold", 1.0);
  declare_parameter("robot_collision.distance_threshold", 0.2);
  declare_parameter("robot_collision.time_window", 0.5);

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

  // 脅威評価結果のパブリッシャー
  game_analysis_pub_ = create_publisher<crane_msgs::msg::GameAnalysis>("game_analysis", 10);

  world_model->addCallback([&]() {
    auto robot_collision_info = getRobotCollisionInfo();

    if (robot_collision_info) {
      //          robot_collision_info->attack_robot.robot_id
      RCLCPP_DEBUG(
        get_logger(), "Collision Detected : ( %d, %d ) , %f [m/s]",
        robot_collision_info->attack_robot.id, robot_collision_info->attacked_robot.id,
        robot_collision_info->relative_velocity);
    }

    // 脅威評価を実行
    auto analysis = evaluateThreats();
    game_analysis_pub_->publish(analysis);

    visualizer->flush();
    CraneVisualizerBuffer::publish();
  });
}

auto GameAnalyzerComponent::evaluateThreats() -> crane_msgs::msg::GameAnalysis
{
  crane_msgs::msg::GameAnalysis msg;

  // ボール脅威の計算
  auto ball_threat = threat_evaluator_.calculateBallThreat(*world_model);
  msg.ball_threat = threat_evaluator_.toThreatInfoMsg(ball_threat);

  // ロボット脅威の計算（優先度順）
  auto robot_threats = threat_evaluator_.calculateRobotThreats(*world_model, ball_threat);
  for (const auto & threat : robot_threats) {
    msg.robot_threats.push_back(threat_evaluator_.toThreatInfoMsg(threat));
  }

  // 推奨守備者数
  int available_robots = static_cast<int>(world_model->ours().getAvailableRobots().size());
  msg.recommended_num_defenders = static_cast<uint8_t>(
    threat_evaluator_.calculateRecommendedDefenders(ball_threat, robot_threats, available_robots));

  // 脅威の可視化
  // ボール脅威ライン（オレンジ）
  visualizer->line()
    .fromSegment(ball_threat.threat_line)
    .stroke("orange", 0.9)
    .strokeWidth(3)
    .build();

  // ボール脅威の防御ライン（シアン）
  if (ball_threat.protection_line) {
    visualizer->line()
      .fromSegment(*ball_threat.protection_line)
      .stroke("cyan", 0.8)
      .strokeWidth(2)
      .build();
    // 防御ライン端点のマーカー
    visualizer->drawStyledCircle(
      ball_threat.protection_line->first, 0.03, "cyan", 0.5, "cyan", 1.0, 2);
    visualizer->drawStyledCircle(
      ball_threat.protection_line->second, 0.03, "cyan", 0.5, "cyan", 1.0, 2);
  }

  // ロボット脅威（上位5つを可視化）
  int vis_count = 0;
  for (const auto & threat : robot_threats) {
    if (vis_count >= 5) break;

    // 脅威度に応じたグラデーション色
    std::string color = threatToColor(threat.threat_rating);

    // 線の太さも脅威度に応じて変化 (1.0 - 4.0)
    double line_width = 1.0 + threat.threat_rating * 3.0;

    // 不透明度も脅威度に応じて変化 (0.4 - 1.0)
    double opacity = 0.4 + threat.threat_rating * 0.6;

    // 脅威ライン
    visualizer->line()
      .fromSegment(threat.threat_line)
      .stroke(color, opacity)
      .strokeWidth(line_width)
      .build();

    // 脅威スコア表示
    std::string score_text = std::to_string(threat.threat_rating).substr(0, 4);
    visualizer->drawCenteredLabel(threat.robot->pose.pos + Vector2(0, 0.15), score_text, color, 30);

    // 順位表示
    std::string rank_text = "#" + std::to_string(vis_count + 1);
    visualizer->drawCenteredLabel(
      threat.robot->pose.pos + Vector2(-0.12, 0.15), rank_text, color, 20);

    // 防御ライン（存在する場合）
    if (threat.protection_line) {
      visualizer->line()
        .fromSegment(*threat.protection_line)
        .stroke("cyan", 0.6)
        .strokeWidth(1.5)
        .build();
    }

    vis_count++;
  }

  // 上位脅威のリダイレクト角度を可視化（上位2つのみ）
  if (!robot_threats.empty()) {
    for (size_t i = 0; i < std::min(size_t(2), robot_threats.size()); ++i) {
      const auto & threat = robot_threats[i];
      Point ball_pos = world_model->ball().pos;
      Point goal_center = world_model->getOurGoalCenter();
      Point threat_pos = threat.robot->pose.pos;

      // ボール→脅威→ゴール のリダイレクト角度を弧で表示
      Vector2 from_ball = (threat_pos - ball_pos).normalized();
      Vector2 to_goal = (goal_center - threat_pos).normalized();

      double angle1 = std::atan2(-from_ball.y(), -from_ball.x());
      double angle2 = std::atan2(to_goal.y(), to_goal.x());

      // 角度が大きすぎる場合はスキップ
      double angle_diff = std::abs(angle2 - angle1);
      if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
      if (angle_diff < M_PI) {
        std::string arc_color = threatToColor(threat.threat_rating);
        visualizer->arc(
          threat_pos, 0.15, std::min(angle1, angle2), std::max(angle1, angle2), arc_color, 1.5, 8);
      }
    }
  }

  // 推奨守備者数を画面左上に表示
  std::string def_text = "DEF:" + std::to_string(msg.recommended_num_defenders);
  visualizer->text().viewBoxPosition(3, -97).text(def_text).fill("cyan").fontSize(100).build();

  return msg;
}

}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crane::GameAnalyzerComponent)
