// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/visualization_manager.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>

namespace crane
{

// VisualizationBuilderRegistry Implementation
VisualizationBuilderRegistry::VisualizationBuilderRegistry(rclcpp::Node & node) : node_(node)
{
}

auto VisualizationBuilderRegistry::getBuilder(const std::string & topic_name) 
  -> crane::VisualizerMessageBuilder::SharedPtr
{
  auto it = builders_.find(topic_name);
  if (it != builders_.end()) {
    return it->second;
  }
  
  // 新しいビルダーを作成
  auto builder = std::make_shared<crane::VisualizerMessageBuilder>(topic_name);
  builders_[topic_name] = builder;
  
  // 初回作成時にバッファをアクティベート
  ensureBufferActivation();
  
  RCLCPP_DEBUG(node_.get_logger(), "Created visualization builder for topic: %s", topic_name.c_str());
  return builder;
}

auto VisualizationBuilderRegistry::removeBuilder(const std::string & topic_name) -> void
{
  auto it = builders_.find(topic_name);
  if (it != builders_.end()) {
    builders_.erase(it);
    RCLCPP_DEBUG(node_.get_logger(), "Removed visualization builder for topic: %s", topic_name.c_str());
  }
}

auto VisualizationBuilderRegistry::clearAllBuilders() -> void
{
  builders_.clear();
  RCLCPP_DEBUG(node_.get_logger(), "Cleared all visualization builders");
}

auto VisualizationBuilderRegistry::flushAll() -> void
{
  for (auto & [topic, builder] : builders_) {
    builder->flush();
  }
}

auto VisualizationBuilderRegistry::publishAll() -> void
{
  if (!builders_.empty()) {
    crane::CraneVisualizerBuffer::publish();
  }
}

auto VisualizationBuilderRegistry::getBuilderCount() const -> size_t
{
  return builders_.size();
}

auto VisualizationBuilderRegistry::getBuilderNames() const -> std::vector<std::string>
{
  std::vector<std::string> names;
  names.reserve(builders_.size());
  
  for (const auto & [topic, builder] : builders_) {
    names.push_back(topic);
  }
  
  return names;
}

auto VisualizationBuilderRegistry::ensureBufferActivation() -> void
{
  if (!buffer_activated_) {
    crane::CraneVisualizerBuffer::activate(node_);
    buffer_activated_ = true;
    RCLCPP_DEBUG(node_.get_logger(), "Activated CraneVisualizerBuffer");
  }
}

// VisualizationManager Implementation
VisualizationManager::VisualizationManager(rclcpp::Node & node) : node_(node)
{
  builder_registry_ = std::make_unique<VisualizationBuilderRegistry>(node_);
  
  // デフォルト設定
  visualization_enabled_[TopicNames::GEOMETRY] = true;
  visualization_enabled_[TopicNames::VISION] = true;
  visualization_enabled_[TopicNames::TRACKED] = true;
  visualization_enabled_[TopicNames::REFEREE] = true;
  visualization_enabled_[TopicNames::TRAJECTORY] = true;
  visualization_enabled_[TopicNames::SLACK] = false;  // パフォーマンス重視でデフォルトOFF
  visualization_enabled_[TopicNames::PASS_SCORE] = false;
  visualization_enabled_[TopicNames::DEBUG] = false;
  visualization_enabled_[TopicNames::PERFORMANCE] = false;
  
  // 詳細レベル設定（0: 最小, 1: 標準, 2: 詳細）
  visualization_detail_level_[TopicNames::GEOMETRY] = 1;
  visualization_detail_level_[TopicNames::VISION] = 1;
  visualization_detail_level_[TopicNames::TRACKED] = 1;
  visualization_detail_level_[TopicNames::REFEREE] = 1;
  visualization_detail_level_[TopicNames::TRAJECTORY] = 1;
  visualization_detail_level_[TopicNames::SLACK] = 1;
  visualization_detail_level_[TopicNames::PASS_SCORE] = 1;
  visualization_detail_level_[TopicNames::DEBUG] = 0;
  visualization_detail_level_[TopicNames::PERFORMANCE] = 0;
  
  loadVisualizationParameters();
  
  RCLCPP_INFO(node_.get_logger(), "VisualizationManager initialized with %zu builders", 
              builder_registry_->getBuilderCount());
}

auto VisualizationManager::visualizeGeometry(const SSL_GeometryData & geometry_data, bool half_court_mode) -> void
{
  if (!isVisualizationEnabled(TopicNames::GEOMETRY)) {
    return;
  }
  
  drawFieldGeometry(geometry_data, half_court_mode);
  
  // 後方互換性のためのハンドラー呼び出し
  if (geometry_handler_) {
    geometry_handler_(geometry_data, half_court_mode);
  }
}

auto VisualizationManager::visualizeDetection(const SSL_DetectionFrame & detection, bool half_court_mode) -> void
{
  if (!isVisualizationEnabled(TopicNames::VISION)) {
    return;
  }
  
  drawVisionDetections(detection, half_court_mode);
}

auto VisualizationManager::visualizeTrackedData(const WorldModelWrapper::SharedPtr & world_model) -> void
{
  if (!isVisualizationEnabled(TopicNames::TRACKED)) {
    return;
  }
  
  drawTrackedObjects(world_model);
}

auto VisualizationManager::visualizeReferee(
  const robocup_ssl_msgs::msg::Referee & msg, double field_width, double field_height) -> void
{
  if (!isVisualizationEnabled(TopicNames::REFEREE)) {
    return;
  }
  
  drawRefereeInfo(msg, field_width, field_height);
}

auto VisualizationManager::visualizeTrajectories(const WorldModelWrapper::SharedPtr & world_model) -> void
{
  if (!isVisualizationEnabled(TopicNames::TRAJECTORY)) {
    return;
  }
  
  drawRobotTrajectories(world_model);
  drawBallTrajectory(world_model);
}

auto VisualizationManager::visualizeSlackAnalysis(const WorldModelWrapper::SharedPtr & world_model) -> void
{
  if (!isVisualizationEnabled(TopicNames::SLACK)) {
    return;
  }
  
  drawSlackTimes(world_model);
}

auto VisualizationManager::visualizePassScoring(const WorldModelWrapper::SharedPtr & world_model) -> void
{
  if (!isVisualizationEnabled(TopicNames::PASS_SCORE)) {
    return;
  }
  
  // パススコア可視化の実装（将来的に実装）
  auto builder = builder_registry_->getBuilder(TopicNames::PASS_SCORE);
  // 実装予定: パス評価結果の描画
}

auto VisualizationManager::visualizeDebugInfo(const std::string & category, const std::string & info) -> void
{
  if (!isVisualizationEnabled(TopicNames::DEBUG)) {
    return;
  }
  
  auto builder = builder_registry_->getBuilder(TopicNames::DEBUG);
  builder->text().text(category + ": " + info).position(0, 0).fontSize(12).fill("white").build();
}

auto VisualizationManager::visualizePerformanceMetrics(const std::string & component, double processing_time_ms) -> void
{
  if (!isVisualizationEnabled(TopicNames::PERFORMANCE)) {
    return;
  }
  
  auto builder = builder_registry_->getBuilder(TopicNames::PERFORMANCE);
  std::string text = component + ": " + std::to_string(processing_time_ms) + "ms";
  builder->text().text(text).position(0, 0).fontSize(10).fill("yellow").build();
}

auto VisualizationManager::flushAllVisualization() -> void
{
  builder_registry_->flushAll();
}

auto VisualizationManager::publishAllVisualization() -> void
{
  builder_registry_->publishAll();
}

auto VisualizationManager::enableVisualization(const std::string & category, bool enabled) -> void
{
  visualization_enabled_[category] = enabled;
  RCLCPP_DEBUG(node_.get_logger(), "Visualization for %s: %s", 
               category.c_str(), enabled ? "enabled" : "disabled");
}

auto VisualizationManager::setVisualizationDetail(const std::string & category, int detail_level) -> void
{
  visualization_detail_level_[category] = std::max(0, std::min(2, detail_level));
  RCLCPP_DEBUG(node_.get_logger(), "Visualization detail for %s set to: %d", 
               category.c_str(), detail_level);
}

auto VisualizationManager::setGeometryVisualizationHandler(
  std::function<void(const SSL_GeometryData &, bool)> handler) -> void
{
  geometry_handler_ = handler;
}

auto VisualizationManager::getBuilder(const std::string & topic_name) -> crane::VisualizerMessageBuilder::SharedPtr
{
  return builder_registry_->getBuilder(topic_name);
}

// Private methods implementation
auto VisualizationManager::drawFieldGeometry(const SSL_GeometryData & geometry_data, bool half_court_mode) -> void
{
  auto builder = builder_registry_->getBuilder(TopicNames::GEOMETRY);
  
  if (!geometry_data.has_field()) {
    return;
  }
  
  const auto & field = geometry_data.field();
  double field_width = field.field_width() / 1000.0;   // mm to m
  double field_height = field.field_length() / 1000.0; // mm to m
  
  int detail_level = getVisualizationDetailLevel(TopicNames::GEOMETRY);
  
  // フィールドラインの描画
  builder->line().start(-field_height / 2, -field_width / 2).end(field_height / 2, -field_width / 2)
    .stroke("white").strokeWidth(2).build();
  builder->line().start(-field_height / 2, field_width / 2).end(field_height / 2, field_width / 2)
    .stroke("white").strokeWidth(2).build();
  builder->line().start(-field_height / 2, -field_width / 2).end(-field_height / 2, field_width / 2)
    .stroke("white").strokeWidth(2).build();
  builder->line().start(field_height / 2, -field_width / 2).end(field_height / 2, field_width / 2)
    .stroke("white").strokeWidth(2).build();
  
  // センターライン
  builder->line().start(0, -field_width / 2).end(0, field_width / 2).stroke("white").strokeWidth(2).build();
  
  // センターサークル
  builder->circle().center(0, 0).radius(0.5).stroke("white").strokeWidth(2).build();
  
  if (detail_level >= 1) {
    // ゴールエリアとペナルティエリアの描画
    if (field.has_goal_width() && field.has_goal_depth()) {
      double goal_width = field.goal_width() / 1000.0;
      double goal_depth = field.goal_depth() / 1000.0;
      
      // ゴール描画
      builder->line().start(-field_height / 2 - goal_depth, -goal_width / 2)
        .end(-field_height / 2 - goal_depth, goal_width / 2).stroke("yellow").strokeWidth(3).build();
      builder->line().start(field_height / 2 + goal_depth, -goal_width / 2)
        .end(field_height / 2 + goal_depth, goal_width / 2).stroke("yellow").strokeWidth(3).build();
    }
  }
  
  if (detail_level >= 2) {
    // 詳細なフィールドライン（コーナーアーク、ペナルティマーク等）
    for (const auto & arc : field.field_arcs()) {
      double center_x = arc.center().x() / 1000.0;
      double center_y = arc.center().y() / 1000.0;
      double radius = arc.radius() / 1000.0;
      builder->circle().center(center_x, center_y).radius(radius).stroke("white").strokeWidth(1).build();
    }
  }
}

auto VisualizationManager::drawVisionDetections(const SSL_DetectionFrame & detection, bool half_court_mode) -> void
{
  auto builder = builder_registry_->getBuilder(TopicNames::VISION);
  
  // ボール検出の描画
  for (const auto & ball : detection.balls()) {
    double x = ball.x() / 1000.0;  // mm to m
    double y = ball.y() / 1000.0;
    builder->circle().center(x, y).radius(0.043).stroke("orange").strokeWidth(2).build();  // SSL規格ボール半径
  }
  
  // ロボット検出の描画（青チーム）
  for (const auto & robot : detection.robots_blue()) {
    double x = robot.x() / 1000.0;
    double y = robot.y() / 1000.0;
    double theta = robot.orientation();
    
    // ロボット本体
    builder->circle().center(x, y).radius(0.09).stroke("blue").strokeWidth(2).build();  // SSL規格ロボット半径
    
    // 方向指示線
    double direction_end_x = x + 0.12 * std::cos(theta);
    double direction_end_y = y + 0.12 * std::sin(theta);
    builder->line().start(x, y).end(direction_end_x, direction_end_y).stroke("blue").strokeWidth(1).build();
    
    // ロボットID
    if (robot.has_robot_id()) {
      builder->text().text(std::to_string(robot.robot_id())).position(x, y + 0.15).fontSize(12).fill("blue").build();
    }
  }
  
  // ロボット検出の描画（黄チーム）
  for (const auto & robot : detection.robots_yellow()) {
    double x = robot.x() / 1000.0;
    double y = robot.y() / 1000.0;
    double theta = robot.orientation();
    
    // ロボット本体
    builder->circle().center(x, y).radius(0.09).stroke("yellow").strokeWidth(2).build();
    
    // 方向指示線
    double direction_end_x = x + 0.12 * std::cos(theta);
    double direction_end_y = y + 0.12 * std::sin(theta);
    builder->line().start(x, y).end(direction_end_x, direction_end_y).stroke("yellow").strokeWidth(1).build();
    
    // ロボットID
    if (robot.has_robot_id()) {
      builder->text().text(std::to_string(robot.robot_id())).position(x, y + 0.15).fontSize(12).fill("yellow").build();
    }
  }
}

auto VisualizationManager::drawTrackedObjects(const WorldModelWrapper::SharedPtr & world_model) -> void
{
  auto builder = builder_registry_->getBuilder(TopicNames::TRACKED);
  
  // トラッキング済みボール
  const auto ball = world_model->ball();
  builder->circle().center(ball.pos.x(), ball.pos.y()).radius(0.043).stroke("red").strokeWidth(3).build();
  
  // 速度ベクトル
  if (ball.vel.norm() > 0.1) {  // 0.1 m/s 以上で表示
    Point vel_end = ball.pos + ball.vel * 0.5;  // 0.5秒後の位置
    builder->line().start(ball.pos.x(), ball.pos.y()).end(vel_end.x(), vel_end.y()).stroke("red").strokeWidth(2).build();
  }
  
  // トラッキング済みロボット（味方）
  for (const auto & robot : world_model->ours().getAvailableRobots()) {
    const Point & pos = robot->pose.pos;
    
    // ロボット本体（トラッキング済みは太い線）
    builder->circle().center(pos.x(), pos.y()).radius(0.09).stroke("cyan").strokeWidth(3).build();
    
    // 方向と速度
    double direction_end_x = pos.x() + 0.12 * std::cos(robot->pose.theta);
    double direction_end_y = pos.y() + 0.12 * std::sin(robot->pose.theta);
    builder->line().start(pos.x(), pos.y()).end(direction_end_x, direction_end_y).stroke("cyan").strokeWidth(2).build();
    
    // 速度ベクトル
    if (robot->vel.linear.norm() > 0.1) {
      double vel_end_x = pos.x() + robot->vel.linear.x() * 0.3;
      double vel_end_y = pos.y() + robot->vel.linear.y() * 0.3;
      builder->line().start(pos.x(), pos.y()).end(vel_end_x, vel_end_y).stroke("green").strokeWidth(1).build();
    }
  }
  
  // トラッキング済みロボット（敵）
  for (const auto & robot : world_model->theirs().getAvailableRobots()) {
    const Point & pos = robot->pose.pos;
    builder->circle().center(pos.x(), pos.y()).radius(0.09).stroke("magenta").strokeWidth(3).build();
    
    double direction_end_x = pos.x() + 0.12 * std::cos(robot->pose.theta);
    double direction_end_y = pos.y() + 0.12 * std::sin(robot->pose.theta);
    builder->line().start(pos.x(), pos.y()).end(direction_end_x, direction_end_y).stroke("magenta").strokeWidth(2).build();
  }
}

auto VisualizationManager::drawRefereeInfo(
  const robocup_ssl_msgs::msg::Referee & msg, double field_width, double field_height) -> void
{
  auto builder = builder_registry_->getBuilder(TopicNames::REFEREE);
  
  // レフェリー状態の表示
  std::string command_name = "UNKNOWN";
  switch (msg.command) {
    case robocup_ssl_msgs::msg::Referee::COMMAND_HALT:
      command_name = "HALT";
      break;
    case robocup_ssl_msgs::msg::Referee::COMMAND_STOP:
      command_name = "STOP";
      break;
    case robocup_ssl_msgs::msg::Referee::COMMAND_NORMAL_START:
      command_name = "NORMAL_START";
      break;
    case robocup_ssl_msgs::msg::Referee::COMMAND_FORCE_START:
      command_name = "FORCE_START";
      break;
    case robocup_ssl_msgs::msg::Referee::COMMAND_PREPARE_KICKOFF_YELLOW:
      command_name = "PREPARE_KICKOFF_YELLOW";
      break;
    case robocup_ssl_msgs::msg::Referee::COMMAND_PREPARE_KICKOFF_BLUE:
      command_name = "PREPARE_KICKOFF_BLUE";
      break;
    case robocup_ssl_msgs::msg::Referee::COMMAND_PREPARE_PENALTY_YELLOW:
      command_name = "PREPARE_PENALTY_YELLOW";
      break;
    case robocup_ssl_msgs::msg::Referee::COMMAND_PREPARE_PENALTY_BLUE:
      command_name = "PREPARE_PENALTY_BLUE";
      break;
    case robocup_ssl_msgs::msg::Referee::COMMAND_DIRECT_FREE_YELLOW:
      command_name = "DIRECT_FREE_YELLOW";
      break;
    case robocup_ssl_msgs::msg::Referee::COMMAND_DIRECT_FREE_BLUE:
      command_name = "DIRECT_FREE_BLUE";
      break;
    default:
      command_name = "COMMAND_" + std::to_string(msg.command);
      break;
  }
  
  // レフェリー情報をフィールド上部に表示
  builder->text().text("Referee: " + command_name)
    .position(-field_height / 2, field_width / 2 + 0.5).fontSize(16).fill("white").build();
  
  // ボール位置（指定されている場合）
  if (!msg.designated_position.empty()) {
    double ball_x = msg.designated_position.front().x / 1000.0;  // mm to m
    double ball_y = msg.designated_position.front().y / 1000.0;
    builder->circle().center(ball_x, ball_y).radius(0.1).stroke("white").strokeWidth(2).build();
    builder->text().text("Ball").position(ball_x, ball_y + 0.2).fontSize(12).fill("white").build();
  }
}

auto VisualizationManager::drawRobotTrajectories(const WorldModelWrapper::SharedPtr & world_model) -> void
{
  auto builder = builder_registry_->getBuilder(TopicNames::TRAJECTORY);
  
  // 実装予定: ロボット軌跡の描画
  // WorldModelPublisherComponentからの移行対象
}

auto VisualizationManager::drawBallTrajectory(const WorldModelWrapper::SharedPtr & world_model) -> void
{
  auto builder = builder_registry_->getBuilder(TopicNames::TRAJECTORY);
  
  // 実装予定: ボール軌跡の描画
  // WorldModelPublisherComponentからの移行対象
}

auto VisualizationManager::drawSlackTimes(const WorldModelWrapper::SharedPtr & world_model) -> void
{
  auto builder = builder_registry_->getBuilder(TopicNames::SLACK);
  
  // 実装予定: スラック時間分析結果の描画
  // WorldModelPublisherComponentのpostProcessWorldModelからの移行対象
}

auto VisualizationManager::visualizeTrajectoryHistory(const TrajectoryHistoryData & trajectory_data) -> void
{
  if (!isVisualizationEnabled(TopicNames::TRAJECTORY)) {
    return;
  }
  
  auto builder = builder_registry_->getBuilder(TopicNames::TRAJECTORY);
  static constexpr int SAMPLING_NUM = 4;
  
  // 味方ロボットの履歴描画
  for (const auto & [robot_id, history] : trajectory_data.friend_history | ranges::views::enumerate) {
    if (history.size() > SAMPLING_NUM + 1 && history.front().detected) {
      for (int i = 0; i < 10; i++) {
        int start = static_cast<int>((history.size() / 10.) * i);
        int end = static_cast<int>((history.size() / 10.) * (i + 1));

        auto polyline_builder = builder->polyline();
        for (int index = start; index < end; index += SAMPLING_NUM) {
          polyline_builder.addPoint(history.at(index).pose.x, history.at(index).pose.y);
        }
        if (i != 9) {
          polyline_builder.addPoint(history.at(end).pose.x, history.at(end).pose.y);
        }
        polyline_builder
          .stroke(
            trajectory_data.is_yellow ? "yellow" : "blue",
            start / static_cast<double>(history.size()))
          .strokeWidth(15)
          .build();
      }
    }
  }

  // 敵ロボットの履歴描画
  for (const auto & [robot_id, history] : trajectory_data.enemy_history | ranges::views::enumerate) {
    if (history.size() > SAMPLING_NUM + 1 && history.front().detected) {
      for (int i = 0; i < 10; i++) {
        int start = static_cast<int>((history.size() / 10.) * i);
        int end = static_cast<int>((history.size() / 10.) * (i + 1));

        auto polyline_builder = builder->polyline();
        for (int index = start; index < end; index += SAMPLING_NUM) {
          polyline_builder.addPoint(history.at(index).pose.x, history.at(index).pose.y);
        }
        if (i != 9) {
          polyline_builder.addPoint(history.at(end).pose.x, history.at(end).pose.y);
        }
        polyline_builder
          .stroke(
            trajectory_data.is_yellow ? "blue" : "yellow",
            start / static_cast<double>(history.size()))
          .strokeWidth(15)
          .build();
      }
    }
  }

  // ボール軌跡描画
  if (trajectory_data.ball_info_history.size() > SAMPLING_NUM + 1) {
    for (int i = 0; i < 10; i++) {
      int start = static_cast<int>((trajectory_data.ball_info_history.size() / 10.) * i);
      int end = static_cast<int>((trajectory_data.ball_info_history.size() / 10.) * (i + 1));

      auto polyline_builder = builder->polyline();
      for (int index = start; index < end; index += SAMPLING_NUM) {
        polyline_builder.addPoint(
          trajectory_data.ball_info_history.at(index).position.x, 
          trajectory_data.ball_info_history.at(index).position.y);
      }
      if (i != 9) {
        polyline_builder.addPoint(
          trajectory_data.ball_info_history.at(end).position.x, 
          trajectory_data.ball_info_history.at(end).position.y);
      }
      polyline_builder.stroke("orange", start / static_cast<double>(trajectory_data.ball_info_history.size()))
        .strokeWidth(30)
        .build();
    }
  }
}

auto VisualizationManager::isVisualizationEnabled(const std::string & category) const -> bool
{
  auto it = visualization_enabled_.find(category);
  return it != visualization_enabled_.end() ? it->second : false;
}

auto VisualizationManager::getVisualizationDetailLevel(const std::string & category) const -> int
{
  auto it = visualization_detail_level_.find(category);
  return it != visualization_detail_level_.end() ? it->second : 1;
}

auto VisualizationManager::loadVisualizationParameters() -> void
{
  // ROS 2パラメータからの設定読み込み
  node_.declare_parameter("visualization.geometry.enabled", visualization_enabled_[TopicNames::GEOMETRY]);
  node_.declare_parameter("visualization.vision.enabled", visualization_enabled_[TopicNames::VISION]);
  node_.declare_parameter("visualization.tracked.enabled", visualization_enabled_[TopicNames::TRACKED]);
  node_.declare_parameter("visualization.referee.enabled", visualization_enabled_[TopicNames::REFEREE]);
  node_.declare_parameter("visualization.trajectory.enabled", visualization_enabled_[TopicNames::TRAJECTORY]);
  node_.declare_parameter("visualization.slack.enabled", visualization_enabled_[TopicNames::SLACK]);
  node_.declare_parameter("visualization.debug.enabled", visualization_enabled_[TopicNames::DEBUG]);
  
  // パラメータ値の取得
  visualization_enabled_[TopicNames::GEOMETRY] = 
    node_.get_parameter("visualization.geometry.enabled").as_bool();
  visualization_enabled_[TopicNames::VISION] = 
    node_.get_parameter("visualization.vision.enabled").as_bool();
  visualization_enabled_[TopicNames::TRACKED] = 
    node_.get_parameter("visualization.tracked.enabled").as_bool();
  visualization_enabled_[TopicNames::REFEREE] = 
    node_.get_parameter("visualization.referee.enabled").as_bool();
  visualization_enabled_[TopicNames::TRAJECTORY] = 
    node_.get_parameter("visualization.trajectory.enabled").as_bool();
  visualization_enabled_[TopicNames::SLACK] = 
    node_.get_parameter("visualization.slack.enabled").as_bool();
  visualization_enabled_[TopicNames::DEBUG] = 
    node_.get_parameter("visualization.debug.enabled").as_bool();
  
  RCLCPP_INFO(node_.get_logger(), "Loaded visualization parameters from ROS2 parameters");
}

// VisualizationStrategy implementations
auto MinimalVisualizationStrategy::shouldVisualize(const std::string & category) const -> bool
{
  // 最小限の可視化: 基本的なフィールドと追跡データのみ
  return category == "geometry" || category == "tracked";
}

auto MinimalVisualizationStrategy::getDetailLevel(const std::string & category) const -> int
{
  return 0;  // 最小詳細レベル
}

auto StandardVisualizationStrategy::shouldVisualize(const std::string & category) const -> bool
{
  // 標準可視化: デバッグ情報以外すべて
  return category != "debug" && category != "performance" && category != "pass_score";
}

auto StandardVisualizationStrategy::getDetailLevel(const std::string & category) const -> int
{
  return 1;  // 標準詳細レベル
}

auto DetailedVisualizationStrategy::shouldVisualize(const std::string & category) const -> bool
{
  return true;  // すべての可視化を有効化
}

auto DetailedVisualizationStrategy::getDetailLevel(const std::string & category) const -> int
{
  return 2;  // 最大詳細レベル
}

// VisualizationManagerFactory implementation
VisualizationManagerFactory::VisualizationManagerFactory(rclcpp::Node & node) : node_(node)
{
  strategy_ = std::make_unique<StandardVisualizationStrategy>();
}

auto VisualizationManagerFactory::createStandardManager() -> std::unique_ptr<VisualizationManager>
{
  auto manager = std::make_unique<VisualizationManager>(node_);
  
  // 標準設定の適用
  setVisualizationStrategy(std::make_unique<StandardVisualizationStrategy>());
  
  return manager;
}

auto VisualizationManagerFactory::createDebugManager() -> std::unique_ptr<VisualizationManager>
{
  auto manager = std::make_unique<VisualizationManager>(node_);
  
  // デバッグ設定の適用
  setVisualizationStrategy(std::make_unique<DetailedVisualizationStrategy>());
  
  return manager;
}

auto VisualizationManagerFactory::createMinimalManager() -> std::unique_ptr<VisualizationManager>
{
  auto manager = std::make_unique<VisualizationManager>(node_);
  
  // 最小設定の適用
  setVisualizationStrategy(std::make_unique<MinimalVisualizationStrategy>());
  
  return manager;
}

auto VisualizationManagerFactory::setVisualizationStrategy(std::unique_ptr<VisualizationStrategy> strategy) -> void
{
  strategy_ = std::move(strategy);
}

}  // namespace crane