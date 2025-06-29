// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__VISUALIZATION_MANAGER_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__VISUALIZATION_MANAGER_HPP_

#include <robocup_ssl_msgs/ssl_vision_detection.pb.h>
#include <robocup_ssl_msgs/ssl_vision_geometry.pb.h>
#include <robocup_ssl_msgs/ssl_vision_wrapper_tracked.pb.h>

#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/referee.hpp>
#include <unordered_map>
#include <memory>
#include <string>
#include <functional>

namespace crane
{

/**
 * @brief VisualizationBuilderRegistryは可視化ビルダーの統一管理を行います。
 * 
 * 従来7つのVisualizerMessageBuilderが散在していた問題を解決し、
 * トピック名ベースでの統一管理とリソース最適化を実現します。
 */
class VisualizationBuilderRegistry
{
public:
  explicit VisualizationBuilderRegistry(rclcpp::Node & node);
  ~VisualizationBuilderRegistry() = default;

  // ビルダー管理
  auto getBuilder(const std::string & topic_name) -> crane::VisualizerMessageBuilder::SharedPtr;
  auto removeBuilder(const std::string & topic_name) -> void;
  auto clearAllBuilders() -> void;
  
  // 一括操作
  auto flushAll() -> void;
  auto publishAll() -> void;
  
  // ビルダー統計
  [[nodiscard]] auto getBuilderCount() const -> size_t;
  [[nodiscard]] auto getBuilderNames() const -> std::vector<std::string>;

private:
  rclcpp::Node & node_;
  std::unordered_map<std::string, crane::VisualizerMessageBuilder::SharedPtr> builders_;
  bool buffer_activated_ = false;
  
  auto ensureBufferActivation() -> void;
};

/**
 * @brief VisualizationManagerは可視化システムの統合管理を担当します。
 * 
 * 従来のVisualizationDataHandlerとWorldModelPublisherComponentに
 * 散在していた可視化ロジックを統合し、単一責任原則に従った設計を実現します。
 */
class VisualizationManager
{
public:
  explicit VisualizationManager(rclcpp::Node & node);
  ~VisualizationManager() = default;

  // SSL Vision データ可視化
  auto visualizeGeometry(const SSL_GeometryData & geometry_data, bool half_court_mode = false) -> void;
  auto visualizeDetection(const SSL_DetectionFrame & detection, bool half_court_mode = false) -> void;
  auto visualizeTrackedData(const WorldModelWrapper::SharedPtr & world_model) -> void;
  
  // レフェリー情報可視化
  auto visualizeReferee(const robocup_ssl_msgs::msg::Referee & msg, double field_width, double field_height) -> void;
  
  // 軌跡・分析結果可視化
  auto visualizeTrajectories(const WorldModelWrapper::SharedPtr & world_model) -> void;
  auto visualizeSlackAnalysis(const WorldModelWrapper::SharedPtr & world_model) -> void;
  auto visualizePassScoring(const WorldModelWrapper::SharedPtr & world_model) -> void;
  
  // デバッグ情報可視化
  auto visualizeDebugInfo(const std::string & category, const std::string & info) -> void;
  auto visualizePerformanceMetrics(const std::string & component, double processing_time_ms) -> void;
  
  // 統合制御
  auto flushAllVisualization() -> void;
  auto publishAllVisualization() -> void;
  
  // 設定制御
  auto enableVisualization(const std::string & category, bool enabled) -> void;
  auto setVisualizationDetail(const std::string & category, int detail_level) -> void;
  
  // コールバック登録（後方互換性）
  auto setGeometryVisualizationHandler(std::function<void(const SSL_GeometryData &, bool)> handler) -> void;

private:
  rclcpp::Node & node_;
  std::unique_ptr<VisualizationBuilderRegistry> builder_registry_;
  
  // 可視化設定
  std::unordered_map<std::string, bool> visualization_enabled_;
  std::unordered_map<std::string, int> visualization_detail_level_;
  
  // トピック名定義
  struct TopicNames {
    static constexpr const char* GEOMETRY = "geometry";
    static constexpr const char* VISION = "vision";
    static constexpr const char* TRACKED = "tracked";
    static constexpr const char* REFEREE = "referee";
    static constexpr const char* TRAJECTORY = "trajectory";
    static constexpr const char* SLACK = "slack";
    static constexpr const char* PASS_SCORE = "pass_score";
    static constexpr const char* DEBUG = "debug";
    static constexpr const char* PERFORMANCE = "performance";
  };
  
  // 内部可視化実装
  auto drawFieldGeometry(const SSL_GeometryData & geometry_data, bool half_court_mode) -> void;
  auto drawVisionDetections(const SSL_DetectionFrame & detection, bool half_court_mode) -> void;
  auto drawTrackedObjects(const WorldModelWrapper::SharedPtr & world_model) -> void;
  auto drawRefereeInfo(const robocup_ssl_msgs::msg::Referee & msg, double field_width, double field_height) -> void;
  auto drawRobotTrajectories(const WorldModelWrapper::SharedPtr & world_model) -> void;
  auto drawBallTrajectory(const WorldModelWrapper::SharedPtr & world_model) -> void;
  auto drawSlackTimes(const WorldModelWrapper::SharedPtr & world_model) -> void;
  
  // ユーティリティメソッド
  auto isVisualizationEnabled(const std::string & category) const -> bool;
  auto getVisualizationDetailLevel(const std::string & category) const -> int;
  auto loadVisualizationParameters() -> void;
  
  // 後方互換性サポート
  std::function<void(const SSL_GeometryData &, bool)> geometry_handler_;
};

/**
 * @brief VisualizationStrategyは可視化の戦略パターンを実装します。
 * 
 * 異なる可視化レベル（簡易・標準・詳細）や用途（試合・デバッグ・分析）に
 * 応じた可視化戦略を動的に切り替え可能にします。
 */
class VisualizationStrategy
{
public:
  enum class VisualizationLevel {
    MINIMAL,    // 最小限の可視化（試合中）
    STANDARD,   // 標準可視化（通常開発）
    DETAILED,   // 詳細可視化（デバッグ・分析）
    DEBUG_FULL  // 完全デバッグ表示
  };
  
  virtual ~VisualizationStrategy() = default;
  
  virtual auto shouldVisualize(const std::string & category) const -> bool = 0;
  virtual auto getDetailLevel(const std::string & category) const -> int = 0;
  virtual auto getUpdateFrequency() const -> double = 0;  // Hz
};

class MinimalVisualizationStrategy : public VisualizationStrategy
{
public:
  auto shouldVisualize(const std::string & category) const -> bool override;
  auto getDetailLevel(const std::string & category) const -> int override;
  auto getUpdateFrequency() const -> double override { return 10.0; }  // 10Hz
};

class StandardVisualizationStrategy : public VisualizationStrategy
{
public:
  auto shouldVisualize(const std::string & category) const -> bool override;
  auto getDetailLevel(const std::string & category) const -> int override;
  auto getUpdateFrequency() const -> double override { return 30.0; }  // 30Hz
};

class DetailedVisualizationStrategy : public VisualizationStrategy
{
public:
  auto shouldVisualize(const std::string & category) const -> bool override;
  auto getDetailLevel(const std::string & category) const -> int override;
  auto getUpdateFrequency() const -> double override { return 60.0; }  // 60Hz
};

/**
 * @brief VisualizationManagerFactoryはVisualizationManagerの
 * 生成と設定を管理するファクトリークラスです。
 */
class VisualizationManagerFactory
{
public:
  explicit VisualizationManagerFactory(rclcpp::Node & node);
  ~VisualizationManagerFactory() = default;
  
  auto createStandardManager() -> std::unique_ptr<VisualizationManager>;
  auto createDebugManager() -> std::unique_ptr<VisualizationManager>;
  auto createMinimalManager() -> std::unique_ptr<VisualizationManager>;
  
  auto setVisualizationStrategy(std::unique_ptr<VisualizationStrategy> strategy) -> void;
  
private:
  rclcpp::Node & node_;
  std::unique_ptr<VisualizationStrategy> strategy_;
};

}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__VISUALIZATION_MANAGER_HPP_