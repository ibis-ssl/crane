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

#include <array>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/ball_info.hpp>
#include <crane_msgs/msg/robot_info.hpp>
#include <deque>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/referee.hpp>
#include <string>
#include <unordered_map>

namespace crane
{
/**
 * @brief VisualizationManagerは可視化システムの統合管理を担当します。
 */
class VisualizationManager
{
public:
  explicit VisualizationManager(rclcpp::Node & node);
  ~VisualizationManager() = default;

  // 可視化（直接実装）
  auto drawFieldGeometry(const SSL_GeometryData & geometry_data, bool half_court_mode = false)
    -> void;
  auto drawVisionDetections(const SSL_DetectionFrame & detection, bool half_court_mode = false)
    -> void;
  auto drawTrackedObjects(const WorldModelWrapper::SharedPtr & world_model) -> void;
  auto drawRefereeInfo(
    const robocup_ssl_msgs::msg::Referee & msg, double field_width, double field_height,
    const std::string & command_text) -> void;
  auto drawSlackTimes(const WorldModelWrapper::SharedPtr & world_model) -> void;
  auto visualizePassScoring(const WorldModelWrapper::SharedPtr & world_model) -> void;

  // 軌跡履歴データ構造
  struct TrajectoryHistoryData
  {
    std::array<std::deque<crane_msgs::msg::RobotInfo>, 20> friend_history;
    std::array<std::deque<crane_msgs::msg::RobotInfo>, 20> enemy_history;
    std::deque<crane_msgs::msg::BallInfo> ball_info_history;
    bool is_yellow;
  };

  // 軌跡履歴可視化
  auto visualizeTrajectoryHistory(const TrajectoryHistoryData & trajectory_data) -> void;

  // デバッグ情報可視化
  auto visualizeDebugInfo(const std::string & category, const std::string & info) -> void;
  auto visualizePerformanceMetrics(const std::string & component, double processing_time_ms)
    -> void;

  // 各用途別の専用Builder
  crane::VisualizerMessageBuilder::SharedPtr geometry_builder;
  crane::VisualizerMessageBuilder::SharedPtr vision_builder;
  crane::VisualizerMessageBuilder::SharedPtr tracked_builder;
  crane::VisualizerMessageBuilder::SharedPtr referee_builder;
  crane::VisualizerMessageBuilder::SharedPtr trajectory_builder;
  crane::VisualizerMessageBuilder::SharedPtr slack_builder;
  crane::VisualizerMessageBuilder::SharedPtr pass_score_builder;
  crane::VisualizerMessageBuilder::SharedPtr debug_builder;
  crane::VisualizerMessageBuilder::SharedPtr performance_builder;
  crane::VisualizerMessageBuilder::SharedPtr kick_event_builder;

private:
  rclcpp::Node & node_;
};

}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__VISUALIZATION_MANAGER_HPP_
