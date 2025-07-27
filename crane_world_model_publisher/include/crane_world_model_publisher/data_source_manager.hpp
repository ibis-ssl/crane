// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__DATA_SOURCE_MANAGER_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__DATA_SOURCE_MANAGER_HPP_

#include <crane_msgs/msg/ball_info.hpp>
#include <crane_msgs/msg/robot_info.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <crane_world_model_publisher/tracker_data_processor.hpp>
#include <crane_world_model_publisher/vision_data_processor.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace crane
{
enum class DataSourceType {
  VISION_PRIMARY,    // Vision with EKF filtering
  TRACKER_FALLBACK,  // External tracker fallback
  FEEDBACK_MERGE,    // Robot feedback integration
  LAST_KNOWN         // Previous state fallback
};

struct DataSourceStatus
{
  bool vision_available;
  bool tracker_available;
  bool feedback_available;
  DataSourceType active_ball_source;
  DataSourceType active_robot_source[2];  // Team 0 and Team 1
  rclcpp::Time last_update_time;
};

struct GameConfiguration
{
  bool is_yellow;
  bool on_positive_half;
  bool is_emplace_positive_side;
  uint8_t our_max_allowed_bots;
  uint8_t their_max_allowed_bots;
  std::vector<uint8_t> robot_ids_mask;
  double field_width;
  double field_height;
  double goal_width;
  double goal_height;
  double penalty_area_width;
  double penalty_area_height;
};

class DataSourceManager
{
public:
  explicit DataSourceManager(rclcpp::Node & node);
  ~DataSourceManager() = default;

  // Main integration method
  auto integrateSensorData(
    const VisionDataProcessor & vision_processor, const TrackerDataProcessor & tracker_processor,
    const std::vector<crane_msgs::msg::RobotInfo> (&feedback_data)[2],
    const crane_msgs::msg::BallInfo & last_ball_info, const GameConfiguration & game_config)
    -> crane_msgs::msg::WorldModel;

  // Individual data source integration
  auto integrateBallData(
    const VisionDataProcessor & vision_processor, const TrackerDataProcessor & tracker_processor,
    const crane_msgs::msg::BallInfo & last_ball_info) -> crane_msgs::msg::BallInfo;

  auto integrateRobotData(
    const VisionDataProcessor & vision_processor, const TrackerDataProcessor & tracker_processor,
    const std::vector<crane_msgs::msg::RobotInfo> (&feedback_data)[2],
    const GameConfiguration & game_config)
    -> std::pair<std::vector<crane_msgs::msg::RobotInfo>, std::vector<crane_msgs::msg::RobotInfo>>;

  // Status monitoring
  [[nodiscard]] auto getDataSourceStatus() const -> const DataSourceStatus & { return status_; }

  auto setLoggingEnabled(bool enabled) -> void { logging_enabled_ = enabled; }

  auto setAngleDifferenceThreshold(double threshold_rad) -> void
  {
    angle_diff_threshold_ = threshold_rad;
  }

  auto setStatusLogInterval(double interval_seconds) -> void
  {
    status_log_interval_ = interval_seconds;
  }

private:
  rclcpp::Node & node_;
  DataSourceStatus status_;
  bool logging_enabled_;
  double angle_diff_threshold_;
  double status_log_interval_;
  rclcpp::Time last_status_log_time_;

  // Ball data integration helpers
  auto selectBallDataSource(
    const VisionDataProcessor & vision_processor, const TrackerDataProcessor & tracker_processor)
    -> DataSourceType;

  // Robot data integration helpers
  auto mergeRobotInfo(
    const crane_msgs::msg::RobotInfo & vision_robot,
    const crane_msgs::msg::RobotInfo & tracker_robot,
    const crane_msgs::msg::RobotInfo & feedback_robot) -> crane_msgs::msg::RobotInfo;

  auto validateAndLogAngleDifference(
    const crane_msgs::msg::RobotInfo & vision_robot,
    const crane_msgs::msg::RobotInfo & tracker_robot, uint8_t team_index, size_t robot_index)
    -> void;

  auto applyTrackerFallback(
    crane_msgs::msg::RobotInfo & merged_robot, const crane_msgs::msg::RobotInfo & tracker_robot,
    uint8_t team_index, size_t robot_index) -> bool;

  auto classifyRobotsByTeam(
    const std::vector<crane_msgs::msg::RobotInfo> & robots_team_0,
    const std::vector<crane_msgs::msg::RobotInfo> & robots_team_1,
    const GameConfiguration & game_config)
    -> std::pair<std::vector<crane_msgs::msg::RobotInfo>, std::vector<crane_msgs::msg::RobotInfo>>;

  // Field information helpers
  auto createFieldInfo(const GameConfiguration & game_config) -> crane_msgs::msg::FieldSize;
  auto createPenaltyAreaInfo(const GameConfiguration & game_config) -> crane_msgs::msg::FieldSize;
  auto createGoalInfo(const GameConfiguration & game_config) -> crane_msgs::msg::FieldSize;

  // Status logging
  auto logDataSourceStatus() -> void;
  auto updateDataSourceStatus(
    const VisionDataProcessor & vision_processor, const TrackerDataProcessor & tracker_processor)
    -> void;
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__DATA_SOURCE_MANAGER_HPP_
