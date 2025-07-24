// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/data_source_manager.hpp"

#include <algorithm>
#include <cmath>

namespace crane
{
DataSourceManager::DataSourceManager(rclcpp::Node & node)
: node_(node),
  logging_enabled_(true),
  angle_diff_threshold_(M_PI / 18.0),  // 10 degrees
  status_log_interval_(5.0),           // 5 seconds
  last_status_log_time_(rclcpp::Clock(RCL_ROS_TIME).now())
{
  status_.vision_available = false;
  status_.tracker_available = false;
  status_.feedback_available = false;
  status_.active_ball_source = DataSourceType::LAST_KNOWN;
  status_.active_robot_source[0] = DataSourceType::LAST_KNOWN;
  status_.active_robot_source[1] = DataSourceType::LAST_KNOWN;
  status_.last_update_time = rclcpp::Clock(RCL_ROS_TIME).now();
}

auto DataSourceManager::integrateSensorData(
  const VisionDataProcessor & vision_processor, const TrackerDataProcessor & tracker_processor,
  const std::vector<crane_msgs::msg::RobotInfo> (&feedback_data)[2],
  const crane_msgs::msg::BallInfo & last_ball_info, const GameConfiguration & game_config)
  -> crane_msgs::msg::WorldModel
{
  crane_msgs::msg::WorldModel msg;

  // Update data source status
  updateDataSourceStatus(vision_processor, tracker_processor);

  // Log status periodically
  if (logging_enabled_) {
    logDataSourceStatus();
  }

  // Basic game configuration
  msg.is_yellow = game_config.is_yellow;
  msg.on_positive_half = game_config.on_positive_half;
  msg.is_emplace_positive_side = game_config.is_emplace_positive_side;
  msg.our_max_allowed_bots = game_config.our_max_allowed_bots;
  msg.their_max_allowed_bots = game_config.their_max_allowed_bots;

  // Integrate ball information
  msg.ball_info = integrateBallData(vision_processor, tracker_processor, last_ball_info);

  // Integrate robot information
  auto [our_robots, their_robots] =
    integrateRobotData(vision_processor, tracker_processor, feedback_data, game_config);
  msg.robot_info_ours = our_robots;
  msg.robot_info_theirs = their_robots;

  // Set field information
  msg.field_info = createFieldInfo(game_config);
  msg.penalty_area_size = createPenaltyAreaInfo(game_config);
  msg.goal_size = createGoalInfo(game_config);

  return msg;
}

auto DataSourceManager::integrateBallData(
  const VisionDataProcessor & vision_processor, const TrackerDataProcessor & tracker_processor,
  const crane_msgs::msg::BallInfo & last_ball_info) -> crane_msgs::msg::BallInfo
{
  auto source_type = selectBallDataSource(vision_processor, tracker_processor);
  status_.active_ball_source = source_type;

  switch (source_type) {
    case DataSourceType::VISION_PRIMARY:
      return vision_processor.getBallInfo();

    case DataSourceType::TRACKER_FALLBACK:
      if (logging_enabled_) {
        RCLCPP_DEBUG(node_.get_logger(), "Using tracker fallback for ball data");
      }
      return tracker_processor.getBallInfo();

    case DataSourceType::LAST_KNOWN:
    default:
      if (logging_enabled_) {
        RCLCPP_WARN(node_.get_logger(), "No fresh ball data available, using last known state");
      }
      return last_ball_info;
  }
}

auto DataSourceManager::integrateRobotData(
  const VisionDataProcessor & vision_processor, const TrackerDataProcessor & tracker_processor,
  const std::vector<crane_msgs::msg::RobotInfo> (&feedback_data)[2],
  const GameConfiguration & game_config)
  -> std::pair<std::vector<crane_msgs::msg::RobotInfo>, std::vector<crane_msgs::msg::RobotInfo>>
{
  std::vector<crane_msgs::msg::RobotInfo> team_0_robots;
  std::vector<crane_msgs::msg::RobotInfo> team_1_robots;

  // Process both teams
  for (int team_idx = 0; team_idx < 2; ++team_idx) {
    auto vision_robots = vision_processor.getRobotInfo(team_idx);
    auto tracker_robots = tracker_processor.getRobotInfo(team_idx);
    const auto & feedback_robots = feedback_data[team_idx];

    std::vector<crane_msgs::msg::RobotInfo> merged_robots;

    // Merge robot information for each robot in the team
    for (size_t i = 0; i < vision_robots.size() && i < feedback_robots.size(); ++i) {
      auto merged_robot = vision_robots[i];

      // Merge feedback information
      merged_robot.feedback_detected = feedback_robots[i].feedback_detected;
      merged_robot.ball_sensor = feedback_robots[i].ball_sensor;
      merged_robot.last_ball_sensor_stamp = feedback_robots[i].last_ball_sensor_stamp;
      merged_robot.last_feedback_detection_stamp = feedback_robots[i].last_feedback_detection_stamp;
      merged_robot.detected = merged_robot.vision_detected || merged_robot.feedback_detected ||
                              merged_robot.internal_tracker_detected;

      // Validate and log angle differences if both vision and tracker are available
      if (
        merged_robot.vision_detected && i < tracker_robots.size() &&
        tracker_robots[i].vision_detected) {
        validateAndLogAngleDifference(merged_robot, tracker_robots[i], team_idx, i);
      }

      // Apply tracker fallback if vision data is not available
      if (!merged_robot.vision_detected && i < tracker_robots.size()) {
        applyTrackerFallback(merged_robot, tracker_robots[i], team_idx, i);
      }

      merged_robots.push_back(merged_robot);
    }

    if (team_idx == 0) {
      team_0_robots = merged_robots;
      status_.active_robot_source[0] = DataSourceType::VISION_PRIMARY;  // Default assumption
    } else {
      team_1_robots = merged_robots;
      status_.active_robot_source[1] = DataSourceType::VISION_PRIMARY;  // Default assumption
    }
  }

  // Classify robots by our team vs their team
  return classifyRobotsByTeam(team_0_robots, team_1_robots, game_config);
}

auto DataSourceManager::selectBallDataSource(
  const VisionDataProcessor & vision_processor, const TrackerDataProcessor & tracker_processor)
  -> DataSourceType
{
  if (vision_processor.hasVisionUpdated()) {
    return DataSourceType::VISION_PRIMARY;
  } else if (tracker_processor.hasTrackerUpdated()) {
    return DataSourceType::TRACKER_FALLBACK;
  } else {
    return DataSourceType::LAST_KNOWN;
  }
}

auto DataSourceManager::mergeRobotInfo(
  const crane_msgs::msg::RobotInfo & vision_robot, const crane_msgs::msg::RobotInfo & tracker_robot,
  const crane_msgs::msg::RobotInfo & feedback_robot) -> crane_msgs::msg::RobotInfo
{
  auto merged = vision_robot;

  // Primary data source is vision (with EKF filtering)
  // Merge feedback information
  merged.feedback_detected = feedback_robot.feedback_detected;
  merged.ball_sensor = feedback_robot.ball_sensor;
  merged.last_ball_sensor_stamp = feedback_robot.last_ball_sensor_stamp;
  merged.last_feedback_detection_stamp = feedback_robot.last_feedback_detection_stamp;

  // Combine detection flags
  merged.detected =
    merged.vision_detected || merged.feedback_detected || merged.internal_tracker_detected;

  return merged;
}

auto DataSourceManager::validateAndLogAngleDifference(
  const crane_msgs::msg::RobotInfo & vision_robot, const crane_msgs::msg::RobotInfo & tracker_robot,
  uint8_t team_index, size_t robot_index) -> void
{
  if (!logging_enabled_) {
    return;
  }

  double vision_angle = vision_robot.pose.theta;
  double tracker_angle = tracker_robot.pose.theta;
  double angle_diff = std::abs(vision_angle - tracker_angle);

  // Normalize angle difference to [-π, π] range
  if (angle_diff > M_PI) {
    angle_diff = 2 * M_PI - angle_diff;
  }

  // Log warning if angle difference is above threshold (default: 10 degrees)
  if (angle_diff > angle_diff_threshold_) {
    RCLCPP_WARN(
      node_.get_logger(),
      "Team%u Robot %zu: Tracker vs Vision angle difference = %.2f deg (Tracker: %.2f, Vision: "
      "%.2f)",
      team_index, robot_index, angle_diff * 180.0 / M_PI, tracker_angle * 180.0 / M_PI,
      vision_angle * 180.0 / M_PI);
  }
}

auto DataSourceManager::applyTrackerFallback(
  crane_msgs::msg::RobotInfo & merged_robot, const crane_msgs::msg::RobotInfo & tracker_robot,
  uint8_t team_index, size_t robot_index) -> bool
{
  if (!tracker_robot.vision_detected) {
    return false;
  }

  if (logging_enabled_) {
    RCLCPP_DEBUG(
      node_.get_logger(), "Team%u Robot %zu: Using tracker fallback (angle: %.2f deg)", team_index,
      robot_index, tracker_robot.pose.theta * 180.0 / M_PI);
  }

  merged_robot.pose = tracker_robot.pose;
  merged_robot.velocity = tracker_robot.velocity;
  merged_robot.velocity_norm = tracker_robot.velocity_norm;
  merged_robot.vision_detected = true;
  merged_robot.detected = true;

  return true;
}

auto DataSourceManager::classifyRobotsByTeam(
  const std::vector<crane_msgs::msg::RobotInfo> & robots_team_0,
  const std::vector<crane_msgs::msg::RobotInfo> & robots_team_1,
  const GameConfiguration & game_config)
  -> std::pair<std::vector<crane_msgs::msg::RobotInfo>, std::vector<crane_msgs::msg::RobotInfo>>
{
  std::vector<crane_msgs::msg::RobotInfo> our_robots;
  std::vector<crane_msgs::msg::RobotInfo> their_robots;

  // Classify team 0 robots
  for (const auto & robot : robots_team_0) {
    if (static_cast<uint8_t>(game_config.is_yellow) == 0) {
      if (
        std::find(game_config.robot_ids_mask.begin(), game_config.robot_ids_mask.end(), robot.id) !=
        game_config.robot_ids_mask.end()) {
        their_robots.push_back(robot);
      } else {
        our_robots.push_back(robot);
      }
    } else {
      their_robots.push_back(robot);
    }
  }

  // Classify team 1 robots
  for (const auto & robot : robots_team_1) {
    if (static_cast<uint8_t>(game_config.is_yellow) == 1) {
      if (
        std::find(game_config.robot_ids_mask.begin(), game_config.robot_ids_mask.end(), robot.id) !=
        game_config.robot_ids_mask.end()) {
        their_robots.push_back(robot);
      } else {
        our_robots.push_back(robot);
      }
    } else {
      their_robots.push_back(robot);
    }
  }

  return {our_robots, their_robots};
}

auto DataSourceManager::createFieldInfo(const GameConfiguration & game_config)
  -> crane_msgs::msg::FieldSize
{
  crane_msgs::msg::FieldSize field_info;
  field_info.x = game_config.field_width;
  field_info.y = game_config.field_height;
  return field_info;
}

auto DataSourceManager::createPenaltyAreaInfo(const GameConfiguration & game_config)
  -> crane_msgs::msg::FieldSize
{
  crane_msgs::msg::FieldSize penalty_area_size;
  penalty_area_size.x = game_config.penalty_area_height;
  penalty_area_size.y = game_config.penalty_area_width;
  return penalty_area_size;
}

auto DataSourceManager::createGoalInfo(const GameConfiguration & game_config)
  -> crane_msgs::msg::FieldSize
{
  crane_msgs::msg::FieldSize goal_size;
  goal_size.x = game_config.goal_height;
  goal_size.y = game_config.goal_width;
  return goal_size;
}

auto DataSourceManager::logDataSourceStatus() -> void
{
  auto now = rclcpp::Clock(RCL_ROS_TIME).now();
  if ((now - last_status_log_time_).seconds() > status_log_interval_) {
    RCLCPP_INFO(
      node_.get_logger(), "Data source status: Vision=%s, Tracker=%s",
      status_.vision_available ? "ACTIVE" : "INACTIVE",
      status_.tracker_available ? "ACTIVE" : "INACTIVE");
    last_status_log_time_ = now;
  }
}

auto DataSourceManager::updateDataSourceStatus(
  const VisionDataProcessor & vision_processor, const TrackerDataProcessor & tracker_processor)
  -> void
{
  status_.vision_available = vision_processor.hasVisionUpdated();
  status_.tracker_available = tracker_processor.hasTrackerUpdated();
  status_.feedback_available = true;  // Feedback is always considered available for simplicity
  status_.last_update_time = rclcpp::Clock(RCL_ROS_TIME).now();
}
}  // namespace crane
