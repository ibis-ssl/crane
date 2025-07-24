// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/vision_data_converter.hpp"

#include <algorithm>
#include <cmath>

namespace crane
{
VisionDataConverter::VisionDataConverter(rclcpp::Node & node)
: node_(node),
  coordinate_system_(CoordinateSystem::SSL_VISION),
  filter_balls_(true),
  filter_robots_(true),
  confidence_threshold_(0.3),
  last_geometry_update_(rclcpp::Clock(RCL_ROS_TIME).now())
{
  resetMetrics();
}

auto VisionDataConverter::processVisionPacket(const VisionPacket & packet) -> bool
{
  auto start_time = std::chrono::high_resolution_clock::now();

  try {
    bool processed = false;

    if (packet.packet.has_detection()) {
      processed |= processDetectionFrame(packet.packet.detection());
    }

    if (packet.packet.has_geometry()) {
      processed |= processGeometryData(packet.packet.geometry());
    }

    updateMetrics(packet);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    metrics_.average_conversion_time_ms = duration.count() / 1000.0;

    return processed;
  } catch (const std::exception & e) {
    metrics_.conversion_errors++;
    reportError("Packet conversion error: " + std::string(e.what()));
    return false;
  }
}

auto VisionDataConverter::processDetectionFrame(const SSL_DetectionFrame & detection) -> bool
{
  if (!validateDetectionFrame(detection)) {
    metrics_.conversion_errors++;
    return false;
  }

  metrics_.detection_frames_processed++;

  // Process ball detections
  auto filtered_balls = filterBallDetections(detection);
  for (const auto & ssl_ball : filtered_balls) {
    auto ball_info = convertBallDetection(ssl_ball);
    if (ball_callback_) {
      ball_callback_(ball_info);
    }
    metrics_.balls_detected++;
  }

  // Process robot detections for both teams
  for (int team = 0; team < 2; ++team) {
    auto filtered_robots = filterRobotDetections(detection, team);
    std::vector<crane_msgs::msg::RobotInfo> robot_infos;

    for (const auto & ssl_robot : filtered_robots) {
      auto robot_info = convertRobotDetection(ssl_robot, team);
      robot_infos.push_back(robot_info);
      metrics_.robots_detected++;
    }

    if (robot_callback_ && !robot_infos.empty()) {
      robot_callback_(team, robot_infos);
    }
  }

  return true;
}

auto VisionDataConverter::processGeometryData(const SSL_GeometryData & geometry) -> bool
{
  if (!validateGeometryData(geometry)) {
    metrics_.conversion_errors++;
    return false;
  }

  field_geometry_ = convertFieldGeometry(geometry);
  last_geometry_update_ = rclcpp::Clock(RCL_ROS_TIME).now();
  metrics_.geometry_frames_processed++;

  if (geometry_callback_) {
    geometry_callback_(field_geometry_);
  }

  return true;
}

auto VisionDataConverter::convertBallDetection(const SSL_DetectionBall & ssl_ball) const
  -> crane_msgs::msg::BallInfo
{
  crane_msgs::msg::BallInfo ball_info;

  // Position conversion
  double x = ssl_ball.x() / 1000.0;  // mm to m
  double y = ssl_ball.y() / 1000.0;
  double z = ssl_ball.z() / 1000.0;

  auto [converted_x, converted_y] =
    transformPoint(x, y, CoordinateSystem::SSL_VISION, coordinate_system_);

  ball_info.position.x = converted_x;
  ball_info.position.y = converted_y;
  ball_info.position.z = z;

  // Velocity (if available)
  ball_info.velocity.x = 0.0;
  ball_info.velocity.y = 0.0;
  ball_info.velocity.z = 0.0;

  // Additional ball info
  ball_info.detected = true;
  ball_info.state = crane_msgs::msg::BallInfo::ROLLING;  // Default state
  ball_info.velocity_norm = 0.0;

  return ball_info;
}

auto VisionDataConverter::convertRobotDetection(
  const SSL_DetectionRobot & ssl_robot, int team_index) const -> crane_msgs::msg::RobotInfo
{
  crane_msgs::msg::RobotInfo robot_info;

  // Robot ID
  robot_info.id = ssl_robot.robot_id();

  // Position conversion
  double x = ssl_robot.x() / 1000.0;  // mm to m
  double y = ssl_robot.y() / 1000.0;

  auto [converted_x, converted_y] =
    transformPoint(x, y, CoordinateSystem::SSL_VISION, coordinate_system_);

  robot_info.pose.x = converted_x;
  robot_info.pose.y = converted_y;
  robot_info.pose.theta =
    transformAngle(ssl_robot.orientation(), CoordinateSystem::SSL_VISION, coordinate_system_);

  // Velocity (simplified - would need temporal tracking for accurate velocity)
  robot_info.velocity.x = 0.0;
  robot_info.velocity.y = 0.0;
  robot_info.velocity_norm = 0.0;

  // Detection flags
  robot_info.vision_detected = true;
  robot_info.feedback_detected = false;
  robot_info.internal_tracker_detected = false;
  robot_info.detected = true;

  // Timestamps
  robot_info.last_tracker_detection_stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  return robot_info;
}

auto VisionDataConverter::convertFieldGeometry(const SSL_GeometryData & ssl_geometry)
  -> FieldGeometry
{
  FieldGeometry geometry;

  if (ssl_geometry.has_field()) {
    const auto & field = ssl_geometry.field();

    // Field dimensions (convert from mm to m)
    geometry.field_width = field.field_width() / 1000.0;
    geometry.field_height = field.field_length() / 1000.0;
    geometry.goal_width = field.goal_width() / 1000.0;
    geometry.goal_depth = field.goal_depth() / 1000.0;

    // Calculate penalty area dimensions from field lines
    // This is a simplified calculation - real implementation would parse field lines
    geometry.penalty_area_width = 2.0;    // Standard SSL value
    geometry.penalty_area_height = 1.0;   // Standard SSL value
    geometry.center_circle_radius = 0.5;  // Standard SSL value

    geometry.is_valid = true;
  }

  return geometry;
}

auto VisionDataConverter::filterBallDetections(const SSL_DetectionFrame & detection)
  -> std::vector<SSL_DetectionBall>
{
  std::vector<SSL_DetectionBall> filtered_balls;

  for (const auto & ball : detection.balls()) {
    if (validateBallDetection(ball)) {
      if (!filter_balls_ || calculateBallConfidence(ball) >= confidence_threshold_) {
        filtered_balls.push_back(ball);
      }
    }
  }

  return filtered_balls;
}

auto VisionDataConverter::filterRobotDetections(
  const SSL_DetectionFrame & detection, int team_index) -> std::vector<SSL_DetectionRobot>
{
  std::vector<SSL_DetectionRobot> filtered_robots;

  const auto & robots = (team_index == 0) ? detection.robots_blue() : detection.robots_yellow();

  for (const auto & robot : robots) {
    if (validateRobotDetection(robot)) {
      if (!filter_robots_ || calculateRobotConfidence(robot) >= confidence_threshold_) {
        filtered_robots.push_back(robot);
      }
    }
  }

  return filtered_robots;
}

auto VisionDataConverter::validateDetectionFrame(const SSL_DetectionFrame & detection) const -> bool
{
  // Basic validation
  if (detection.camera_id() > 32) {
    return false;
  }

  if (detection.t_capture() <= 0.0 || detection.t_sent() <= 0.0) {
    return false;
  }

  return true;
}

auto VisionDataConverter::validateGeometryData(const SSL_GeometryData & geometry) const -> bool
{
  // Basic validation for geometry data
  if (!geometry.has_field()) {
    return false;
  }

  const auto & field = geometry.field();
  if (field.field_width() <= 0 || field.field_length() <= 0) {
    return false;
  }

  return true;
}

auto VisionDataConverter::validateBallDetection(const SSL_DetectionBall & ball) const -> bool
{
  double x = ball.x() / 1000.0;  // mm to m
  double y = ball.y() / 1000.0;

  return isValidBallPosition(x, y);
}

auto VisionDataConverter::validateRobotDetection(const SSL_DetectionRobot & robot) const -> bool
{
  double x = robot.x() / 1000.0;  // mm to m
  double y = robot.y() / 1000.0;

  return isValidRobotPosition(x, y) && isValidOrientation(robot.orientation());
}

auto VisionDataConverter::isValidBallPosition(double x, double y) const -> bool
{
  return std::abs(x) <= MAX_FIELD_WIDTH && std::abs(y) <= MAX_FIELD_HEIGHT;
}

auto VisionDataConverter::isValidRobotPosition(double x, double y) const -> bool
{
  return std::abs(x) <= MAX_FIELD_WIDTH && std::abs(y) <= MAX_FIELD_HEIGHT;
}

auto VisionDataConverter::isValidOrientation(double orientation) const -> bool
{
  return std::isfinite(orientation) && std::abs(orientation) <= 2 * M_PI;
}

auto VisionDataConverter::calculateBallConfidence(const SSL_DetectionBall & ball) const -> double
{
  // Simplified confidence calculation based on position validity
  double confidence = 1.0;

  double x = ball.x() / 1000.0;
  double y = ball.y() / 1000.0;

  // Reduce confidence for balls near field edges
  double edge_distance =
    std::min({MAX_FIELD_WIDTH / 2 - std::abs(x), MAX_FIELD_HEIGHT / 2 - std::abs(y)});

  if (edge_distance < 0.5) {
    confidence *= 0.7;
  }

  return std::max(MIN_CONFIDENCE, confidence);
}

auto VisionDataConverter::calculateRobotConfidence(const SSL_DetectionRobot & robot) const -> double
{
  // Simplified confidence calculation
  double confidence = [&]() {
    if (!robot.has_confidence()) {
      return 0.8;  // Default confidence when not provided
    } else {
      return robot.confidence();
    }
  }();

  return std::max(MIN_CONFIDENCE, confidence);
}

auto VisionDataConverter::transformPoint(
  double x, double y, CoordinateSystem from, CoordinateSystem to) const -> std::pair<double, double>
{
  if (from == to) {
    return {x, y};
  }

  // For now, implement basic transformations
  // Real implementation would handle complex coordinate transformations
  switch (to) {
    case CoordinateSystem::SSL_VISION:
      return {x, y};
    case CoordinateSystem::ROS_STANDARD:
      return {x, y};  // Simplified
    case CoordinateSystem::FIELD_RELATIVE:
      return {x, y};  // Simplified
    default:
      return {x, y};
  }
}

auto VisionDataConverter::transformAngle(
  double angle, CoordinateSystem from, CoordinateSystem to) const -> double
{
  if (from == to) {
    return angle;
  }

  // Simplified angle transformation
  return normalizeAngle(angle);
}

auto VisionDataConverter::normalizeAngle(double angle) const -> double
{
  while (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}

auto VisionDataConverter::updateMetrics(const VisionPacket & packet) -> void
{
  metrics_.total_packets_processed++;
  metrics_.last_conversion_time = rclcpp::Clock(RCL_ROS_TIME).now();
}

auto VisionDataConverter::reportError(const std::string & error_message) -> void
{
  if (error_callback_) {
    error_callback_(error_message);
  }
  RCLCPP_WARN(node_.get_logger(), "Vision conversion error: %s", error_message.c_str());
}

auto VisionDataConverter::resetMetrics() -> void { metrics_ = ConversionMetrics(); }

// Factory implementations
auto VisionDataConverterFactory::createStandardConverter(rclcpp::Node & node)
  -> std::unique_ptr<VisionDataConverter>
{
  auto converter = std::make_unique<VisionDataConverter>(node);
  converter->setCoordinateSystem(CoordinateSystem::SSL_VISION);
  converter->enableBallFiltering(true);
  converter->enableRobotFiltering(true);
  converter->setConfidenceThreshold(0.3);
  return converter;
}

auto VisionDataConverterFactory::getStandardConfig() -> ConverterConfig
{
  return {CoordinateSystem::SSL_VISION, true, true, 0.3, true};
}

// Batch processor implementation (simplified)
VisionPacketBatchProcessor::VisionPacketBatchProcessor(
  std::shared_ptr<VisionDataConverter> converter)
: converter_(converter),
  max_batch_size_(10),
  batch_processing_interval_(0.016),  // ~60 Hz
  batches_processed_(0),
  total_packets_in_batches_(0),
  last_batch_time_(rclcpp::Clock(RCL_ROS_TIME).now())
{
}

auto VisionPacketBatchProcessor::processBatch(const std::vector<VisionPacket> & packets) -> void
{
  for (const auto & packet : packets) {
    converter_->processVisionPacket(packet);
  }

  batches_processed_++;
  total_packets_in_batches_ += packets.size();
  last_batch_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
}
}  // namespace crane
