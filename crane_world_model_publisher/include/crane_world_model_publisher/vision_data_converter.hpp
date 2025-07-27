// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__VISION_DATA_CONVERTER_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__VISION_DATA_CONVERTER_HPP_

#include <robocup_ssl_msgs/ssl_vision_detection.pb.h>
#include <robocup_ssl_msgs/ssl_vision_geometry.pb.h>
#include <robocup_ssl_msgs/ssl_vision_wrapper.pb.h>

#include <crane_msgs/msg/ball_info.hpp>
#include <crane_msgs/msg/robot_info.hpp>
#include <crane_world_model_publisher/vision_packet_receiver.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace crane
{
struct FieldGeometry
{
  double field_width;
  double field_height;
  double goal_width;
  double goal_height;
  double penalty_area_width;
  double penalty_area_height;
  double center_circle_radius;
  double goal_depth;
  bool is_valid;

  FieldGeometry()
  : field_width(0.0),
    field_height(0.0),
    goal_width(0.0),
    goal_height(0.0),
    penalty_area_width(0.0),
    penalty_area_height(0.0),
    center_circle_radius(0.0),
    goal_depth(0.0),
    is_valid(false)
  {
  }
};

struct ConversionMetrics
{
  uint64_t total_packets_processed;
  uint64_t detection_frames_processed;
  uint64_t geometry_frames_processed;
  uint64_t balls_detected;
  uint64_t robots_detected;
  uint64_t conversion_errors;
  rclcpp::Time last_conversion_time;
  double average_conversion_time_ms;

  ConversionMetrics()
  : total_packets_processed(0),
    detection_frames_processed(0),
    geometry_frames_processed(0),
    balls_detected(0),
    robots_detected(0),
    conversion_errors(0),
    last_conversion_time(rclcpp::Clock(RCL_ROS_TIME).now()),
    average_conversion_time_ms(0.0)
  {
  }
};

enum class CoordinateSystem {
  SSL_VISION,     // SSL-Vision coordinate system (origin at field center)
  ROS_STANDARD,   // ROS standard coordinate system
  FIELD_RELATIVE  // Field-relative coordinate system
};

class VisionDataConverter
{
public:
  using BallDetectionCallback = std::function<void(const crane_msgs::msg::BallInfo &)>;
  using RobotDetectionCallback =
    std::function<void(int team_index, const std::vector<crane_msgs::msg::RobotInfo> &)>;
  using GeometryCallback = std::function<void(const FieldGeometry &)>;
  using ErrorCallback = std::function<void(const std::string &)>;

  explicit VisionDataConverter(rclcpp::Node & node);
  ~VisionDataConverter() = default;

  // Packet processing
  auto processVisionPacket(const VisionPacket & packet) -> bool;
  auto processDetectionFrame(const SSL_DetectionFrame & detection) -> bool;
  auto processGeometryData(const SSL_GeometryData & geometry) -> bool;

  // Callback registration
  auto setBallDetectionCallback(BallDetectionCallback callback) -> void
  {
    ball_callback_ = callback;
  }
  auto setRobotDetectionCallback(RobotDetectionCallback callback) -> void
  {
    robot_callback_ = callback;
  }
  auto setGeometryCallback(GeometryCallback callback) -> void { geometry_callback_ = callback; }
  auto setErrorCallback(ErrorCallback callback) -> void { error_callback_ = callback; }

  // Configuration
  auto setCoordinateSystem(CoordinateSystem system) -> void { coordinate_system_ = system; }
  auto enableBallFiltering(bool enable) -> void { filter_balls_ = enable; }
  auto enableRobotFiltering(bool enable) -> void { filter_robots_ = enable; }
  auto setConfidenceThreshold(double threshold) -> void { confidence_threshold_ = threshold; }

  // Field geometry access
  [[nodiscard]] auto getFieldGeometry() const -> const FieldGeometry & { return field_geometry_; }
  [[nodiscard]] auto hasValidGeometry() const -> bool { return field_geometry_.is_valid; }

  // Metrics and statistics
  [[nodiscard]] auto getConversionMetrics() const -> const ConversionMetrics & { return metrics_; }
  auto resetMetrics() -> void;

  // Coordinate transformations
  [[nodiscard]] auto transformPoint(double x, double y, CoordinateSystem from, CoordinateSystem to)
    const -> std::pair<double, double>;
  [[nodiscard]] auto transformAngle(double angle, CoordinateSystem from, CoordinateSystem to) const
    -> double;

  // Validation and filtering
  [[nodiscard]] auto isValidBallPosition(double x, double y) const -> bool;
  [[nodiscard]] auto isValidRobotPosition(double x, double y) const -> bool;
  [[nodiscard]] auto isValidOrientation(double orientation) const -> bool;

  // Quality assessment
  [[nodiscard]] auto assessDetectionQuality(const SSL_DetectionFrame & detection) const -> double;
  [[nodiscard]] auto getConversionEfficiency() const -> double;  // [0.0, 1.0]

private:
  rclcpp::Node & node_;

  // Callbacks
  BallDetectionCallback ball_callback_;
  RobotDetectionCallback robot_callback_;
  GeometryCallback geometry_callback_;
  ErrorCallback error_callback_;

  // Configuration
  CoordinateSystem coordinate_system_;
  bool filter_balls_;
  bool filter_robots_;
  double confidence_threshold_;

  // Field state
  FieldGeometry field_geometry_;
  rclcpp::Time last_geometry_update_;

  // Metrics
  ConversionMetrics metrics_;

  // Conversion methods
  auto convertBallDetection(const SSL_DetectionBall & ssl_ball) const -> crane_msgs::msg::BallInfo;
  auto convertRobotDetection(const SSL_DetectionRobot & ssl_robot, int team_index) const
    -> crane_msgs::msg::RobotInfo;
  auto convertFieldGeometry(const SSL_GeometryData & ssl_geometry) -> FieldGeometry;

  // Filtering methods
  auto filterBallDetections(const SSL_DetectionFrame & detection) -> std::vector<SSL_DetectionBall>;
  auto filterRobotDetections(const SSL_DetectionFrame & detection, int team_index)
    -> std::vector<SSL_DetectionRobot>;

  // Validation methods
  auto validateDetectionFrame(const SSL_DetectionFrame & detection) const -> bool;
  auto validateGeometryData(const SSL_GeometryData & geometry) const -> bool;
  auto validateBallDetection(const SSL_DetectionBall & ball) const -> bool;
  auto validateRobotDetection(const SSL_DetectionRobot & robot) const -> bool;

  // Quality assessment methods
  auto calculateBallConfidence(const SSL_DetectionBall & ball) const -> double;
  auto calculateRobotConfidence(const SSL_DetectionRobot & robot) const -> double;

  // Coordinate transformation helpers
  auto applyFieldTransformation(double & x, double & y) const -> void;
  auto applyRosTransformation(double & x, double & y) const -> void;
  auto normalizeAngle(double angle) const -> double;

  // Utility methods
  auto updateMetrics(const VisionPacket & packet) -> void;
  auto reportError(const std::string & error_message) -> void;
  auto logConversionStatistics() -> void;

  // Constants for validation
  static constexpr double MAX_FIELD_WIDTH = 15.0;   // meters
  static constexpr double MAX_FIELD_HEIGHT = 10.0;  // meters
  static constexpr double MIN_CONFIDENCE = 0.1;
  static constexpr double MAX_ROBOT_SPEED = 10.0;  // m/s
  static constexpr double MAX_BALL_SPEED = 20.0;   // m/s
};

// Factory for creating converter with different configurations
class VisionDataConverterFactory
{
public:
  static auto createStandardConverter(rclcpp::Node & node) -> std::unique_ptr<VisionDataConverter>;
  static auto createHighAccuracyConverter(rclcpp::Node & node)
    -> std::unique_ptr<VisionDataConverter>;
  static auto createFastConverter(rclcpp::Node & node) -> std::unique_ptr<VisionDataConverter>;

  // Configuration presets
  struct ConverterConfig
  {
    CoordinateSystem coordinate_system;
    bool filter_balls;
    bool filter_robots;
    double confidence_threshold;
    bool enable_validation;
  };

  static auto createCustomConverter(rclcpp::Node & node, const ConverterConfig & config)
    -> std::unique_ptr<VisionDataConverter>;

  // Preset configurations
  static auto getStandardConfig() -> ConverterConfig;
  static auto getHighAccuracyConfig() -> ConverterConfig;
  static auto getFastConfig() -> ConverterConfig;
};

// Utility class for batch processing of vision packets
class VisionPacketBatchProcessor
{
public:
  explicit VisionPacketBatchProcessor(std::shared_ptr<VisionDataConverter> converter);
  ~VisionPacketBatchProcessor() = default;

  // Batch processing
  auto processBatch(const std::vector<VisionPacket> & packets) -> void;
  auto setMaxBatchSize(size_t max_size) -> void { max_batch_size_ = max_size; }
  auto setBatchProcessingInterval(double interval_seconds) -> void;

  // Statistics
  [[nodiscard]] auto getBatchProcessingRate() const -> double;
  [[nodiscard]] auto getAverageBatchSize() const -> double;

private:
  std::shared_ptr<VisionDataConverter> converter_;
  size_t max_batch_size_;
  double batch_processing_interval_;
  rclcpp::TimerBase::SharedPtr batch_timer_;

  // Statistics
  uint64_t batches_processed_;
  uint64_t total_packets_in_batches_;
  rclcpp::Time last_batch_time_;

  auto processQueuedPackets() -> void;
  std::vector<VisionPacket> packet_queue_;
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__VISION_DATA_CONVERTER_HPP_
