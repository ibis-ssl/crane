// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/vision_stream_processor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace crane
{
VisionStreamProcessor::VisionStreamProcessor(rclcpp::Node & node)
: node_(node),
  our_team_color_(TeamColor::BLUE),
  has_vision_updated_(false),
  last_t_capture_(0.0),
  last_t_sent_(0.0),
  last_ball_detect_time_(node_.get_clock()->now()),
  last_prediction_time_(node_.get_clock()->now())
{
  // パラメータ設定
  node_.declare_parameter("vision_address", config_.vision_address);
  node_.declare_parameter("vision_port", config_.vision_port);
  node_.declare_parameter("confidence_threshold", config_.confidence_threshold);

  config_.vision_address = node_.get_parameter("vision_address").get_value<std::string>();
  config_.vision_port = node_.get_parameter("vision_port").get_value<int>();
  config_.confidence_threshold = node_.get_parameter("confidence_threshold").get_value<double>();

  // MulticastReceiver初期化 - コンストラクタで直接実行
  try {
    multicast_receiver_ = std::make_unique<multicast::MulticastReceiver>(
      config_.vision_address, config_.vision_port);
    updateStatus(StreamStatus::ACTIVE, "Vision stream initialized successfully");
    RCLCPP_INFO(
      node_.get_logger(), "VisionStreamProcessor initialized on %s:%d", 
      config_.vision_address.c_str(), config_.vision_port);
  } catch (const std::exception & e) {
    updateStatus(StreamStatus::ERROR, "Exception during initialization: " + std::string(e.what()));
    reportError("Failed to initialize vision stream: " + std::string(e.what()));
    RCLCPP_ERROR(node_.get_logger(), "VisionStreamProcessor initialization failed: %s", e.what());
  }

  // ロボット情報初期化
  for (int team = 0; team < 2; ++team) {
    robot_info_[team].resize(MAX_ROBOT_COUNT);
    for (size_t i = 0; i < MAX_ROBOT_COUNT; ++i) {
      auto & robot = robot_info_[team][i];
      robot.id = static_cast<uint8_t>(i);
      robot.vision_detected = false;
      robot.feedback_detected = false;
      robot.internal_tracker_detected = false;
      robot.detected = false;
    }
  }

  // ボール情報初期化
  ball_info_.detected = false;
  ball_info_.state = crane_msgs::msg::BallInfo::STOPPED;

  resetStatistics();
}

VisionStreamProcessor::~VisionStreamProcessor() {
  // MulticastReceiverはunique_ptrによりRAIIで自動解放
}


auto VisionStreamProcessor::configure(const ProcessorConfig & config) -> void
{
  // 設定更新 - MulticastReceiverは再初期化せず、設定のみ更新
  config_ = config;
  
  // 新しい設定でMulticastReceiverを再作成
  try {
    multicast_receiver_ = std::make_unique<multicast::MulticastReceiver>(
      config_.vision_address, config_.vision_port);
    updateStatus(StreamStatus::ACTIVE, "Vision stream reconfigured successfully");
    RCLCPP_INFO(
      node_.get_logger(), "VisionStreamProcessor reconfigured on %s:%d", 
      config_.vision_address.c_str(), config_.vision_port);
  } catch (const std::exception & e) {
    updateStatus(StreamStatus::ERROR, "Exception during reconfiguration: " + std::string(e.what()));
    reportError("Failed to reconfigure vision stream: " + std::string(e.what()));
    RCLCPP_ERROR(node_.get_logger(), "VisionStreamProcessor reconfiguration failed: %s", e.what());
  }
}

auto VisionStreamProcessor::processIncomingData() -> void
{
  if (!multicast_receiver_) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, 
      "MulticastReceiver is not initialized");
    return;
  }


  // マルチキャストパケット受信処理
  std::vector<char> raw_packet_data(65536);  // 最大UDPパケットサイズ
  try {
    size_t received = multicast_receiver_->receive(raw_packet_data);
    if (received > 0) {
      std::vector<uint8_t> packet_data(raw_packet_data.begin(), raw_packet_data.begin() + received);
      if (processRawPacket(packet_data)) {
        has_vision_updated_ = true;
        system_health_.total_packets_processed++;
      }
    }
  } catch (const std::exception & e) {
    // 受信エラーは無視（非ブロッキング受信のため）
  }
  updateSystemHealth();

}

auto VisionStreamProcessor::processRawPacket(const std::vector<uint8_t> & raw_data) -> bool
{
  try {
    SSL_WrapperPacket packet;
    if (!packet.ParseFromArray(raw_data.data(), static_cast<int>(raw_data.size()))) {
      system_health_.conversion_errors++;
      return false;
    }

    return processVisionPacket(packet);
  } catch (const std::exception & e) {
    system_health_.conversion_errors++;
    reportError("Packet parsing error: " + std::string(e.what()));
    return false;
  }
}

auto VisionStreamProcessor::processVisionPacket(const SSL_WrapperPacket & packet) -> bool
{

  bool processed = false;

  if (packet.has_detection()) {
    processed |= processDetectionFrame(packet.detection());
  }

  if (packet.has_geometry()) {
    processed |= processGeometryData(packet.geometry());
  }

  return processed;
}

auto VisionStreamProcessor::processDetectionFrame(const SSL_DetectionFrame & detection) -> bool
{

  // タイムスタンプ更新
  last_t_capture_ = detection.t_capture();
  last_t_sent_ = detection.t_sent();
  system_health_.last_packet_time = node_.get_clock()->now();

  // 全ロボットの検出フラグリセット
  for (auto & team : robot_info_) {
    for (auto & robot : team) {
      robot.vision_detected = false;
      robot.internal_tracker_detected = false;
    }
  }

  // ボール検出処理
  if (!detection.balls().empty()) {
    const auto & ssl_ball = detection.balls().at(0);
    if (validateBallDetection(ssl_ball)) {
      convertBallDetection(ssl_ball);
      last_ball_detect_time_ = node_.get_clock()->now();
    }
  } else {
    // ボール未検出時の処理
    auto now = node_.get_clock()->now();
    if ((now - last_ball_detect_time_).seconds() > 0.1) {
      ball_info_.detected = false;
      ball_info_.velocity.x = 0.0;
      ball_info_.velocity.y = 0.0;
      ball_info_.velocity.z = 0.0;
    }
  }

  // ロボット検出処理
  // 青チーム
  for (const auto & ssl_robot : detection.robots_blue()) {
    if (ssl_robot.has_robot_id()) {
      uint8_t robot_id = static_cast<uint8_t>(ssl_robot.robot_id());
      if (robot_id < MAX_ROBOT_COUNT) {
        if (validateRobotDetection(ssl_robot)) {
          int team_index = (our_team_color_ == TeamColor::BLUE) ? 0 : 1;
          convertRobotDetection(ssl_robot, team_index, robot_id);
        }
      }
    }
  }

  // 黄チーム
  for (const auto & ssl_robot : detection.robots_yellow()) {
    if (ssl_robot.has_robot_id()) {
      uint8_t robot_id = static_cast<uint8_t>(ssl_robot.robot_id());
      if (robot_id < MAX_ROBOT_COUNT) {
        if (validateRobotDetection(ssl_robot)) {
          int team_index = (our_team_color_ == TeamColor::YELLOW) ? 0 : 1;
          convertRobotDetection(ssl_robot, team_index, robot_id);
        }
      }
    }
  }

  return true;
}

auto VisionStreamProcessor::processGeometryData(const SSL_GeometryData & geometry) -> bool
{

  convertFieldGeometry(geometry);

  if (geometry_callback_) {
    geometry_callback_(field_geometry_);
  }


  return true;
}

auto VisionStreamProcessor::convertBallDetection(const SSL_DetectionBall & ssl_ball) -> void
{
  // 座標変換 (mm -> m)
  auto [x, y] = transformPoint(ssl_ball.x(), ssl_ball.y());
  double z = ssl_ball.has_z() ? ssl_ball.z() / 1000.0 : 0.0;

  // 位置更新
  ball_info_.position.x = x;
  ball_info_.position.y = y;
  ball_info_.position.z = z;

  // Vision情報更新
  ball_info_.vision.stamp = node_.get_clock()->now();
  ball_info_.vision.pos.x = x;
  ball_info_.vision.pos.y = y;
  ball_info_.vision.pos.z = z;

  // 速度計算
  static std::pair<Vector3, rclcpp::Time> last_ball_data = {Vector3::Zero(), node_.get_clock()->now()};
  auto now = node_.get_clock()->now();
  double dt = (now - last_ball_data.second).seconds();

  if (dt > 0.001) {  // 1ms以上の差分のみ計算
    Vector3 current_pos(x, y, z);
    Vector3 vel = (current_pos - last_ball_data.first) / dt;
    ball_info_.velocity.x = vel(0);
    ball_info_.velocity.y = vel(1);
    ball_info_.velocity.z = vel(2);
    ball_info_.velocity_norm = vel.norm();
    last_ball_data = {current_pos, now};
  }

  ball_info_.detected = true;
  ball_info_.state = crane_msgs::msg::BallInfo::ROLLING;  // 簡易状態設定
}

auto VisionStreamProcessor::convertRobotDetection(
  const SSL_DetectionRobot & ssl_robot, int team_index, uint8_t robot_id) -> void
{
  if (team_index >= 2 || robot_id >= robot_info_[team_index].size()) {
    return;
  }

  auto & robot = robot_info_[team_index][robot_id];

  // 座標変換
  auto [x, y] = transformPoint(ssl_robot.x(), ssl_robot.y());
  double theta = normalizeAngle(ssl_robot.orientation());

  // 位置・姿勢更新
  robot.pose.x = x;
  robot.pose.y = y;
  robot.pose.theta = theta;

  // 検出フラグ更新
  robot.vision_detected = true;
  robot.detected = true;
  robot.last_tracker_detection_stamp = node_.get_clock()->now();

  // 速度は簡易的に0に設定（実際の実装では時間微分で計算）
  robot.velocity.x = 0.0;
  robot.velocity.y = 0.0;
  robot.velocity_norm = 0.0;
}

auto VisionStreamProcessor::convertFieldGeometry(const SSL_GeometryData & ssl_geometry) -> void
{
  if (!ssl_geometry.has_field()) {
    return;
  }

  const auto & field = ssl_geometry.field();

  field_geometry_.field_width = field.field_length() / 1000.0;  // mm -> m
  field_geometry_.field_height = field.field_width() / 1000.0;
  field_geometry_.goal_width = field.goal_width() / 1000.0;
  field_geometry_.goal_depth = field.goal_depth() / 1000.0;

  // ペナルティエリア寸法（標準SSL値または計算）
  if (field.has_penalty_area_depth()) {
    field_geometry_.penalty_area_height = field.penalty_area_depth() / 1000.0;
  } else {
    field_geometry_.penalty_area_height = field_geometry_.goal_width;
  }

  if (field.has_penalty_area_width()) {
    field_geometry_.penalty_area_width = field.penalty_area_width() / 1000.0;
  } else {
    field_geometry_.penalty_area_width = field_geometry_.goal_width * 2.0;
  }

  field_geometry_.center_circle_radius = 0.5;  // 標準SSL値
  field_geometry_.is_valid = true;
}

auto VisionStreamProcessor::validatePacket(const SSL_WrapperPacket & packet) const -> bool
{
  return packet.has_detection() || packet.has_geometry();
}

auto VisionStreamProcessor::validateDetectionFrame(const SSL_DetectionFrame & detection) const -> bool
{
  return detection.camera_id() <= 32 && detection.t_capture() > 0.0 && detection.t_sent() > 0.0;
}

auto VisionStreamProcessor::validateGeometryData(const SSL_GeometryData & geometry) const -> bool
{
  if (!geometry.has_field()) {
    return false;
  }
  const auto & field = geometry.field();
  return field.field_width() > 0 && field.field_length() > 0;
}

auto VisionStreamProcessor::validateBallDetection(const SSL_DetectionBall & ball) const -> bool
{
  double x = ball.x() / 1000.0;
  double y = ball.y() / 1000.0;
  return std::abs(x) <= MAX_FIELD_WIDTH && std::abs(y) <= MAX_FIELD_HEIGHT;
}

auto VisionStreamProcessor::validateRobotDetection(const SSL_DetectionRobot & robot) const -> bool
{
  double x = robot.x() / 1000.0;
  double y = robot.y() / 1000.0;
  return std::abs(x) <= MAX_FIELD_WIDTH && std::abs(y) <= MAX_FIELD_HEIGHT &&
         std::isfinite(robot.orientation());
}

auto VisionStreamProcessor::calculateBallConfidence(const SSL_DetectionBall & ball) const -> double
{
  double confidence = 1.0;
  double x = ball.x() / 1000.0;
  double y = ball.y() / 1000.0;

  // フィールド端からの距離による信頼度調整
  double edge_distance =
    std::min({MAX_FIELD_WIDTH / 2 - std::abs(x), MAX_FIELD_HEIGHT / 2 - std::abs(y)});

  if (edge_distance < 0.5) {
    confidence *= 0.7;
  }

  return std::max(MIN_CONFIDENCE, confidence);
}

auto VisionStreamProcessor::calculateRobotConfidence(const SSL_DetectionRobot & robot) const
  -> double
{
  return robot.has_confidence() ? std::max(MIN_CONFIDENCE, static_cast<double>(robot.confidence())) : 0.8;
}

auto VisionStreamProcessor::transformPoint(double x_mm, double y_mm) const
  -> std::pair<double, double>
{
  // mm -> m 変換とCrane座標系適用
  return {x_mm / 1000.0, y_mm / 1000.0};
}

auto VisionStreamProcessor::normalizeAngle(double angle) const -> double
{
  return crane::normalizeAngle(angle);
}

auto VisionStreamProcessor::updateSystemHealth() -> void
{
  auto now = node_.get_clock()->now();
  static rclcpp::Time last_update = now;
  double dt = (now - last_update).seconds();

  if (dt > 1.0) {  // 1秒間隔で更新
    // パケットレート計算
    static uint64_t last_packet_count = 0;
    uint64_t current_packets = system_health_.total_packets_processed;
    system_health_.packet_rate_hz = (current_packets - last_packet_count) / dt;
    last_packet_count = current_packets;

    checkPacketRate();
    last_update = now;
  }
}

auto VisionStreamProcessor::updateStatus(StreamStatus new_status, const std::string & message)
  -> void
{
  if (system_health_.status != new_status) {
    system_health_.status = new_status;
    system_health_.status_message = message;
    notifyStatusChange();
  }
}

auto VisionStreamProcessor::checkPacketRate() -> void
{
  // パケットレート監視
  if (system_health_.packet_rate_hz < 30.0) {
    updateStatus(StreamStatus::DEGRADED, "Low packet rate detected");
  } else if (system_health_.packet_rate_hz >= 50.0) {
    updateStatus(StreamStatus::ACTIVE, "Normal operation");
  }
}

auto VisionStreamProcessor::notifyStatusChange() -> void
{
  if (status_callback_) {
    status_callback_(system_health_.status, system_health_.status_message);
  }

  const char * status_str = "UNKNOWN";
  switch (system_health_.status) {
    case StreamStatus::INACTIVE:
      status_str = "INACTIVE";
      break;
    case StreamStatus::ACTIVE:
      status_str = "ACTIVE";
      break;
    case StreamStatus::DEGRADED:
      status_str = "DEGRADED";
      break;
    case StreamStatus::ERROR:
      status_str = "ERROR";
      break;
  }

  RCLCPP_INFO(
    node_.get_logger(), "Vision stream status: %s - %s", status_str,
    system_health_.status_message.c_str());
}



auto VisionStreamProcessor::resetStatistics() -> void
{
  system_health_ = SystemHealth();
  has_vision_updated_ = false;
}

auto VisionStreamProcessor::reportError(const std::string & error_message) -> void
{
  RCLCPP_WARN(node_.get_logger(), "VisionStreamProcessor error: %s", error_message.c_str());
}


// ファクトリ関数
auto createVisionStreamProcessor(rclcpp::Node & node) -> std::unique_ptr<VisionStreamProcessor>
{
  return std::make_unique<VisionStreamProcessor>(node);
}

auto createVisionStreamProcessor(rclcpp::Node & node, const ProcessorConfig & config)
  -> std::unique_ptr<VisionStreamProcessor>
{
  auto processor = std::make_unique<VisionStreamProcessor>(node);
  processor->configure(config);
  return processor;
}

}  // namespace crane
