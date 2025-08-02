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
    multicast_receiver_ =
      std::make_unique<multicast::MulticastReceiver>(config_.vision_address, config_.vision_port);
    RCLCPP_INFO(
      node_.get_logger(), "VisionStreamProcessor initialized on %s:%d",
      config_.vision_address.c_str(), config_.vision_port);
  } catch (const std::exception & e) {
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
}

VisionStreamProcessor::~VisionStreamProcessor()
{
  // MulticastReceiverはunique_ptrによりRAIIで自動解放
}

auto VisionStreamProcessor::configure(const ProcessorConfig & config) -> void
{
  // 設定更新 - MulticastReceiverは再初期化せず、設定のみ更新
  config_ = config;

  // 新しい設定でMulticastReceiverを再作成
  try {
    multicast_receiver_ =
      std::make_unique<multicast::MulticastReceiver>(config_.vision_address, config_.vision_port);
    RCLCPP_INFO(
      node_.get_logger(), "VisionStreamProcessor reconfigured on %s:%d",
      config_.vision_address.c_str(), config_.vision_port);
  } catch (const std::exception & e) {
    reportError("Failed to reconfigure vision stream: " + std::string(e.what()));
    RCLCPP_ERROR(node_.get_logger(), "VisionStreamProcessor reconfiguration failed: %s", e.what());
  }
}

auto VisionStreamProcessor::processIncomingData() -> void
{
  if (!multicast_receiver_) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 5000, "MulticastReceiver is not initialized");
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
      }
    }
  } catch (const std::exception & e) {
    // 受信エラーは無視（非ブロッキング受信のため）
  }
}

auto VisionStreamProcessor::processRawPacket(const std::vector<uint8_t> & raw_data) -> bool
{
  try {
    SSL_WrapperPacket packet;
    if (!packet.ParseFromArray(raw_data.data(), static_cast<int>(raw_data.size()))) {
      return false;
    }

    return processVisionPacket(packet);
  } catch (const std::exception & e) {
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
    // Ball validation: check if position is within field bounds
    double ball_x = ssl_ball.x() / 1000.0;
    double ball_y = ssl_ball.y() / 1000.0;
    if (std::abs(ball_x) <= MAX_FIELD_WIDTH && std::abs(ball_y) <= MAX_FIELD_HEIGHT) {
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
        // Robot validation: check position bounds and orientation
        double robot_x = ssl_robot.x() / 1000.0;
        double robot_y = ssl_robot.y() / 1000.0;
        if (
          std::abs(robot_x) <= MAX_FIELD_WIDTH && std::abs(robot_y) <= MAX_FIELD_HEIGHT &&
          std::isfinite(ssl_robot.orientation())) {
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
        // Robot validation: check position bounds and orientation
        double robot_x = ssl_robot.x() / 1000.0;
        double robot_y = ssl_robot.y() / 1000.0;
        if (
          std::abs(robot_x) <= MAX_FIELD_WIDTH && std::abs(robot_y) <= MAX_FIELD_HEIGHT &&
          std::isfinite(ssl_robot.orientation())) {
          int team_index = (our_team_color_ == TeamColor::YELLOW) ? 0 : 1;
          convertRobotDetection(ssl_robot, team_index, robot_id);
        }
      }
    }
  }

  // 検出されなかったロボットの速度を0にリセット（停止検知）
  auto now = node_.get_clock()->now();
  for (int team_idx = 0; team_idx < 2; ++team_idx) {
    for (size_t robot_id = 0; robot_id < MAX_ROBOT_COUNT; ++robot_id) {
      auto & robot = robot_info_[team_idx][robot_id];

      auto & history = robot_history_[team_idx][robot_id];

      if (!robot.vision_detected) {
        // 可視性を更新
        updateVisibility(history, false);

        // 可視性に基づいて検出状態を決定
        robot.vision_detected = isVisibleRobot(history);

        if (history.is_initialized) {
          double dt = (now - history.last_update_time).seconds();

          // 検出されなくなってから100ms以上経過したら速度を0にする
          if (dt > 0.1) {
            robot.velocity.x = 0.0;
            robot.velocity.y = 0.0;
            robot.velocity_norm = 0.0;
            robot.detected = false;
          } else {
            // 短時間ならチャタリング抑制された検出状態を維持
            robot.detected = robot.vision_detected || robot.feedback_detected;
          }
        } else {
          robot.detected = false;
        }
      } else {
        // 検出された場合はdetectedフラグを更新
        robot.detected = true;
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
  double x = ssl_ball.x() / 1000.0;
  double y = ssl_ball.y() / 1000.0;
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
  static std::pair<Vector3, rclcpp::Time> last_ball_data = {
    Vector3::Zero(), node_.get_clock()->now()};
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
  double x = ssl_robot.x() / 1000.0;
  double y = ssl_robot.y() / 1000.0;
  double theta = crane::normalizeAngle(ssl_robot.orientation());

  // 位置・姿勢更新
  robot.pose.x = x;
  robot.pose.y = y;
  robot.pose.theta = theta;

  // チャタリング抑制を含む検出フラグ更新
  auto & history = robot_history_[team_index][robot_id];
  auto now = node_.get_clock()->now();

  // 可視性を更新
  updateVisibility(history, true);

  // 可視性に基づいて検出状態を決定
  robot.vision_detected = isVisibleRobot(history);
  robot.detected = robot.vision_detected || robot.feedback_detected;
  robot.last_tracker_detection_stamp = now;

  // 速度計算（時間微分）

  if (history.is_initialized) {
    double dt = (now - history.last_update_time).seconds();

    if (dt > 0.001) {  // 1ms以上の差分のみ計算
      Eigen::Vector3d current_pos(x, y, theta);
      Eigen::Vector3d position_diff = current_pos - history.last_position;

      // 角度差の正規化（-π to π）
      position_diff(2) = crane::normalizeAngle(position_diff(2));

      Eigen::Vector3d vel = position_diff / dt;

      robot.velocity.x = vel(0);
      robot.velocity.y = vel(1);
      robot.velocity_norm = vel.head<2>().norm();

      history.last_position = current_pos;
      history.last_update_time = now;
    }
  } else {
    // 初回は速度0で初期化
    robot.velocity.x = 0.0;
    robot.velocity.y = 0.0;
    robot.velocity_norm = 0.0;

    history.last_position = Eigen::Vector3d(x, y, theta);
    history.last_update_time = now;
    history.is_initialized = true;
  }
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

auto VisionStreamProcessor::reportError(const std::string & error_message) -> void
{
  RCLCPP_WARN(node_.get_logger(), "VisionStreamProcessor error: %s", error_message.c_str());
}

auto VisionStreamProcessor::updateVisibility(RobotHistoryData & history, bool vision_detected)
  -> void
{
  // シンプルな可視性管理
  const double VISIBILITY_INCREMENT = 0.3;  // 検出時の上昇量
  const double VISIBILITY_DECREMENT = 0.2;  // 非検出時の減少量

  if (vision_detected) {
    // 検出された場合、可視性を上げる
    history.visibility = std::min(1.0, history.visibility + VISIBILITY_INCREMENT);
  } else {
    // 検出されなかった場合、可視性を下げる
    history.visibility = std::max(0.0, history.visibility - VISIBILITY_DECREMENT);
  }
}

auto VisionStreamProcessor::isVisibleRobot(const RobotHistoryData & history) const -> bool
{
  // チャタリング抑制のためのハイステリシス閾値
  const double DETECTION_THRESHOLD = 0.6;  // 検出確定の閾値
  const double LOSS_THRESHOLD = 0.4;       // 検出ロス確定の閾値

  // 可視性に基づくハイステリシス判定
  if (history.visibility > DETECTION_THRESHOLD) {
    return true;  // 高可視性 -> 検出状態
  } else if (history.visibility < LOSS_THRESHOLD) {
    return false;  // 低可視性 -> 非検出状態
  } else {
    // 中間領域では前回の状態を維持（ハイステリシス）
    // この場合は現在の vision_detected 状態をそのまま返す
    return history.visibility >= 0.5;  // 中央値で判定
  }
}
}  // namespace crane
