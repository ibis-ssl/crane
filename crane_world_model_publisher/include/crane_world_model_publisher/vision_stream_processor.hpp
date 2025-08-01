// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__VISION_STREAM_PROCESSOR_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__VISION_STREAM_PROCESSOR_HPP_

#include <robocup_ssl_msgs/ssl_vision_detection.pb.h>
#include <robocup_ssl_msgs/ssl_vision_geometry.pb.h>
#include <robocup_ssl_msgs/ssl_vision_wrapper.pb.h>

#include <crane_comm/multicast.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_msgs/msg/ball_info.hpp>
#include <crane_msgs/msg/robot_feedback.hpp>
#include <crane_msgs/msg/robot_info.hpp>
#include <functional>
#include <memory>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace crane
{
enum class TeamColor { BLUE, YELLOW };

enum class StreamStatus {
  INACTIVE,  // ストリーム停止中
  ACTIVE,    // 正常受信中
  DEGRADED,  // 品質低下
  ERROR      // 接続エラー
};

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

struct SystemHealth
{
  StreamStatus status;
  double packet_rate_hz;
  uint64_t total_packets_processed;
  uint64_t conversion_errors;
  rclcpp::Time last_packet_time;
  std::string status_message;

  SystemHealth()
  : status(StreamStatus::INACTIVE),
    packet_rate_hz(0.0),
    total_packets_processed(0),
    conversion_errors(0),
    last_packet_time(rclcpp::Clock(RCL_ROS_TIME).now())
  {
  }
};

struct ProcessorConfig
{
  std::string vision_address;
  int vision_port;
  double confidence_threshold;

  ProcessorConfig() : vision_address("224.5.23.2"), vision_port(10020), confidence_threshold(0.3) {}
};

class VisionStreamProcessor
{
public:
  using GeometryUpdateCallback = std::function<void(const FieldGeometry &)>;
  using StatusChangeCallback = std::function<void(StreamStatus, const std::string &)>;

  explicit VisionStreamProcessor(rclcpp::Node & node);
  ~VisionStreamProcessor();

  // システム制御
  auto configure(const ProcessorConfig & config) -> void;

  // メインデータ処理
  auto processIncomingData() -> void;

  // データアクセス
  [[nodiscard]] auto getBallInfo() const -> const crane_msgs::msg::BallInfo & { return ball_info_; }
  [[nodiscard]] auto getRobotInfo(int team_index) const
    -> const std::vector<crane_msgs::msg::RobotInfo> &
  {
    return robot_info_[team_index];
  }

  // フィールド情報
  [[nodiscard]] auto getFieldGeometry() const -> const FieldGeometry & { return field_geometry_; }
  [[nodiscard]] auto hasValidGeometry() const -> bool { return field_geometry_.is_valid; }
  [[nodiscard]] auto getFieldWidth() const -> double { return field_geometry_.field_width; }
  [[nodiscard]] auto getFieldHeight() const -> double { return field_geometry_.field_height; }
  [[nodiscard]] auto getGoalWidth() const -> double { return field_geometry_.goal_width; }
  [[nodiscard]] auto getGoalHeight() const -> double { return field_geometry_.goal_height; }
  [[nodiscard]] auto getPenaltyAreaWidth() const -> double
  {
    return field_geometry_.penalty_area_width;
  }
  [[nodiscard]] auto getPenaltyAreaHeight() const -> double
  {
    return field_geometry_.penalty_area_height;
  }

  // システム状態
  [[nodiscard]] auto getSystemHealth() const -> const SystemHealth & { return system_health_; }
  [[nodiscard]] auto getStatus() const -> StreamStatus { return system_health_.status; }
  [[nodiscard]] auto isActive() const -> bool { return multicast_receiver_ != nullptr; }
  [[nodiscard]] auto hasVisionUpdated() const -> bool { return has_vision_updated_; }

  // タイムスタンプ情報
  [[nodiscard]] auto getLastVisionTCapture() const -> double { return last_t_capture_; }
  [[nodiscard]] auto getLastVisionTSent() const -> double { return last_t_sent_; }

  // チーム設定
  auto setOurTeamColor(TeamColor color) -> void { our_team_color_ = color; }
  [[nodiscard]] auto getOurTeamColor() const -> TeamColor { return our_team_color_; }

  // コールバック登録
  auto setGeometryUpdateCallback(GeometryUpdateCallback callback) -> void
  {
    geometry_callback_ = callback;
  }
  auto setStatusChangeCallback(StatusChangeCallback callback) -> void
  {
    status_callback_ = callback;
  }

  // 統計リセット
  auto resetStatistics() -> void;

private:
  rclcpp::Node & node_;

  // 設定
  ProcessorConfig config_;
  TeamColor our_team_color_;

  // ネットワーク通信
  std::unique_ptr<multicast::MulticastReceiver> multicast_receiver_;

  // データ状態
  crane_msgs::msg::BallInfo ball_info_;
  std::vector<crane_msgs::msg::RobotInfo> robot_info_[2];  // [our_team, their_team]
  FieldGeometry field_geometry_;
  bool has_vision_updated_;

  // タイムスタンプ
  double last_t_capture_;
  double last_t_sent_;
  rclcpp::Time last_ball_detect_time_;
  rclcpp::Time last_prediction_time_;

  // システム監視
  SystemHealth system_health_;

  // コールバック
  GeometryUpdateCallback geometry_callback_;
  StatusChangeCallback status_callback_;

  // 内部処理メソッド
  auto processRawPacket(const std::vector<uint8_t> & raw_data) -> bool;
  auto processVisionPacket(const SSL_WrapperPacket & packet) -> bool;
  auto processDetectionFrame(const SSL_DetectionFrame & detection) -> bool;
  auto processGeometryData(const SSL_GeometryData & geometry) -> bool;

  // データ変換
  auto convertBallDetection(const SSL_DetectionBall & ssl_ball) -> void;
  auto convertRobotDetection(const SSL_DetectionRobot & ssl_robot, int team_index, uint8_t robot_id)
    -> void;
  auto convertFieldGeometry(const SSL_GeometryData & ssl_geometry) -> void;

  // データ検証・フィルタリング
  auto validatePacket(const SSL_WrapperPacket & packet) const -> bool;
  auto validateDetectionFrame(const SSL_DetectionFrame & detection) const -> bool;
  auto validateGeometryData(const SSL_GeometryData & geometry) const -> bool;
  auto validateBallDetection(const SSL_DetectionBall & ball) const -> bool;
  auto validateRobotDetection(const SSL_DetectionRobot & robot) const -> bool;

  auto calculateBallConfidence(const SSL_DetectionBall & ball) const -> double;
  auto calculateRobotConfidence(const SSL_DetectionRobot & robot) const -> double;

  // 座標変換
  auto transformPoint(double x_mm, double y_mm) const -> std::pair<double, double>;
  auto normalizeAngle(double angle) const -> double;

  // システム監視
  auto updateSystemHealth() -> void;
  auto updateStatus(StreamStatus new_status, const std::string & message) -> void;
  auto checkPacketRate() -> void;
  auto notifyStatusChange() -> void;

  // エラーハンドリング
  auto reportError(const std::string & error_message) -> void;

  // 定数
  static constexpr double MAX_FIELD_WIDTH = 15.0;   // meters
  static constexpr double MAX_FIELD_HEIGHT = 10.0;  // meters
  static constexpr double MIN_CONFIDENCE = 0.1;
  static constexpr double MAX_ROBOT_SPEED = 10.0;  // m/s
  static constexpr double MAX_BALL_SPEED = 20.0;   // m/s
  static constexpr size_t MAX_ROBOT_COUNT = 20;
};

// ファクトリ関数
auto createVisionStreamProcessor(rclcpp::Node & node) -> std::unique_ptr<VisionStreamProcessor>;
auto createVisionStreamProcessor(rclcpp::Node & node, const ProcessorConfig & config)
  -> std::unique_ptr<VisionStreamProcessor>;

}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__VISION_STREAM_PROCESSOR_HPP_
