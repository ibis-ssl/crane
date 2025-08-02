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

#include <Eigen/Dense>
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
  // 定数
  static constexpr size_t MAX_ROBOT_COUNT = 20;
  static constexpr double MAX_FIELD_WIDTH = 15.0;   // meters
  static constexpr double MAX_FIELD_HEIGHT = 10.0;  // meters
  static constexpr double MIN_CONFIDENCE = 0.1;
  static constexpr double MAX_ROBOT_SPEED = 10.0;  // m/s
  static constexpr double MAX_BALL_SPEED = 20.0;   // m/s

  using GeometryUpdateCallback = std::function<void(const FieldGeometry &)>;

  explicit VisionStreamProcessor(rclcpp::Node & node);
  ~VisionStreamProcessor();

  auto configure(const ProcessorConfig & config) -> void;

  auto processIncomingData() -> void;

  [[nodiscard]] auto getBallInfo() const -> const crane_msgs::msg::BallInfo & { return ball_info_; }
  [[nodiscard]] auto getRobotInfo(int team_index) const
    -> const std::vector<crane_msgs::msg::RobotInfo> &
  {
    return robot_info_[team_index];
  }

  // フィールド情報
  [[nodiscard]] auto field_geometry() const -> const FieldGeometry & { return field_geometry_; }

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

  // ロボット位置履歴管理（速度計算用）
  struct RobotHistoryData
  {
    Eigen::Vector3d last_position;
    rclcpp::Time last_update_time;
    bool is_initialized;
    double visibility;  // 可視性（0.0-1.0、チャタリング抑制用）

    RobotHistoryData()
    : last_position(Eigen::Vector3d::Zero()), is_initialized(false), visibility(0.0)
    {
    }
  };

  // 各チーム・各ロボットの位置履歴 [team_index][robot_id]
  std::array<std::array<RobotHistoryData, MAX_ROBOT_COUNT>, 2> robot_history_;

  // コールバック
  GeometryUpdateCallback geometry_callback_;

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

  // チャタリング抑制
  auto updateVisibility(RobotHistoryData & history, bool vision_detected) -> void;
  auto isVisibleRobot(const RobotHistoryData & history) const -> bool;

  // エラーハンドリング
  auto reportError(const std::string & error_message) -> void;
};

}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__VISION_STREAM_PROCESSOR_HPP_
