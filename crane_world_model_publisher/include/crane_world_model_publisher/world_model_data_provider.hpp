// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__WORLD_MODEL_DATA_PROVIDER_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__WORLD_MODEL_DATA_PROVIDER_HPP_

#include <robocup_ssl_msgs/ssl_vision_detection.pb.h>
#include <robocup_ssl_msgs/ssl_vision_geometry.pb.h>
#include <robocup_ssl_msgs/ssl_vision_wrapper.pb.h>

#include <Eigen/Dense>
#include <crane_comm/multicast.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_msgs/msg/ball_info.hpp>
#include <crane_msgs/msg/play_situation.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/robot_feedback.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <crane_msgs/msg/robot_info.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <deque>
#include <functional>
#include <memory>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/detection_frame.hpp>
#include <robocup_ssl_msgs/msg/referee.hpp>
#include <robocup_ssl_msgs/msg/robots_status.hpp>
#include <robocup_ssl_msgs/msg/tracked_frame.hpp>
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

class WorldModelDataProvider
{
public:
  explicit WorldModelDataProvider(rclcpp::Node & node);

  ~WorldModelDataProvider() = default;

  auto on_udp_timer() -> void;

  crane_msgs::msg::WorldModel getMsg();

  [[nodiscard]] auto available() const -> bool
  {
    return has_vision_updated_ || has_tracked_frame_updated_;
  }

  auto setRobotIDsMask(const std::vector<uint8_t> & ids) -> void { robot_ids_mask = ids; }

  auto setAreaMask(const Box & area) -> void { area_mask = area; }

  auto setVisualizationCallbacks(
    std::function<void(const SSL_GeometryData &, bool)> geometry_callback,
    std::function<void(const robocup_ssl_msgs::msg::Referee &, double, double)> referee_callback)
    -> void;

  auto updateGeometryIfNeeded() -> void;

  // Vision処理関連メソッド
  [[nodiscard]] auto getBallInfo() const -> const crane_msgs::msg::BallInfo & { return ball_info_; }
  [[nodiscard]] auto getRobotInfo(int team_index) const
    -> const std::vector<crane_msgs::msg::RobotInfo> &
  {
    return robot_info_[team_index];
  }
  [[nodiscard]] auto field_geometry() const -> const FieldGeometry & { return field_geometry_; }
  [[nodiscard]] auto getLastVisionTCapture() const -> double { return last_t_capture_; }
  [[nodiscard]] auto getLastVisionTSent() const -> double { return last_t_sent_; }
  auto setOurTeamColor(TeamColor color) -> void { our_team_color_ = color; }

private:
  rclcpp::Node & node;

  std::function<void(const SSL_GeometryData &, bool)> geometry_visualization_callback_;
  std::function<void(const robocup_ssl_msgs::msg::Referee &, double, double)>
    referee_visualization_callback_;

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

  // エラー継続時間を算出するためのトラッキング用構造体
  struct ErrorTracker
  {
    bool active = false;
    uint16_t id = 0;
    uint16_t info = 0;
    rclcpp::Time start{};  // 現在のエラー(id, info)が開始した時刻
  };
  std::vector<ErrorTracker> error_tracker_[2];  // [our_team, their_team]

  // 最新のSSL_DetectionFrame（detection_frame生成用）
  mutable SSL_DetectionFrame latest_ssl_detection_frame_;
  mutable bool has_latest_detection_frame_;

  // タイムスタンプ
  double last_t_capture_;
  double last_t_sent_;
  rclcpp::Time last_ball_detect_time_;
  rclcpp::Time last_prediction_time_;

  static constexpr size_t MAX_ROBOT_COUNT = 20;

  // ボールデータ品質管理
  std::pair<Eigen::Vector3d, rclcpp::Time> last_ball_data_;
  std::deque<Eigen::Vector3d> velocity_history_;  // 移動平均用
  bool ball_data_initialized_ = false;

  rclcpp::TimerBase::SharedPtr udp_timer;

  enum class Color { BLUE, YELLOW };

  struct GameData
  {
    std::string team_name;

    Color our_color;

    Color their_color;

    int our_goalie_id;

    int their_goalie_id;

    int our_max_allowed_bots;

    int their_max_allowed_bots;

    double field_w;

    double field_h;

    double goal_w;

    double goal_h;

    double penalty_area_w;

    double penalty_area_h;
  } game_data;

  struct Data
  {
    double ball_placement_target_x;
    double ball_placement_target_y;
  } data;

  bool on_positive_half;

  bool is_emplace_positive_side;

  rclcpp::Time last_ball_detect_time;

  struct BallAnalysis
  {
    bool is_our_ball;

    bool is_their_ball;

    bool ball_event_detected;

    enum class BallEvent { NONE, OUR_BALL, THEIR_BALL };

    BallEvent last_ball_event;
  };

  rclcpp::Subscription<crane_msgs::msg::PlaySituation>::SharedPtr sub_play_situation;

  crane_msgs::msg::PlaySituation latest_play_situation;

  rclcpp::Subscription<crane_msgs::msg::RobotFeedbackArray>::SharedPtr sub_robot_feedback;

  crane_msgs::msg::RobotFeedbackArray robot_feedback;

  rclcpp::Subscription<robocup_ssl_msgs::msg::RobotsStatus>::SharedPtr sub_robots_status_blue;

  rclcpp::Subscription<robocup_ssl_msgs::msg::RobotsStatus>::SharedPtr sub_robots_status_yellow;

  rclcpp::Subscription<robocup_ssl_msgs::msg::Referee>::SharedPtr sub_referee;

  robocup_ssl_msgs::msg::TrackedFrame latest_tracked_frame;
  bool has_tracked_frame_updated_;

  // Tracker UDP receiver
  std::unique_ptr<multicast::MulticastReceiver> tracker_receiver_;

  // Whether to use UDP detection (legacy Vision) for ball/robots
  bool use_udp_detection_ = false;

  // Helper: convert Tracker protobuf to ROS msg
  auto parseTrackedFrameFromWrapper(const TrackerWrapperPacket & wrapper_packet)
    -> robocup_ssl_msgs::msg::TrackedFrame;

  std::vector<uint8_t> robot_ids_mask;

  Box area_mask;

  bool geometry_initialized = false;

  auto processDetectionFrame(const SSL_DetectionFrame & detection) -> bool;
  auto processGeometryData(const SSL_GeometryData & geometry) -> bool;
  auto convertBallDetection(const SSL_DetectionBall & ssl_ball) -> void;
  auto convertFieldGeometry(const SSL_GeometryData & ssl_geometry) -> void;
  auto reportError(const std::string & error_message) -> void;

  auto mergeRobotInfo(
    const crane_msgs::msg::RobotInfo & vision_robot,
    const crane_msgs::msg::RobotInfo & feedback_robot) -> crane_msgs::msg::RobotInfo;

  // TrackedFrame処理関連メソッド
  auto processTrackedFrame(const robocup_ssl_msgs::msg::TrackedFrame & tracked_frame) -> void;
  auto convertTrackedBall(const robocup_ssl_msgs::msg::TrackedBall & tracked_ball)
    -> crane_msgs::msg::BallInfo;
  auto convertTrackedRobot(
    const robocup_ssl_msgs::msg::TrackedRobot & tracked_robot, int team_index)
    -> crane_msgs::msg::RobotInfo;
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__WORLD_MODEL_DATA_PROVIDER_HPP_
