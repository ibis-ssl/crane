// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__RONAR_EVENT_DETECTOR_HPP_
#define CRANE_GAME_ANALYZER__RONAR_EVENT_DETECTOR_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/ronar_event.hpp>
#include <crane_physics/ball_info.hpp>
#include <deque>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace crane
{
/// RONAR (RObot Non-verbal Action Recognition) イベント検出設定
struct RonarConfig
{
  // ボール所持判定
  double possession_distance_threshold = 0.15;  // m
  double possession_hysteresis = 0.03;          // m

  // シュート検出
  double shot_velocity_threshold = 6.0;     // m/s (FAST_SHOTの閾値)
  double shot_goal_angle_threshold = 30.0;  // degrees

  // ゴール検出
  double goal_detection_delay = 0.25;  // seconds

  // パス検出
  double pass_min_distance = 0.5;  // m
  int pass_confirmation_frames = 5;

  // イベント遅延検証
  double event_delay_verification = 0.25;  // seconds
};

/// ボール所持記録
struct PossessionRecord
{
  RobotIdentifier robot;
  rclcpp::Time timestamp;
  Point position;
};

/// 保留中のイベント（遅延検証用）
struct PendingEvent
{
  crane_msgs::msg::RonarEvent event;
  rclcpp::Time detection_time;
  int confirmation_count;
};

/// RONAR イベント検出器
/// SSL試合からリアルタイムでイベントを検出する
class RonarEventDetector
{
public:
  explicit RonarEventDetector(rclcpp::Clock::SharedPtr clock);

  /// 毎フレーム呼び出し、検出されたイベントを返す
  auto detect(const WorldModelWrapper & world_model) -> std::vector<crane_msgs::msg::RonarEvent>;

  /// 設定を更新
  auto setConfig(const RonarConfig & config) -> void { config_ = config; }

  /// 現在のボール所持者を取得
  auto getCurrentPossession() const -> std::optional<RobotIdentifier>
  {
    return current_possession_;
  }

private:
  // イベント検出メソッド
  auto detectPossessionChange(const WorldModelWrapper & wm)
    -> std::optional<crane_msgs::msg::RonarEvent>;
  auto detectShot(const WorldModelWrapper & wm) -> std::optional<crane_msgs::msg::RonarEvent>;
  auto detectPass(const WorldModelWrapper & wm) -> std::optional<crane_msgs::msg::RonarEvent>;
  auto detectSetPlay(const WorldModelWrapper & wm) -> std::optional<crane_msgs::msg::RonarEvent>;

  // ユーティリティメソッド
  auto findNearestRobotToBall(const WorldModelWrapper & wm) const
    -> std::optional<std::pair<RobotIdentifier, double>>;
  auto isBallMovingTowardsGoal(const WorldModelWrapper & wm, bool their_goal) const -> bool;
  auto createEvent(
    uint8_t type, const Point & position, float ball_speed, float confidence,
    const std::optional<RobotIdentifier> & primary = std::nullopt,
    const std::optional<RobotIdentifier> & secondary = std::nullopt) const
    -> crane_msgs::msg::RonarEvent;

  // 設定
  RonarConfig config_;

  // 時計
  rclcpp::Clock::SharedPtr clock_;

  // ボール所持状態
  std::optional<RobotIdentifier> current_possession_;
  Hysteresis possession_hysteresis_{0.15, 0.03};

  // 所持履歴
  std::deque<PossessionRecord> possession_history_;
  static constexpr size_t MAX_POSSESSION_HISTORY = 100;

  // 保留中イベント（遅延検証用）
  std::deque<PendingEvent> pending_events_;

  // 前回の状態（変化検出用）
  uint8_t last_play_situation_ = 0;
  std::optional<Point> last_ball_pos_;
  std::optional<Point> last_ball_vel_;

  // シュート検出用
  bool shot_in_progress_ = false;
  rclcpp::Time shot_start_time_;
  RobotIdentifier shot_kicker_;
};

}  // namespace crane

#endif  // CRANE_GAME_ANALYZER__RONAR_EVENT_DETECTOR_HPP_
