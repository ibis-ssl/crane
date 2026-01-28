// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PHYSICS__ROBOT_INFO_HPP_
#define CRANE_PHYSICS__ROBOT_INFO_HPP_

#include <algorithm>
#include <cmath>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_physics/ball_contact.hpp>
#include <memory>
#include <optional>
#include <rclcpp/time.hpp>

// 前方宣言でcircular dependencyを回避
namespace crane
{
auto getTravelTimeTrapezoidal(
  const Point & current_pos, const Vector2 & current_vel, const Point & target,
  const double max_acceleration, const double max_velocity) -> double;

auto getPredictedPositionTrapezoidal(
  const Point & current_pos, const Vector2 & current_vel, const Point & target_pos,
  const double time, const double max_acceleration, const double max_velocity) -> Point;
}  // namespace crane

namespace crane
{
struct RobotIdentifier
{
  bool is_ours;

  uint8_t id;

  [[nodiscard]] auto operator==(const RobotIdentifier & other) const -> bool
  {
    return is_ours == other.is_ours && id == other.id;
  }

  [[nodiscard]] auto operator!=(const RobotIdentifier & other) const -> bool
  {
    return not(*this == other);
  }
};

struct RobotInfo
{
  uint8_t id;

  [[nodiscard]] auto getID() const -> RobotIdentifier { return {.is_ours = true, .id = id}; }

  Pose2D pose;

  Velocity2D vel;

  // 利用可能性判定の基礎データ（入力ソース別）
  bool available_vision = false;    // ビジョンで検出されているか (robot.detected)
  bool available_hardware = false;  // ハードウェア診断が正常か (!robot_diagnostic_errors_)
  bool available_feedback = false;  // フィードバックにエラーがないか (!robot.has_error)
  bool available_tracker = false;   // トラッカーで検出されているか

  // 利用可能性判定関数（用途に応じて使い分け）
  [[nodiscard]] auto available() const -> bool
  {
    return (available_vision || available_tracker) && available_hardware;
  }

  [[nodiscard]] auto availableStrict() const -> bool
  {
    return available_vision && available_hardware && available_feedback;
  }

  [[nodiscard]] auto availableLoose() const -> bool
  {
    return available_vision || available_tracker;
  }

  rclcpp::Time vision_detection_stamp;

  std::optional<rclcpp::Time> last_tracker_detection_stamp;
  std::optional<rclcpp::Time> last_feedback_detection_stamp;

  std::optional<rclcpp::Time> ball_sensor_stamp;

  bool ball_sensor = false;

  auto getBallSensorAvailable(
    rclcpp::Time now, rclcpp::Duration interval = rclcpp::Duration::from_seconds(0.001)) const
    -> bool
  {
    return ball_sensor_stamp.has_value() &&
           now.get_clock_type() == ball_sensor_stamp->get_clock_type() &&
           (now - *ball_sensor_stamp).seconds() < interval.seconds();
  }

  using SharedPtr = std::shared_ptr<RobotInfo>;

  [[nodiscard]] auto getDribblerDistance() const -> double { return 0.090; }

  [[nodiscard]] auto center_to_kicker() const -> Vector2
  {
    return getNormVec(pose.theta) * getDribblerDistance();
  }

  [[nodiscard]] auto kicker_center() const -> Point { return pose.pos + center_to_kicker(); }

  BallContact ball_contact;

  auto geometry() const { return Circle{.center = pose.pos, .radius = 0.060}; }

  // モーター温度情報（/robot_feedback 由来）
  std::vector<uint8_t> motor_temperatures;

  auto getDistance(const Point & pos) const -> double { return (pos - pose.pos).norm(); }

  auto getDistance(const Pose2D & pose2d) const -> double
  {
    return (this->pose.pos - pose2d.pos).norm();
  }

  auto getSquareDistance(const Point & pos) const -> double
  {
    return (pos - pose.pos).squaredNorm();
  }

  auto getSquareDistance(const Pose2D & pose2d) const -> double
  {
    return (this->pose.pos - pose2d.pos).squaredNorm();
  }

  /**
   * @brief 動的障害物予測：未来位置を計算（Sumatra参考）
   *
   * 反応時間と台形速度プロファイルを考慮した未来位置予測
   *
   * @param target_pos 目標位置（通常はボール位置）
   * @param max_accel 最大加速度 [m/s^2]（デフォルト: 3.0）
   * @param max_vel 最大速度 [m/s]（デフォルト: 2.0）
   * @return 予測される未来位置
   */
  [[nodiscard]] auto predictFuturePosition(
    const Point & target_pos, double max_accel = 3.0, double max_vel = 2.0) const -> Point
  {
    constexpr double MAX_HORIZON = 2.0;                        // 最大予測時間 [s]
    constexpr double TIME_FOR_BOT_TO_REACT = 0.12;             // 反応時間 [s]
    constexpr double MAX_OPPONENT_REACTION_VEL = 1.5;          // 反応速度閾値 [m/s]
    constexpr double TIME_BEFORE_REACTION_USAGE_FACTOR = 0.1;  // 反応時間調整係数

    // 反応時間を計算（速度に応じて調整）
    double current_vel_norm = vel.linear.norm();
    double velocity_factor = std::min(current_vel_norm / MAX_OPPONENT_REACTION_VEL, 1.0);
    double reaction_time =
      TIME_FOR_BOT_TO_REACT * (1.0 - TIME_BEFORE_REACTION_USAGE_FACTOR * velocity_factor);

    // 台形速度プロファイルで目標位置までの移動時間を計算
    double travel_time =
      getTravelTimeTrapezoidal(pose.pos, vel.linear, target_pos, max_accel, max_vel);

    // 実効予測時間 = 反応時間 + 移動時間（最大2.0s）
    double total_time = std::min(reaction_time + travel_time, MAX_HORIZON);

    // 実際の移動時間（反応時間後に移動開始）
    double actual_move_time = total_time - reaction_time;

    // 台形速度プロファイルで予測位置を計算（travel_time.hppの関数を使用）
    return getPredictedPositionTrapezoidal(
      pose.pos, vel.linear, target_pos, actual_move_time, max_accel, max_vel);
  }
};
}  // namespace crane

#endif  // CRANE_PHYSICS__ROBOT_INFO_HPP_
