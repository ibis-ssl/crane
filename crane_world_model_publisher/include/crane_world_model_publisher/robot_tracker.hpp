// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__ROBOT_TRACKER_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__ROBOT_TRACKER_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace crane
{
// consai_vision_tracker互換の型定義
using Vector2 = Eigen::Vector2d;
using Vector3 = Eigen::Vector3d;

enum class RobotTrackerType {
  FRIENDLY,  // 味方ロボット（Vision + オドメトリ + コマンド統合）
  ENEMY      // 敵ロボット（Visionのみ）
};

// EKF状態ベクトルのインデックス定義（6次元: [x, y, theta, vx, vy, omega]）
enum class StateIndex : int {
  X = 0,      // X座標 (m)
  Y = 1,      // Y座標 (m)
  THETA = 2,  // 姿勢角 (rad)
  VX = 3,     // X方向速度 (m/s)
  VY = 4,     // Y方向速度 (m/s)
  OMEGA = 5   // 角速度 (rad/s)
};

// 状態インデックス用ヘルパー関数（簡潔なアクセスのため）
constexpr int idx(StateIndex index) { return static_cast<int>(index); }

class RobotTracker
{
public:
  explicit RobotTracker(
    uint8_t robot_id, RobotTrackerType type,
    const Vector3 & initial_pose,  // [x, y, theta]
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(), double dt = 0.01);

  virtual ~RobotTracker() = default;

  auto predict(double dt) -> void;
  auto updateVision(const Vector3 & measurement) -> void;  // [x, y, theta]

  [[nodiscard]] auto getPosition() const -> Vector2;
  [[nodiscard]] auto getTheta() const -> double;
  [[nodiscard]] auto getVelocity() const -> Vector2;
  [[nodiscard]] auto getAngularVelocity() const -> double;
  [[nodiscard]] auto getCovariance() const -> Eigen::Matrix<double, 6, 6>;
  [[nodiscard]] auto getState() const -> Eigen::Matrix<double, 6, 1>;

  [[nodiscard]] auto getMahalanobisDistance(const Vector3 & measurement) const -> double;
  [[nodiscard]] auto isValidMeasurement(const Vector3 & measurement, double threshold = 5.99) const
    -> bool;

  [[nodiscard]] auto getLastUpdateTime() const -> rclcpp::Time { return last_update_time_; }
  auto setLastUpdateTime(const rclcpp::Time & time) -> void { last_update_time_ = time; }
  [[nodiscard]] auto getTrackingConfidence() const -> double { return tracking_confidence_; }
  [[nodiscard]] auto getRobotId() const -> uint8_t { return robot_id_; }
  [[nodiscard]] auto getTrackerType() const -> RobotTrackerType { return tracker_type_; }

  auto resetTracker(const Vector3 & pose) -> void;

private:
  uint8_t robot_id_;
  RobotTrackerType tracker_type_;

  // 状態ベクトル: [x, y, theta, vx, vy, omega] (consai_vision_tracker互換)
  Eigen::Matrix<double, 6, 1> state_;
  Eigen::Matrix<double, 6, 6> covariance_;
  Eigen::Matrix<double, 6, 6> process_noise_;
  Eigen::Matrix<double, 3, 3> measurement_noise_;

  rclcpp::Time last_update_time_;
  double tracking_confidence_;
  std::shared_ptr<rclcpp::Clock> clock_;
  double dt_;  // タイムステップ
  int outlier_count_;

  // consai_vision_tracker互換のメソッド
  auto initializeMatrices() -> void;
  auto getMeasurementMatrix() const -> Eigen::Matrix<double, 3, 6>;
  auto getStateTransitionMatrix(double dt) const -> Eigen::Matrix<double, 6, 6>;
  auto updateTrackingConfidence(bool measurement_received) -> void;
  auto normalizeAngle(double angle) const -> double;
  auto normalizeAngle(double from, double to) const -> double;
  auto isOutlier(const Vector3 & measurement) const -> bool;
};

// consai_vision_tracker互換の簡素化された味方ロボットトラッカー
class FriendlyRobotTracker : public RobotTracker
{
public:
  explicit FriendlyRobotTracker(
    uint8_t robot_id, const Vector3 & initial_pose,
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(), double dt = 0.01);

  // 基底クラスと同じ実装を使用（Vision専用）
};

// consai_vision_tracker互換の敵ロボットトラッカー
class EnemyRobotTracker : public RobotTracker
{
public:
  explicit EnemyRobotTracker(
    uint8_t robot_id, const Vector3 & initial_pose,
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(), double dt = 0.01);

  // 基底クラスと同じ実装を使用（Vision専用）
};

class RobotTrackerManager
{
public:
  explicit RobotTrackerManager(std::shared_ptr<rclcpp::Clock> clock);
  ~RobotTrackerManager() = default;

  auto processVisionDetection(
    uint8_t robot_id, RobotTrackerType type, const Vector3 & robot_pose,
    const rclcpp::Time & timestamp) -> void;

  auto predict(double dt) -> void;
  auto removeOldTrackers(double max_age_seconds = 2.0) -> void;

  [[nodiscard]] auto getRobotTracker(uint8_t robot_id, RobotTrackerType type) const
    -> std::shared_ptr<RobotTracker>;

  [[nodiscard]] auto getAllRobotInfo() const -> std::vector<crane_msgs::msg::RobotInfo>;

private:
  std::map<std::pair<uint8_t, RobotTrackerType>, std::shared_ptr<RobotTracker>> trackers_;
  std::shared_ptr<rclcpp::Clock> clock_;

  static constexpr double OUTLIER_THRESHOLD = 5.99;  // consai_vision_tracker互換
  static constexpr double MIN_TRACKING_CONFIDENCE = 0.2;
  static constexpr double VISIBILITY_CONTROL_VALUE = 0.005;  // consai_vision_tracker互換

  auto createNewTracker(uint8_t robot_id, RobotTrackerType type, const Vector3 & pose)
    -> std::shared_ptr<RobotTracker>;
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__ROBOT_TRACKER_HPP_
