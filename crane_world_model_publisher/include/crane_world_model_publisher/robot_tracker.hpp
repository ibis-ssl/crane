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

// 拡張状態ベクトルのインデックス定義（8次元: 基本6次元 + バイアス項）
enum class ExtendedStateIndex : int {
  X = 0,       // X座標 (m)
  Y = 1,       // Y座標 (m)
  THETA = 2,   // 姿勢角 (rad)
  VX = 3,      // X方向速度 (m/s)
  VY = 4,      // Y方向速度 (m/s)
  OMEGA = 5,   // 角速度 (rad/s)
  BIAS_X = 6,  // Xバイアス項
  BIAS_Y = 7   // Yバイアス項
};

// 状態インデックス用ヘルパー関数（簡潔なアクセスのため）
constexpr int idx(StateIndex index) { return static_cast<int>(index); }
constexpr int idx(ExtendedStateIndex index) { return static_cast<int>(index); }

class RobotPhysicsModel
{
public:
  struct Config
  {
    double max_velocity = 3.0;           // 最大速度 (m/s)
    double max_acceleration = 2.0;       // 最大加速度 (m/s²)
    double max_angular_velocity = 12.0;  // 最大角速度 (rad/s)
    double friction_coefficient = 0.8;   // 摩擦係数
    double mass = 2.7;                   // ロボット質量 (kg)
  };

  RobotPhysicsModel() : config_(Config{}) {}
  explicit RobotPhysicsModel(const Config & config) : config_(config) {}

  auto getStateTransitionMatrix(double dt) const -> Eigen::Matrix<double, 6, 6>;
  auto getControlInputMatrix(double dt) const -> Eigen::Matrix<double, 6, 2>;
  auto applyPhysicsConstraints(Eigen::Matrix<double, 6, 1> & state) const -> void;
  auto getConfig() const -> const Config & { return config_; }

private:
  Config config_;
};

class RobotTracker
{
public:
  explicit RobotTracker(
    uint8_t robot_id, RobotTrackerType type,
    const Vector3 & initial_pose,  // [x, y, theta]
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>());

  virtual ~RobotTracker() = default;

  virtual auto predict(double dt) -> void;
  virtual auto updateVision(const Vector3 & measurement) -> void;  // [x, y, theta]

  [[nodiscard]] auto getPosition() const -> Vector2;
  [[nodiscard]] auto getTheta() const -> double;
  [[nodiscard]] auto getVelocity() const -> Vector2;
  [[nodiscard]] auto getAngularVelocity() const -> double;
  [[nodiscard]] auto getCovariance() const -> Eigen::Matrix<double, 6, 6>;
  [[nodiscard]] auto getState() const -> Eigen::Matrix<double, 6, 1>;

  [[nodiscard]] auto getMahalanobisDistance(const Vector3 & measurement) const -> double;
  [[nodiscard]] auto isValidMeasurement(const Vector3 & measurement, double threshold = 9.0) const
    -> bool;

  [[nodiscard]] auto getLastUpdateTime() const -> rclcpp::Time { return last_update_time_; }
  auto setLastUpdateTime(const rclcpp::Time & time) -> void { last_update_time_ = time; }
  [[nodiscard]] auto getTrackingConfidence() const -> double { return tracking_confidence_; }
  [[nodiscard]] auto getRobotId() const -> uint8_t { return robot_id_; }
  [[nodiscard]] auto getTrackerType() const -> RobotTrackerType { return tracker_type_; }

  auto resetTracker(const Vector3 & pose) -> void;

protected:
  uint8_t robot_id_;
  RobotTrackerType tracker_type_;

  // 状態ベクトル: [x, y, theta, vx, vy, omega]
  Eigen::Matrix<double, 6, 1> state_;
  Eigen::Matrix<double, 6, 6> covariance_;
  Eigen::Matrix<double, 6, 6> process_noise_;
  Eigen::Matrix<double, 3, 3> measurement_noise_;

  rclcpp::Time last_update_time_;
  double tracking_confidence_;

  std::shared_ptr<RobotPhysicsModel> physics_model_;
  std::shared_ptr<rclcpp::Clock> clock_;

  virtual auto initializeMatrices() -> void;
  auto getMeasurementMatrix() const -> Eigen::Matrix<double, 3, 6>;
  auto updateTrackingConfidence(bool measurement_received) -> void;
};

class FriendlyRobotTracker : public RobotTracker
{
public:
  explicit FriendlyRobotTracker(
    uint8_t robot_id, const Vector3 & initial_pose,
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>());

  auto predict(double dt) -> void override;
  auto updateOdometry(const Vector2 & odom_pos, const Vector2 & odom_vel, double yaw_angle) -> void;
  auto updateMouseSensor(const Vector2 & mouse_vel) -> void;
  auto updateCommand(const Vector2 & cmd_vel, double cmd_omega) -> void;
  auto updateFeedback(const crane_msgs::msg::RobotFeedback & feedback) -> void;

  [[nodiscard]] auto getQualityScore() const -> double;

private:
  // 拡張状態ベクトル: [x, y, theta, vx, vy, omega, bias_x, bias_y]
  Eigen::Matrix<double, 8, 1> extended_state_;
  Eigen::Matrix<double, 8, 8> extended_covariance_;

  Vector2 last_command_velocity_;
  double last_command_omega_;
  double odometry_quality_;

  auto initializeMatrices() -> void override;
  auto evaluateOdometryQuality(const crane_msgs::msg::RobotFeedback & feedback) -> void;
  auto getExtendedStateTransitionMatrix(double dt) const -> Eigen::Matrix<double, 8, 8>;
};

class EnemyRobotTracker : public RobotTracker
{
public:
  explicit EnemyRobotTracker(
    uint8_t robot_id, const Vector3 & initial_pose,
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>());

  // Vision のみを使用する従来型EKF
  // 基底クラスの実装をそのまま使用
};

class RobotTrackerManager
{
public:
  explicit RobotTrackerManager(std::shared_ptr<rclcpp::Clock> clock);
  ~RobotTrackerManager() = default;

  auto processVisionDetection(
    uint8_t robot_id, RobotTrackerType type, const Vector3 & robot_pose,
    const rclcpp::Time & timestamp) -> void;

  auto updateFriendlyRobotFeedback(
    uint8_t robot_id, const crane_msgs::msg::RobotFeedback & feedback) -> void;

  auto updateFriendlyRobotCommand(uint8_t robot_id, const Vector2 & cmd_vel, double cmd_omega)
    -> void;

  auto predict(double dt) -> void;
  auto removeOldTrackers(double max_age_seconds = 2.0) -> void;

  [[nodiscard]] auto getRobotTracker(uint8_t robot_id, RobotTrackerType type) const
    -> std::shared_ptr<RobotTracker>;

  [[nodiscard]] auto getAllRobotInfo() const -> std::vector<crane_msgs::msg::RobotInfo>;

private:
  std::map<std::pair<uint8_t, RobotTrackerType>, std::shared_ptr<RobotTracker>> trackers_;
  std::shared_ptr<rclcpp::Clock> clock_;

  static constexpr double OUTLIER_THRESHOLD = 9.0;
  static constexpr double MIN_TRACKING_CONFIDENCE = 0.2;

  auto createNewTracker(uint8_t robot_id, RobotTrackerType type, const Vector3 & pose)
    -> std::shared_ptr<RobotTracker>;
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__ROBOT_TRACKER_HPP_
