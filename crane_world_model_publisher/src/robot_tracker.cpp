// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/robot_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <crane_geometry/geometry_operations.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace crane
{
auto RobotPhysicsModel::getStateTransitionMatrix(double dt) const -> Eigen::Matrix<double, 6, 6>
{
  Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();

  // 位置 = 位置 + 速度 * dt
  F(idx(StateIndex::X), idx(StateIndex::VX)) = dt;         // x = x + vx * dt
  F(idx(StateIndex::Y), idx(StateIndex::VY)) = dt;         // y = y + vy * dt
  F(idx(StateIndex::THETA), idx(StateIndex::OMEGA)) = dt;  // theta = theta + omega * dt

  // 摩擦減衰 (λ = 1 - friction * dt)
  double friction_decay = 1.0 - config_.friction_coefficient * dt;
  friction_decay = std::max(0.0, friction_decay);

  F(idx(StateIndex::VX), idx(StateIndex::VX)) = friction_decay;        // vx 減衰
  F(idx(StateIndex::VY), idx(StateIndex::VY)) = friction_decay;        // vy 減衰
  F(idx(StateIndex::OMEGA), idx(StateIndex::OMEGA)) = friction_decay;  // omega 減衰

  return F;
}

auto RobotPhysicsModel::getControlInputMatrix(double dt) const -> Eigen::Matrix<double, 6, 2>
{
  Eigen::Matrix<double, 6, 2> B = Eigen::Matrix<double, 6, 2>::Zero();

  // 制御入力: [ax, ay] (加速度)
  B(idx(StateIndex::VX), 0) = dt;  // vx += ax * dt
  B(idx(StateIndex::VY), 1) = dt;  // vy += ay * dt

  return B;
}

auto RobotPhysicsModel::applyPhysicsConstraints(Eigen::Matrix<double, 6, 1> & state) const -> void
{
  // 速度制約
  double speed = std::sqrt(
    state(idx(StateIndex::VX)) * state(idx(StateIndex::VX)) +
    state(idx(StateIndex::VY)) * state(idx(StateIndex::VY)));
  if (speed > config_.max_velocity) {
    double scale = config_.max_velocity / speed;
    state(idx(StateIndex::VX)) *= scale;
    state(idx(StateIndex::VY)) *= scale;
  }

  // 角速度制約
  state(idx(StateIndex::OMEGA)) = std::clamp(
    state(idx(StateIndex::OMEGA)), -config_.max_angular_velocity, config_.max_angular_velocity);

  // 角度正規化 [-π, π]
  state(idx(StateIndex::THETA)) = normalizeAngle(state(idx(StateIndex::THETA)));
}

RobotTracker::RobotTracker(
  uint8_t robot_id, RobotTrackerType type, const Vector3 & initial_pose,
  std::shared_ptr<rclcpp::Clock> clock)
: robot_id_(robot_id), tracker_type_(type), clock_(clock)
{
  state_ = Eigen::Matrix<double, 6, 1>::Zero();
  state_(idx(StateIndex::X)) = initial_pose(0);      // x
  state_(idx(StateIndex::Y)) = initial_pose(1);      // y
  state_(idx(StateIndex::THETA)) = initial_pose(2);  // theta

  tracking_confidence_ = 1.0;
  last_update_time_ = clock_->now();
  physics_model_ = std::make_shared<RobotPhysicsModel>();

  initializeMatrices();
}

auto RobotTracker::initializeMatrices() -> void
{
  // 初期共分散 (位置: 0.1m, 角度: 0.1rad, 速度: 0.5m/s)
  covariance_ = Eigen::Matrix<double, 6, 6>::Identity();
  covariance_.diagonal() << 0.1, 0.1, 0.1, 0.5, 0.5, 0.5;

  // プロセスノイズ (加速度のランダムウォーク)
  process_noise_ = Eigen::Matrix<double, 6, 6>::Zero();
  process_noise_.diagonal() << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1;

  // 観測ノイズ (Vision精度)
  measurement_noise_ = Eigen::Matrix<double, 3, 3>::Identity();
  measurement_noise_.diagonal() << 0.005, 0.005, 0.02;  // [x, y, theta]
}

auto RobotTracker::predict(double dt) -> void
{
  if (dt <= 0.0) return;

  auto F = physics_model_->getStateTransitionMatrix(dt);

  // 状態予測
  state_ = F * state_;

  // 物理制約適用
  physics_model_->applyPhysicsConstraints(state_);

  // 共分散予測
  covariance_ = F * covariance_ * F.transpose() + process_noise_ * dt;

  // 信頼度減衰
  tracking_confidence_ = std::max(0.0, tracking_confidence_ - 0.1 * dt);
}

auto RobotTracker::updateVision(const Vector3 & measurement) -> void
{
  auto H = getMeasurementMatrix();

  // イノベーション (観測残差)
  Vector3 innovation = measurement - H * state_;

  // 角度差正規化
  innovation(idx(StateIndex::THETA)) = normalizeAngle(innovation(idx(StateIndex::THETA)));

  // イノベーション共分散
  Eigen::Matrix3d S = H * covariance_ * H.transpose() + measurement_noise_;

  // カルマンゲイン
  Eigen::Matrix<double, 6, 3> K = covariance_ * H.transpose() * S.inverse();

  // 状態更新
  state_ = state_ + K * innovation;

  // 物理制約適用
  physics_model_->applyPhysicsConstraints(state_);

  // 共分散更新
  Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
  covariance_ = (I - K * H) * covariance_;

  updateTrackingConfidence(true);
  last_update_time_ = clock_->now();
}

auto RobotTracker::getMeasurementMatrix() const -> Eigen::Matrix<double, 3, 6>
{
  Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();  // [x, y, theta]
  return H;
}

auto RobotTracker::getMahalanobisDistance(const Vector3 & measurement) const -> double
{
  auto H = getMeasurementMatrix();
  Vector3 innovation = measurement - H * state_;

  // 角度差正規化
  innovation(idx(StateIndex::THETA)) = normalizeAngle(innovation(idx(StateIndex::THETA)));

  Eigen::Matrix3d S = H * covariance_ * H.transpose() + measurement_noise_;

  return std::sqrt(innovation.transpose() * S.inverse() * innovation);
}

auto RobotTracker::isValidMeasurement(const Vector3 & measurement, double threshold) const -> bool
{
  return getMahalanobisDistance(measurement) < threshold;
}

auto RobotTracker::getPosition() const -> Eigen::Vector2d { return state_.head<2>(); }

auto RobotTracker::getTheta() const -> double { return state_(idx(StateIndex::THETA)); }

auto RobotTracker::getVelocity() const -> Eigen::Vector2d { return state_.segment<2>(3); }

auto RobotTracker::getAngularVelocity() const -> double { return state_(idx(StateIndex::OMEGA)); }

auto RobotTracker::getCovariance() const -> Eigen::Matrix<double, 6, 6> { return covariance_; }

auto RobotTracker::getState() const -> Eigen::Matrix<double, 6, 1> { return state_; }

auto RobotTracker::updateTrackingConfidence(bool measurement_received) -> void
{
  if (measurement_received) {
    tracking_confidence_ = std::min(1.0, tracking_confidence_ + 0.2);
  } else {
    double time_since_update = (clock_->now() - last_update_time_).seconds();
    tracking_confidence_ = std::max(0.0, tracking_confidence_ - 0.1 * time_since_update);
  }
}

auto RobotTracker::resetTracker(const Vector3 & pose) -> void
{
  state_ = Eigen::Matrix<double, 6, 1>::Zero();
  state_(idx(StateIndex::X)) = pose(0);
  state_(idx(StateIndex::Y)) = pose(1);
  state_(idx(StateIndex::THETA)) = pose(2);
  tracking_confidence_ = 1.0;
  initializeMatrices();
}

FriendlyRobotTracker::FriendlyRobotTracker(
  uint8_t robot_id, const Eigen::Vector3d & initial_pose, std::shared_ptr<rclcpp::Clock> clock)
: RobotTracker(robot_id, RobotTrackerType::FRIENDLY, initial_pose, clock)
{
  extended_state_ = Eigen::Matrix<double, 8, 1>::Zero();
  extended_state_.head<6>() = state_;

  last_command_velocity_ = Eigen::Vector2d::Zero();
  last_command_omega_ = 0.0;
  odometry_quality_ = 1.0;

  initializeMatrices();
}

auto FriendlyRobotTracker::initializeMatrices() -> void
{
  RobotTracker::initializeMatrices();

  // 拡張共分散行列: [x, y, theta, vx, vy, omega, bias_x, bias_y]
  extended_covariance_ = Eigen::Matrix<double, 8, 8>::Identity();
  extended_covariance_.block<6, 6>(0, 0) = covariance_;
  extended_covariance_(6, 6) = 0.01;  // bias_x 初期共分散
  extended_covariance_(7, 7) = 0.01;  // bias_y 初期共分散
}

auto FriendlyRobotTracker::predict(double dt) -> void
{
  if (dt <= 0.0) return;

  auto F = getExtendedStateTransitionMatrix(dt);

  // 拡張状態予測
  extended_state_ = F * extended_state_;

  // 基底クラスの状態を更新
  state_ = extended_state_.head<6>();
  physics_model_->applyPhysicsConstraints(state_);
  extended_state_.head<6>() = state_;

  // 拡張共分散予測
  Eigen::Matrix<double, 8, 8> Q = Eigen::Matrix<double, 8, 8>::Zero();
  Q.block<6, 6>(0, 0) = process_noise_;
  Q(6, 6) = 0.001 * dt;  // バイアス項のプロセスノイズ
  Q(7, 7) = 0.001 * dt;

  extended_covariance_ = F * extended_covariance_ * F.transpose() + Q * dt;
  covariance_ = extended_covariance_.block<6, 6>(0, 0);

  updateTrackingConfidence(false);
}

auto FriendlyRobotTracker::getExtendedStateTransitionMatrix(double dt) const
  -> Eigen::Matrix<double, 8, 8>
{
  Eigen::Matrix<double, 8, 8> F = Eigen::Matrix<double, 8, 8>::Identity();

  // 基底状態遷移
  F.block<6, 6>(0, 0) = physics_model_->getStateTransitionMatrix(dt);

  // バイアス項は定数 (bias_x, bias_y は変化しない)
  F(6, 6) = 1.0;
  F(7, 7) = 1.0;

  return F;
}

auto FriendlyRobotTracker::updateOdometry(
  const Eigen::Vector2d & odom_pos, const Eigen::Vector2d & odom_vel, double yaw_angle) -> void
{
  // オドメトリ観測: [x + bias_x, y + bias_y, theta]
  Eigen::Matrix<double, 3, 8> H = Eigen::Matrix<double, 3, 8>::Zero();
  H(0, idx(ExtendedStateIndex::X)) = 1.0;
  H(0, idx(ExtendedStateIndex::BIAS_X)) = 1.0;  // x観測 = x + bias_x
  H(1, idx(ExtendedStateIndex::Y)) = 1.0;
  H(1, idx(ExtendedStateIndex::BIAS_Y)) = 1.0;  // y観測 = y + bias_y
  H(2, idx(ExtendedStateIndex::THETA)) = 1.0;   // theta観測 = theta

  Eigen::Vector3d odom_measurement;
  odom_measurement << odom_pos(0), odom_pos(1), yaw_angle;

  // 品質に基づく観測ノイズ調整
  Eigen::Matrix3d R = measurement_noise_;
  R *= (2.0 - odometry_quality_);  // 品質が悪いほどノイズ増加

  // 拡張EKF更新
  Eigen::Vector3d innovation = odom_measurement - H * extended_state_;
  innovation(idx(ExtendedStateIndex::THETA)) =
    normalizeAngle(innovation(idx(ExtendedStateIndex::THETA)));

  Eigen::Matrix3d S = H * extended_covariance_ * H.transpose() + R;
  Eigen::Matrix<double, 8, 3> K = extended_covariance_ * H.transpose() * S.inverse();

  extended_state_ = extended_state_ + K * innovation;
  state_ = extended_state_.head<6>();

  Eigen::Matrix<double, 8, 8> I = Eigen::Matrix<double, 8, 8>::Identity();
  extended_covariance_ = (I - K * H) * extended_covariance_;
  covariance_ = extended_covariance_.block<6, 6>(0, 0);

  updateTrackingConfidence(true);
}

auto FriendlyRobotTracker::updateMouseSensor(const Eigen::Vector2d & mouse_vel) -> void
{
  // マウスセンサによる速度観測
  Eigen::Matrix<double, 2, 8> H = Eigen::Matrix<double, 2, 8>::Zero();
  H(0, idx(ExtendedStateIndex::VX)) = 1.0;  // vx観測
  H(1, idx(ExtendedStateIndex::VY)) = 1.0;  // vy観測

  Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 0.1;  // マウスセンサノイズ

  Eigen::Vector2d innovation = mouse_vel - H * extended_state_;
  Eigen::Matrix2d S = H * extended_covariance_ * H.transpose() + R;
  Eigen::Matrix<double, 8, 2> K = extended_covariance_ * H.transpose() * S.inverse();

  extended_state_ = extended_state_ + K * innovation;
  state_ = extended_state_.head<6>();

  Eigen::Matrix<double, 8, 8> I = Eigen::Matrix<double, 8, 8>::Identity();
  extended_covariance_ = (I - K * H) * extended_covariance_;
  covariance_ = extended_covariance_.block<6, 6>(0, 0);
}

auto FriendlyRobotTracker::updateCommand(const Eigen::Vector2d & cmd_vel, double cmd_omega) -> void
{
  last_command_velocity_ = cmd_vel;
  last_command_omega_ = cmd_omega;
}

auto FriendlyRobotTracker::updateFeedback(const crane_msgs::msg::RobotFeedback & feedback) -> void
{
  // フィードバック品質評価
  evaluateOdometryQuality(feedback);

  // オドメトリ更新
  Eigen::Vector2d odom_pos(feedback.odom[0], feedback.odom[1]);
  Eigen::Vector2d odom_vel(feedback.odom_speed[0], feedback.odom_speed[1]);
  updateOdometry(odom_pos, odom_vel, feedback.yaw_angle * M_PI / 180.0);

  // マウスセンサ更新（利用可能な場合）
  if (feedback.mouse_vel[0] != 0.0 || feedback.mouse_vel[1] != 0.0) {
    Eigen::Vector2d mouse_vel(feedback.mouse_vel[0], feedback.mouse_vel[1]);
    updateMouseSensor(mouse_vel);
  }
}

auto FriendlyRobotTracker::evaluateOdometryQuality(const crane_msgs::msg::RobotFeedback & feedback)
  -> void
{
  double quality = 1.0;

  // モータ電流による品質評価
  for (int i = 0; i < 4; ++i) {
    if (feedback.motor_current[i] > 5.0) {  // 高電流 = スリップの可能性
      quality *= 0.8;
    }
  }

  // 温度による品質評価
  for (int i = 0; i < 7; ++i) {
    if (feedback.temperatures[i] > 60.0) {  // 高温 = 性能劣化
      quality *= 0.9;
    }
  }

  // エラー状態による品質評価
  if (feedback.error_id != 0) {
    quality *= 0.5;
  }

  // 指数移動平均でスムージング
  odometry_quality_ = 0.9 * odometry_quality_ + 0.1 * quality;
}

auto FriendlyRobotTracker::getQualityScore() const -> double { return odometry_quality_; }

EnemyRobotTracker::EnemyRobotTracker(
  uint8_t robot_id, const Eigen::Vector3d & initial_pose, std::shared_ptr<rclcpp::Clock> clock)
: RobotTracker(robot_id, RobotTrackerType::ENEMY, initial_pose, clock)
{
  // 敵ロボットはVisionのみなので基底クラスの実装をそのまま使用
}

RobotTrackerManager::RobotTrackerManager(std::shared_ptr<rclcpp::Clock> clock) : clock_(clock) {}

auto RobotTrackerManager::processVisionDetection(
  uint8_t robot_id, RobotTrackerType type, const Eigen::Vector3d & robot_pose,
  const rclcpp::Time & timestamp) -> void
{
  auto key = std::make_pair(robot_id, type);
  auto tracker = trackers_.find(key);

  if (tracker != trackers_.end()) {
    double dt = (timestamp - tracker->second->getLastUpdateTime()).seconds();
    if (dt > 0.0) {
      tracker->second->predict(dt);
    }
    tracker->second->updateVision(robot_pose);
    tracker->second->setLastUpdateTime(timestamp);
  } else {
    auto new_tracker = createNewTracker(robot_id, type, robot_pose);
    new_tracker->setLastUpdateTime(timestamp);
    trackers_[key] = new_tracker;
  }
}

auto RobotTrackerManager::createNewTracker(
  uint8_t robot_id, RobotTrackerType type, const Eigen::Vector3d & pose)
  -> std::shared_ptr<RobotTracker>
{
  if (type == RobotTrackerType::FRIENDLY) {
    return std::make_shared<FriendlyRobotTracker>(robot_id, pose, clock_);
  } else {
    return std::make_shared<EnemyRobotTracker>(robot_id, pose, clock_);
  }
}

auto RobotTrackerManager::updateFriendlyRobotFeedback(
  uint8_t robot_id, const crane_msgs::msg::RobotFeedback & feedback) -> void
{
  auto key = std::make_pair(robot_id, RobotTrackerType::FRIENDLY);
  auto tracker = trackers_.find(key);

  if (tracker != trackers_.end()) {
    auto friendly_tracker = std::dynamic_pointer_cast<FriendlyRobotTracker>(tracker->second);
    if (friendly_tracker) {
      friendly_tracker->updateFeedback(feedback);
    }
  }
}

auto RobotTrackerManager::updateFriendlyRobotCommand(
  uint8_t robot_id, const Eigen::Vector2d & cmd_vel, double cmd_omega) -> void
{
  auto key = std::make_pair(robot_id, RobotTrackerType::FRIENDLY);
  auto tracker = trackers_.find(key);

  if (tracker != trackers_.end()) {
    auto friendly_tracker = std::dynamic_pointer_cast<FriendlyRobotTracker>(tracker->second);
    if (friendly_tracker) {
      friendly_tracker->updateCommand(cmd_vel, cmd_omega);
    }
  }
}

auto RobotTrackerManager::predict(double dt) -> void
{
  for (auto & [key, tracker] : trackers_) {
    tracker->predict(dt);
  }
}

auto RobotTrackerManager::removeOldTrackers(double max_age_seconds) -> void
{
  auto current_time = clock_->now();

  for (auto it = trackers_.begin(); it != trackers_.end();) {
    double age = (current_time - it->second->getLastUpdateTime()).seconds();
    if (age > max_age_seconds || it->second->getTrackingConfidence() < MIN_TRACKING_CONFIDENCE) {
      it = trackers_.erase(it);
    } else {
      ++it;
    }
  }
}

auto RobotTrackerManager::getRobotTracker(uint8_t robot_id, RobotTrackerType type) const
  -> std::shared_ptr<RobotTracker>
{
  auto key = std::make_pair(robot_id, type);
  auto it = trackers_.find(key);
  return (it != trackers_.end()) ? it->second : nullptr;
}

auto RobotTrackerManager::getAllRobotInfo() const -> std::vector<crane_msgs::msg::RobotInfo>
{
  std::vector<crane_msgs::msg::RobotInfo> robot_infos;

  for (const auto & [key, tracker] : trackers_) {
    if (tracker->getTrackingConfidence() > MIN_TRACKING_CONFIDENCE) {
      crane_msgs::msg::RobotInfo info;
      info.id = tracker->getRobotId();

      auto pos = tracker->getPosition();
      info.pose.x = pos(0);
      info.pose.y = pos(1);
      info.pose.theta = tracker->getTheta();

      auto vel = tracker->getVelocity();
      info.velocity.x = vel(0);
      info.velocity.y = vel(1);
      info.velocity.theta = tracker->getAngularVelocity();

      // フィードバック品質情報（味方ロボットのみ）
      if (tracker->getTrackerType() == RobotTrackerType::FRIENDLY) {
        auto friendly_tracker = std::dynamic_pointer_cast<const FriendlyRobotTracker>(tracker);
        if (friendly_tracker) {
          // 将来的にメッセージに品質情報を追加する場合
          // info.quality_score = friendly_tracker->getQualityScore();
        }
      }

      robot_infos.push_back(info);
    }
  }

  return robot_infos;
}
}  // namespace crane
