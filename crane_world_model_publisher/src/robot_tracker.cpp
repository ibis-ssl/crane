// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/robot_tracker.hpp"
#include <crane_geometry/geometry_operations.hpp>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// consai_vision_tracker互換の定数と関数
namespace
{
// visibilityの操作量 (consai_vision_tracker互換)
static const double VISIBILITY_CONTROL_VALUE = 0.005;

double normalizeAngle(double angle)
{
  while (angle >= M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle <= -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double normalizeAngle(double from, double to) { return from + normalizeAngle(to - from); }
}  // namespace

namespace crane
{

RobotTracker::RobotTracker(
  uint8_t robot_id, RobotTrackerType type, const Vector3 & initial_pose,
  std::shared_ptr<rclcpp::Clock> clock, double dt)
: robot_id_(robot_id), tracker_type_(type), clock_(clock), dt_(dt), outlier_count_(0)
{
  state_ = Eigen::Matrix<double, 6, 1>::Zero();
  state_(idx(StateIndex::X)) = initial_pose(0);      // x
  state_(idx(StateIndex::Y)) = initial_pose(1);      // y
  state_(idx(StateIndex::THETA)) = initial_pose(2);  // theta

  tracking_confidence_ = 1.0;
  last_update_time_ = clock_->now();

  initializeMatrices();
}

auto RobotTracker::initializeMatrices() -> void
{
  // consai_vision_tracker互換の初期化

  // 初期共分散行列 (consai_vision_tracker互換)
  covariance_ = Eigen::Matrix<double, 6, 6>::Identity() * 100.0;

  // プロセスノイズ
  // 位置、速度の変化をシステムノイズで表現する
  const double MAX_LINEAR_ACC_MPS = 6.0;                                               // [m/s²]
  const double MAX_ANGULAR_ACC_RADPS = 12.0;                                           // [rad/s²]
  const double MAX_LINEAR_ACCEL_IN_DT = MAX_LINEAR_ACC_MPS * dt_;                      // [m/s]
  const double MAX_ANGULAR_ACCEL_IN_DT = MAX_ANGULAR_ACC_RADPS * dt_;                  // [rad/s]
  const double MAX_LINEAR_MOVEMENT_IN_DT = MAX_LINEAR_ACC_MPS / 2 * std::pow(dt_, 2);  // [m]
  const double MAX_ANGULAR_MOVEMENT_IN_DT = MAX_ANGULAR_ACC_RADPS / 2 * std::pow(dt_, 2);  // [rad]

  process_noise_ = Eigen::Matrix<double, 6, 6>::Zero();
  process_noise_(0, 0) = std::pow(MAX_LINEAR_MOVEMENT_IN_DT, 2);   // x
  process_noise_(1, 1) = std::pow(MAX_LINEAR_MOVEMENT_IN_DT, 2);   // y
  process_noise_(2, 2) = std::pow(MAX_ANGULAR_MOVEMENT_IN_DT, 2);  // theta
  process_noise_(3, 3) = std::pow(MAX_LINEAR_ACCEL_IN_DT, 2);      // vx
  process_noise_(4, 4) = std::pow(MAX_LINEAR_ACCEL_IN_DT, 2);      // vy
  process_noise_(5, 5) = std::pow(MAX_ANGULAR_ACCEL_IN_DT, 2);     // omega

  // 観測ノイズ
  measurement_noise_ = Eigen::Matrix<double, 3, 3>::Zero();
  measurement_noise_(0, 0) = std::pow(0.02, 2);                // x
  measurement_noise_(1, 1) = std::pow(0.02, 2);                // y
  measurement_noise_(2, 2) = std::pow(3.0 * M_PI / 180.0, 2);  // theta
}

auto RobotTracker::predict(double dt) -> void
{
  if (dt <= 0.0) return;

  auto F = getStateTransitionMatrix(dt);

  // 状態予測 (consai_vision_tracker互換)
  state_ = F * state_;

  // 共分散予測 (consai_vision_tracker互換)
  covariance_ = F * covariance_ * F.transpose() + process_noise_ * dt;

  // 信頼度減衰 (consai_vision_tracker互換)
  tracking_confidence_ = std::max(0.0, tracking_confidence_ - VISIBILITY_CONTROL_VALUE);
}

auto RobotTracker::updateVision(const Eigen::Vector3d & measurement) -> void
{
  // 外れ値チェック (consai_vision_tracker互換)
  if (isOutlier(measurement)) {
    // 外れ値が連続できたら、観測値をそのまま使用する（誘拐対応）
    outlier_count_++;
    if (outlier_count_ <= 10) {  // OUTLIER_COUNT_THRESHOLD
      updateTrackingConfidence(false);
      return;
    }
  } else {
    outlier_count_ = 0;
  }

  auto H = getMeasurementMatrix();

  // イノベーション (観測残差) (consai_vision_tracker互換)
  Eigen::Vector3d innovation = measurement - H * state_;

  // 角度差正規化 (consai_vision_tracker互換)
  innovation(2) = crane::getAngleDiff(measurement(2), state_(idx(StateIndex::THETA)));

  // イノベーション共分散
  Eigen::Matrix3d S = H * covariance_ * H.transpose() + measurement_noise_;

  // カルマンゲイン
  Eigen::Matrix<double, 6, 3> K = covariance_ * H.transpose() * S.inverse();

  // 状態更新 (consai_vision_tracker互換)
  state_ = state_ + K * innovation;

  // 角度正規化
  state_(idx(StateIndex::THETA)) = ::normalizeAngle(state_(idx(StateIndex::THETA)));

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

auto RobotTracker::getMahalanobisDistance(const Eigen::Vector3d & measurement) const -> double
{
  // consai_vision_tracker互換の簡略化されたマハラノビス距離計算
  double diff_x = measurement(0) - state_(idx(StateIndex::X));
  double diff_y = measurement(1) - state_(idx(StateIndex::Y));
  double covariance_x = covariance_(0, 0);
  double covariance_y = covariance_(1, 1);

  // 0除算を避ける
  if (std::fabs(covariance_x) < 1E-15 || std::fabs(covariance_y) < 1E-15) {
    return 0.0;
  }

  return std::sqrt(std::pow(diff_x, 2) / covariance_x + std::pow(diff_y, 2) / covariance_y);
}

auto RobotTracker::isValidMeasurement(const Eigen::Vector3d & measurement, double threshold) const
  -> bool
{
  return getMahalanobisDistance(measurement) < threshold;
}

// consai_vision_tracker互換の新しいメソッド群
auto RobotTracker::getStateTransitionMatrix(double dt) const -> Eigen::Matrix<double, 6, 6>
{
  // consai_vision_tracker互換の状態遷移行列
  Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();

  // pos(t+1) = pos(t) + vel(t)*dt
  F(0, 3) = dt;  // x = x + vx * dt
  F(1, 4) = dt;  // y = y + vy * dt
  F(2, 5) = dt;  // theta = theta + omega * dt

  // 速度は等速運動モデル（摩擦なし）
  // F(3,3) = F(4,4) = F(5,5) = 1.0 (既にIdentityで設定済み)

  return F;
}

auto RobotTracker::isOutlier(const Eigen::Vector3d & measurement) const -> bool
{
  // 観測が外れ値かどうか判定する (consai_vision_tracker互換)
  const double THRESHOLD = 5.99;  // 自由度2、棄却率5%のしきい値

  return getMahalanobisDistance(measurement) > THRESHOLD;
}

auto RobotTracker::getPosition() const -> Eigen::Vector2d { return state_.head<2>(); }

auto RobotTracker::getTheta() const -> double { return state_(idx(StateIndex::THETA)); }

auto RobotTracker::getVelocity() const -> Eigen::Vector2d { return state_.segment<2>(3); }

auto RobotTracker::getAngularVelocity() const -> double { return state_(idx(StateIndex::OMEGA)); }

auto RobotTracker::getCovariance() const -> Eigen::Matrix<double, 6, 6> { return covariance_; }

auto RobotTracker::getState() const -> Eigen::Matrix<double, 6, 1> { return state_; }

auto RobotTracker::updateTrackingConfidence(bool measurement_received) -> void
{
  // consai_vision_tracker互換のvisibility管理
  if (measurement_received) {
    // 観測値があればvisibilityをn倍のレートで上げる
    tracking_confidence_ += VISIBILITY_CONTROL_VALUE * 5.0;
    if (tracking_confidence_ > 1.0) {
      tracking_confidence_ = 1.0;
    }
  } else {
    // 観測値が無い場合のvisibilityを下げる
    tracking_confidence_ -= VISIBILITY_CONTROL_VALUE;
    if (tracking_confidence_ <= 0.0) {
      tracking_confidence_ = 0.0;
      // visibilityが0になったらカルマンフィルタの演算を初期化
      resetTracker(Eigen::Vector3d(state_(0), state_(1), state_(2)));
    }
  }
}

auto RobotTracker::resetTracker(const Eigen::Vector3d & pose) -> void
{
  // consai_vision_tracker互換のリセット処理
  state_ = Eigen::Matrix<double, 6, 1>::Zero();
  state_(idx(StateIndex::X)) = pose(0);
  state_(idx(StateIndex::Y)) = pose(1);
  state_(idx(StateIndex::THETA)) = pose(2);
  tracking_confidence_ = 1.0;
  outlier_count_ = 0;
  initializeMatrices();
}

FriendlyRobotTracker::FriendlyRobotTracker(
  uint8_t robot_id, const Vector3 & initial_pose, std::shared_ptr<rclcpp::Clock> clock, double dt)
: RobotTracker(robot_id, RobotTrackerType::FRIENDLY, initial_pose, clock, dt)
{
  // consai_vision_tracker互換 - 簡素化された実装
  // 基底クラスと同じ機能のみ使用
}

EnemyRobotTracker::EnemyRobotTracker(
  uint8_t robot_id, const Vector3 & initial_pose, std::shared_ptr<rclcpp::Clock> clock, double dt)
: RobotTracker(robot_id, RobotTrackerType::ENEMY, initial_pose, clock, dt)
{
  // consai_vision_tracker互換 - 基底クラスと同じ実装を使用
}

RobotTrackerManager::RobotTrackerManager(std::shared_ptr<rclcpp::Clock> clock) : clock_(clock) {}

auto RobotTrackerManager::processVisionDetection(
  uint8_t robot_id, RobotTrackerType type, const Vector3 & robot_pose,
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
  // consai_vision_tracker互換 - デフォルトdt=0.01を使用
  if (type == RobotTrackerType::FRIENDLY) {
    return std::make_shared<FriendlyRobotTracker>(robot_id, pose, clock_, 0.01);
  } else {
    return std::make_shared<EnemyRobotTracker>(robot_id, pose, clock_, 0.01);
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

      // consai_vision_tracker互換 - 品質情報は削除

      robot_infos.push_back(info);
    }
  }

  return robot_infos;
}
}  // namespace crane
