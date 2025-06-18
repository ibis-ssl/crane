// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/ball_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace crane
{
BallTracker::BallTracker(const Eigen::Vector3d & initial_position, Ball::State initial_state, std::shared_ptr<BallPhysicsModel> physics_model)
{
  state_ = Eigen::Matrix<double, 6, 1>::Zero();
  state_.head<3>() = initial_position;
  
  ball_state_ = initial_state;
  tracking_confidence_ = 1.0;
  last_update_time_ = rclcpp::Clock().now();
  physics_model_ = physics_model;
  
  initializeMatrices();
}

auto BallTracker::initializeMatrices() -> void
{
  covariance_ = Eigen::Matrix<double, 6, 6>::Identity() * 0.1;
  
  process_noise_ = Eigen::Matrix<double, 6, 6>::Zero();
  process_noise_.diagonal() << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1;
  
  measurement_noise_ = Eigen::Matrix<double, 3, 3>::Identity() * 0.001;
}


auto BallTracker::getMeasurementMatrix() const -> Eigen::Matrix<double, 3, 6>
{
  Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  return H;
}

auto BallTracker::predict(double dt) -> void
{
  if (dt <= 0.0) return;
  
  auto F = physics_model_->getStateTransitionMatrix(ball_state_, dt);
  auto control_input = physics_model_->getControlInput(ball_state_, dt);
  
  state_ = F * state_ + control_input;
  
  // 地面との接触チェック（飛行→転がり遷移）
  if (ball_state_ == Ball::State::FLYING && state_(2) <= 0.0) {
    state_(2) = 0.0; // Z位置を地面に設定
    state_(5) = 0.0; // Z速度を0に設定
    ball_state_ = Ball::State::ROLLING;
  }
  
  // 状態遷移の更新
  updateStateTransition();
  
  covariance_ = F * covariance_ * F.transpose() + process_noise_ * dt;
  
  tracking_confidence_ = std::max(0.0, tracking_confidence_ - 0.01 * dt);
}

auto BallTracker::update(const Eigen::Vector3d & measurement, Ball::State observed_state) -> void
{
  auto H = getMeasurementMatrix();
  
  Eigen::Vector3d innovation = measurement - H * state_;
  
  Eigen::Matrix3d S = H * covariance_ * H.transpose() + measurement_noise_;
  
  Eigen::Matrix<double, 6, 3> K = covariance_ * H.transpose() * S.inverse();
  
  state_ = state_ + K * innovation;
  
  // 状態遷移の処理
  Ball::State estimated_state = physics_model_->estimateStateFromMeasurement(measurement, getVelocity());
  
  // 観測された状態と推定状態を組み合わせて最終状態を決定
  if (observed_state != Ball::State::STOPPED) {
    ball_state_ = observed_state;
  } else {
    ball_state_ = estimated_state;
  }
  
  // 状態に応じた制約の適用
  switch (ball_state_) {
    case Ball::State::STOPPED:
      state_.tail<3>().setZero(); // 速度を0に設定
      break;
    case Ball::State::ROLLING:
      state_(2) = 0.0; // Z位置を地面に設定
      state_(5) = 0.0; // Z速度を0に設定
      break;
    case Ball::State::FLYING:
      // 飛行中は制約なし
      break;
  }
  
  Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
  covariance_ = (I - K * H) * covariance_;
  
  tracking_confidence_ = std::min(1.0, tracking_confidence_ + 0.1);
  
  last_update_time_ = rclcpp::Clock().now();
}

auto BallTracker::getMahalanobisDistance(const Eigen::Vector3d & measurement) const -> double
{
  auto H = getMeasurementMatrix();
  Eigen::Vector3d innovation = measurement - H * state_;
  Eigen::Matrix3d S = H * covariance_ * H.transpose() + measurement_noise_;
  
  return std::sqrt(innovation.transpose() * S.inverse() * innovation);
}

auto BallTracker::isValidMeasurement(const Eigen::Vector3d & measurement, double threshold) const -> bool
{
  return getMahalanobisDistance(measurement) < threshold;
}

auto BallTracker::getPosition() const -> Eigen::Vector3d
{
  return state_.head<3>();
}

auto BallTracker::getVelocity() const -> Eigen::Vector3d
{
  return state_.tail<3>();
}

auto BallTracker::getCovariance() const -> Eigen::Matrix<double, 6, 6>
{
  return covariance_;
}

auto BallTracker::updateStateTransition() -> void
{
  auto current_position = getPosition();
  auto current_velocity = getVelocity();
  
  ball_state_ = physics_model_->checkStateTransition(ball_state_, current_position, current_velocity);
}

auto BallTracker::getBall() const -> Ball
{
  Ball ball;
  
  auto position = getPosition();
  auto velocity = getVelocity();
  
  ball.pos << position(0), position(1);
  ball.pos_z = position(2);
  ball.vel << velocity(0), velocity(1);
  ball.vel_z = velocity(2);
  
  ball.state = ball_state_;
  ball.detected = true;
  
  // 共有物理モデルを設定
  ball.setPhysicsModel(physics_model_);
  
  // 後方互換性のため物理パラメータも設定
  const auto& config = physics_model_->getConfig();
  ball.deceleration = config.deceleration;
  ball.gravity = config.gravity;
  ball.air_resistance = config.air_resistance;
  
  return ball;
}

auto BallTracker::getState() const -> crane_msgs::msg::BallInfo
{
  crane_msgs::msg::BallInfo ball_info;
  
  Ball ball = getBall();
  ball.toMsg(ball_info);
  
  return ball_info;
}

auto BallTracker::resetTracker(const Eigen::Vector3d & position, Ball::State state) -> void
{
  state_ = Eigen::Matrix<double, 6, 1>::Zero();
  state_.head<3>() = position;
  ball_state_ = state;
  tracking_confidence_ = 1.0;
  initializeMatrices();
}

BallTrackerManager::BallTrackerManager(std::shared_ptr<BallPhysicsModel> physics_model)
: physics_model_(physics_model)
{
}

auto BallTrackerManager::processVisionDetection(const Eigen::Vector3d & ball_position, const rclcpp::Time & timestamp) -> crane_msgs::msg::BallInfo
{
  // BallPhysicsModelを使って状態推定
  Eigen::Vector3d dummy_velocity = Eigen::Vector3d::Zero();  // 速度情報がない場合
  Ball::State estimated_state = physics_model_->estimateStateFromMeasurement(ball_position, dummy_velocity);
  
  return processVisionDetectionWithState(ball_position, estimated_state, timestamp);
}

auto BallTrackerManager::processVisionDetectionWithState(const Eigen::Vector3d & ball_position, Ball::State observed_state, const rclcpp::Time & timestamp) -> crane_msgs::msg::BallInfo
{
  auto best_tracker = findBestMatchingTracker(ball_position);
  
  if (best_tracker) {
    double dt = (timestamp - best_tracker->getLastUpdateTime()).seconds();
    if (dt > 0.0) {
      best_tracker->predict(dt);
    }
    best_tracker->update(ball_position, observed_state);
    best_tracker->setLastUpdateTime(timestamp);
  } else {
    best_tracker = createNewTracker(ball_position, observed_state);
    best_tracker->setLastUpdateTime(timestamp);
  }
  
  updateTrackingConfidences();
  
  return best_tracker->getState();
}

auto BallTrackerManager::findBestMatchingTracker(const Eigen::Vector3d & measurement) -> std::shared_ptr<BallTracker>
{
  std::shared_ptr<BallTracker> best_tracker = nullptr;
  double best_distance = std::numeric_limits<double>::max();
  
  for (auto & tracker : trackers_) {
    if (tracker->isValidMeasurement(measurement, OUTLIER_THRESHOLD)) {
      double distance = tracker->getMahalanobisDistance(measurement);
      if (distance < best_distance) {
        best_distance = distance;
        best_tracker = tracker;
      }
    }
  }
  
  return best_tracker;
}

auto BallTrackerManager::createNewTracker(const Eigen::Vector3d & position) -> std::shared_ptr<BallTracker>
{
  Eigen::Vector3d dummy_velocity = Eigen::Vector3d::Zero();
  Ball::State initial_state = physics_model_->estimateStateFromMeasurement(position, dummy_velocity);
  return createNewTracker(position, initial_state);
}

auto BallTrackerManager::createNewTracker(const Eigen::Vector3d & position, Ball::State state) -> std::shared_ptr<BallTracker>
{
  auto new_tracker = std::make_shared<BallTracker>(position, state, physics_model_);
  trackers_.push_back(new_tracker);
  return new_tracker;
}

auto BallTrackerManager::predict(double dt) -> void
{
  for (auto & tracker : trackers_) {
    tracker->predict(dt);
  }
}

auto BallTrackerManager::getBestTracker() const -> std::shared_ptr<BallTracker>
{
  if (trackers_.empty()) {
    return nullptr;
  }
  
  auto best_tracker = *std::max_element(
    trackers_.begin(), trackers_.end(),
    [](const auto & a, const auto & b) {
      return a->getTrackingConfidence() < b->getTrackingConfidence();
    });
  
  if (best_tracker->getTrackingConfidence() < MIN_TRACKING_CONFIDENCE) {
    return nullptr;
  }
  
  return best_tracker;
}

auto BallTrackerManager::updateTrackingConfidences() -> void
{
  for (auto & tracker : trackers_) {
    double time_since_update = (rclcpp::Clock().now() - tracker->getLastUpdateTime()).seconds();
    if (time_since_update > 0.1) {
      tracker->predict(time_since_update);
    }
  }
}

auto BallTrackerManager::removeOldTrackers(double max_age_seconds) -> void
{
  auto current_time = rclcpp::Clock().now();
  
  trackers_.erase(
    std::remove_if(trackers_.begin(), trackers_.end(),
      [current_time, max_age_seconds](const auto & tracker) {
        double age = (current_time - tracker->getLastUpdateTime()).seconds();
        return age > max_age_seconds || tracker->getTrackingConfidence() < MIN_TRACKING_CONFIDENCE;
      }),
    trackers_.end());
}
}  // namespace crane