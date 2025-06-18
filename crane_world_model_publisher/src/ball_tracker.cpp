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
BallTracker::BallTracker(const Eigen::Vector3d & initial_position, Ball::State initial_state)
{
  state_ = Eigen::Matrix<double, 6, 1>::Zero();
  state_.head<3>() = initial_position;
  
  ball_state_ = initial_state;
  tracking_confidence_ = 1.0;
  last_update_time_ = rclcpp::Clock().now();
  
  initializeMatrices();
}

auto BallTracker::initializeMatrices() -> void
{
  covariance_ = Eigen::Matrix<double, 6, 6>::Identity() * 0.1;
  
  process_noise_ = Eigen::Matrix<double, 6, 6>::Zero();
  process_noise_.diagonal() << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1;
  
  measurement_noise_ = Eigen::Matrix<double, 3, 3>::Identity() * 0.001;
}

auto BallTracker::getStateTransitionMatrix(double dt) const -> Eigen::Matrix<double, 6, 6>
{
  Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
  
  F(0, 3) = dt;
  F(1, 4) = dt;
  F(2, 5) = dt;
  
  switch (ball_state_) {
    case Ball::State::STOPPED:
      // 停止状態では速度は0のまま
      break;
      
    case Ball::State::ROLLING:
      // 転がっているときは減速
      {
        double deceleration_factor = std::exp(-BALL_DECELERATION * dt);
        F(3, 3) = deceleration_factor;
        F(4, 4) = deceleration_factor;
        // Z方向の速度は0
        F(5, 5) = 0.0;
      }
      break;
      
    case Ball::State::FLYING:
      // 飛んでいるときはXY速度は一定、Z速度は重力の影響
      // Z方向の速度変化 (vz = vz0 + g*t)
      F(5, 5) = 1.0; // 速度は連続
      break;
  }
  
  return F;
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
  
  auto F = getStateTransitionMatrix(dt);
  
  Eigen::Matrix<double, 6, 1> control_input = Eigen::Matrix<double, 6, 1>::Zero();
  
  switch (ball_state_) {
    case Ball::State::STOPPED:
      // 停止状態では制御入力なし
      break;
      
    case Ball::State::ROLLING:
      // 転がっているときは重力のZ成分のみ（地面に接触）
      control_input(2) = 0.0; // Z位置は変化しない
      control_input(5) = 0.0; // Z速度も0
      break;
      
    case Ball::State::FLYING:
      // 飛んでいるときは重力の影響
      {
        Eigen::Vector3d gravity_effect = Eigen::Vector3d::Zero();
        gravity_effect(2) = 0.5 * GRAVITY * dt * dt;
        control_input.head<3>() = gravity_effect;
        control_input(5) = GRAVITY * dt;
      }
      break;
  }
  
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
  Ball::State estimated_state = estimateStateFromMeasurement(measurement, getVelocity());
  
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

auto BallTracker::estimateStateFromMeasurement(const Eigen::Vector3d & position, const Eigen::Vector3d & velocity) const -> Ball::State
{
  double height = position(2);
  double speed = velocity.head<2>().norm();
  
  if (height > 0.05) {
    return Ball::State::FLYING;
  } else if (speed > 0.1) {
    return Ball::State::ROLLING;
  } else {
    return Ball::State::STOPPED;
  }
}

auto BallTracker::updateStateTransition() -> void
{
  auto current_position = getPosition();
  auto current_velocity = getVelocity();
  
  // 自動状態遷移のロジック
  switch (ball_state_) {
    case Ball::State::FLYING:
      if (current_position(2) <= 0.0) {
        ball_state_ = Ball::State::ROLLING;
      }
      break;
    case Ball::State::ROLLING:
      if (current_velocity.head<2>().norm() < 0.05) {
        ball_state_ = Ball::State::STOPPED;
      }
      break;
    case Ball::State::STOPPED:
      if (current_velocity.head<2>().norm() > 0.1) {
        ball_state_ = Ball::State::ROLLING;
      }
      break;
  }
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
  
  ball.deceleration = BALL_DECELERATION;
  ball.gravity = GRAVITY;
  ball.air_resistance = AIR_RESISTANCE;
  
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

BallTrackerManager::BallTrackerManager()
{
}

auto BallTrackerManager::processVisionDetection(const Eigen::Vector3d & ball_position, const rclcpp::Time & timestamp) -> crane_msgs::msg::BallInfo
{
  // デフォルトでROLLING状態と仮定
  Ball::State estimated_state = Ball::State::ROLLING;
  if (ball_position(2) > 0.05) {
    estimated_state = Ball::State::FLYING;
  }
  
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
  Ball::State initial_state = Ball::State::ROLLING;
  if (position(2) > 0.05) {
    initial_state = Ball::State::FLYING;
  }
  return createNewTracker(position, initial_state);
}

auto BallTrackerManager::createNewTracker(const Eigen::Vector3d & position, Ball::State state) -> std::shared_ptr<BallTracker>
{
  auto new_tracker = std::make_shared<BallTracker>(position, state);
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