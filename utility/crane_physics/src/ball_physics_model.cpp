// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_physics/ball_physics_model.hpp"

#include <cmath>

namespace crane
{

BallPhysicsModel::BallPhysicsModel() : config_() {}

BallPhysicsModel::BallPhysicsModel(const Config & config) : config_(config) {}

auto BallPhysicsModel::getStateTransitionMatrix(Ball::State state, double dt) const
  -> Eigen::Matrix<double, 6, 6>
{
  Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
  
  // 位置 = 位置 + 速度 * dt
  F.block<3, 3>(0, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dt;

  switch (state) {
    case Ball::State::ROLLING:
      // 転がり時は減速のみ
      break;
    case Ball::State::FLYING:
      // 飛行時は重力と空気抵抗
      break;
    case Ball::State::STOPPED:
    default:
      // 停止時は変化なし
      break;
  }

  return F;
}

auto BallPhysicsModel::getControlInput(Ball::State state, double dt) const
  -> Eigen::Matrix<double, 6, 1>
{
  Eigen::Matrix<double, 6, 1> u = Eigen::Matrix<double, 6, 1>::Zero();

  switch (state) {
    case Ball::State::FLYING:
      // 重力による加速度
      u(5) = config_.gravity * dt;  // Z方向に重力
      break;
    case Ball::State::ROLLING:
    case Ball::State::STOPPED:
    default:
      // 制御入力なし
      break;
  }

  return u;
}

auto BallPhysicsModel::estimateStateFromMeasurement(
  const Vector3 & position, const Vector3 & velocity) const -> Ball::State
{
  double speed = velocity.head<2>().norm();
  
  if (speed < config_.stop_threshold) {
    return Ball::State::STOPPED;
  } else if (position.z() > config_.height_threshold || std::abs(velocity.z()) > config_.speed_threshold) {
    return Ball::State::FLYING;
  } else {
    return Ball::State::ROLLING;
  }
}

auto BallPhysicsModel::checkStateTransition(
  Ball::State current_state, const Vector3 & position, const Vector3 & velocity) const
  -> Ball::State
{
  return estimateStateFromMeasurement(position, velocity);
}

auto BallPhysicsModel::predictPosition(
  const Point & position, const Point & velocity, Ball::State state, double pos_z, double vel_z,
  double time_ahead) const -> Point
{
  switch (state) {
    case Ball::State::ROLLING:
      return getRollingPredictedPosition(position, velocity, time_ahead);
    case Ball::State::FLYING:
      // 飛行時の放物運動
      return position + velocity * time_ahead;
    case Ball::State::STOPPED:
    default:
      return position;  // 停止時は位置変化なし
  }
}

auto BallPhysicsModel::predictVelocity(
  const Point & position, const Point & velocity, Ball::State state, double pos_z, double vel_z,
  double time_ahead) const -> Point
{
  switch (state) {
    case Ball::State::ROLLING:
      return getRollingPredictedVelocity(velocity, time_ahead);
    case Ball::State::FLYING:
      // 飛行時は水平成分は一定、垂直成分は重力による減速
      return velocity;  // 簡略化
    case Ball::State::STOPPED:
    default:
      return Point::Zero();  // 停止時は速度ゼロ
  }
}

auto BallPhysicsModel::getStopTime(const Point & velocity, Ball::State state, double vel_z) const
  -> double
{
  switch (state) {
    case Ball::State::ROLLING:
      return getRollingStopTime(velocity);
    case Ball::State::FLYING:
      // 着地時間 + 転がり停止時間の概算
      if (vel_z < 0) {
        return 0.5;  // 簡略化された着地時間
      }
      return 0.0;
    case Ball::State::STOPPED:
    default:
      return 0.0;
  }
}

auto BallPhysicsModel::getMaxDistance(
  const Point & position, const Point & velocity, Ball::State state, double pos_z,
  double vel_z) const -> double
{
  switch (state) {
    case Ball::State::ROLLING:
      return getRollingMaxDistance(velocity);
    case Ball::State::FLYING:
      // 飛行距離の概算
      return velocity.norm() * 1.0;  // 簡略化
    case Ball::State::STOPPED:
    default:
      return 0.0;
  }
}

auto BallPhysicsModel::getRollingStopTime(const Point & velocity) const -> double
{
  double speed = velocity.norm();
  if (speed < config_.stop_threshold) {
    return 0.0;
  }
  return speed / config_.deceleration;
}

auto BallPhysicsModel::getRollingMaxDistance(const Point & velocity) const -> double
{
  double speed = velocity.norm();
  if (speed < config_.stop_threshold) {
    return 0.0;
  }
  // v² = v₀² - 2ad → d = v₀²/(2a)
  return (speed * speed) / (2.0 * config_.deceleration);
}

auto BallPhysicsModel::getRollingPredictedPosition(
  const Point & position, const Point & velocity, double time_ahead) const -> Point
{
  double speed = velocity.norm();
  if (speed < config_.stop_threshold) {
    return position;
  }

  Point direction = velocity.normalized();
  double stop_time = getRollingStopTime(velocity);
  
  if (time_ahead >= stop_time) {
    // 停止後は移動なし
    double distance = getRollingMaxDistance(velocity);
    return position + direction * distance;
  } else {
    // 等減速運動: s = v₀t - (1/2)at²
    double distance = speed * time_ahead - 0.5 * config_.deceleration * time_ahead * time_ahead;
    return position + direction * distance;
  }
}

auto BallPhysicsModel::getRollingPredictedVelocity(const Point & velocity, double time_ahead) const
  -> Point
{
  double speed = velocity.norm();
  if (speed < config_.stop_threshold) {
    return Point::Zero();
  }

  Point direction = velocity.normalized();
  double stop_time = getRollingStopTime(velocity);
  
  if (time_ahead >= stop_time) {
    return Point::Zero();  // 停止後は速度ゼロ
  } else {
    // 等減速運動: v = v₀ - at
    double new_speed = speed - config_.deceleration * time_ahead;
    return direction * std::max(0.0, new_speed);
  }
}

// ファクトリー実装
std::shared_ptr<BallPhysicsModel> BallPhysicsModelFactory::instance_ = nullptr;

auto BallPhysicsModelFactory::getInstance() -> std::shared_ptr<BallPhysicsModel>
{
  if (!instance_) {
    instance_ = std::make_shared<BallPhysicsModel>();
  }
  return instance_;
}

auto BallPhysicsModelFactory::createWithConfig(const BallPhysicsModel::Config & config)
  -> std::shared_ptr<BallPhysicsModel>
{
  return std::make_shared<BallPhysicsModel>(config);
}

}  // namespace crane