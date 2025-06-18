// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/ball_physics_model.hpp"

#include <algorithm>
#include <cmath>

namespace crane
{
BallPhysicsModel::BallPhysicsModel() : config_({})
{
}

BallPhysicsModel::BallPhysicsModel(const Config & config) : config_(config)
{
}

auto BallPhysicsModel::getStateTransitionMatrix(Ball::State state, double dt) const -> Eigen::Matrix<double, 6, 6>
{
  Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
  
  // 位置更新 (x = x + v*dt)
  F(0, 3) = dt;
  F(1, 4) = dt;
  F(2, 5) = dt;
  
  switch (state) {
    case Ball::State::STOPPED:
      // 停止状態では速度は0のまま（デフォルトの単位行列）
      break;
      
    case Ball::State::ROLLING:
      // 転がっているときは減速
      {
        double deceleration_factor = std::exp(-config_.deceleration * dt);
        F(3, 3) = deceleration_factor;  // vx減速
        F(4, 4) = deceleration_factor;  // vy減速
        F(5, 5) = 0.0;                  // vzは0
      }
      break;
      
    case Ball::State::FLYING:
      // 飛んでいるときはXY速度は一定、Z速度は重力の影響
      F(5, 5) = 1.0;  // Z速度は連続（重力は制御入力で処理）
      break;
  }
  
  return F;
}

auto BallPhysicsModel::getControlInput(Ball::State state, double dt) const -> Eigen::Matrix<double, 6, 1>
{
  Eigen::Matrix<double, 6, 1> control_input = Eigen::Matrix<double, 6, 1>::Zero();
  
  switch (state) {
    case Ball::State::STOPPED:
    case Ball::State::ROLLING:
      // 停止・転がり状態では制御入力なし（地面に接触）
      break;
      
    case Ball::State::FLYING:
      // 飛行状態では重力の影響
      {
        double gravity_pos_effect = 0.5 * config_.gravity * dt * dt;
        double gravity_vel_effect = config_.gravity * dt;
        
        control_input(2) = gravity_pos_effect;  // Z位置への重力効果
        control_input(5) = gravity_vel_effect;  // Z速度への重力効果
      }
      break;
  }
  
  return control_input;
}

auto BallPhysicsModel::estimateStateFromMeasurement(const Eigen::Vector3d & position, const Eigen::Vector3d & velocity) const -> Ball::State
{
  double height = position(2);
  double speed = velocity.head<2>().norm();
  
  if (height > config_.height_threshold) {
    return Ball::State::FLYING;
  } else if (speed > config_.speed_threshold) {
    return Ball::State::ROLLING;
  } else {
    return Ball::State::STOPPED;
  }
}

auto BallPhysicsModel::checkStateTransition(Ball::State current_state, const Eigen::Vector3d & position, const Eigen::Vector3d & velocity) const -> Ball::State
{
  double height = position(2);
  double speed = velocity.head<2>().norm();
  
  switch (current_state) {
    case Ball::State::FLYING:
      if (height <= 0.0) {
        return Ball::State::ROLLING;
      }
      break;
      
    case Ball::State::ROLLING:
      if (speed < config_.stop_threshold) {
        return Ball::State::STOPPED;
      }
      break;
      
    case Ball::State::STOPPED:
      if (speed > config_.speed_threshold) {
        return Ball::State::ROLLING;
      }
      break;
  }
  
  return current_state;  // 状態変化なし
}

auto BallPhysicsModel::predictPosition(const Ball & ball, double time_ahead) const -> Point
{
  switch (ball.state) {
    case Ball::State::STOPPED:
      return ball.pos;
      
    case Ball::State::ROLLING:
      return getRollingPredictedPosition(ball.pos, ball.vel, time_ahead);
      
    case Ball::State::FLYING: {
      auto parabolic = ball.getParabolicPhysics();
      auto [landing_pos, landing_time] = parabolic.getGroundIntersection();
      
      if (time_ahead <= landing_time) {
        // まだ空中
        Point3D pos_3d = parabolic.getPredictedPosition3D(time_ahead);
        return {pos_3d.x(), pos_3d.y()};
      } else {
        // 着地後転がり
        double time_after_landing = time_ahead - landing_time;
        Point landing_vel = parabolic.getPredictedVelocity2D(landing_time);
        return getRollingPredictedPosition(landing_pos, landing_vel, time_after_landing);
      }
    }
  }
  return ball.pos;
}

auto BallPhysicsModel::predictVelocity(const Ball & ball, double time_ahead) const -> Point
{
  switch (ball.state) {
    case Ball::State::STOPPED:
      return {0, 0};
      
    case Ball::State::ROLLING:
      return getRollingPredictedVelocity(ball.vel, time_ahead);
      
    case Ball::State::FLYING: {
      auto parabolic = ball.getParabolicPhysics();
      auto [landing_pos, landing_time] = parabolic.getGroundIntersection();
      
      if (time_ahead <= landing_time) {
        // まだ空中
        return parabolic.getPredictedVelocity2D(time_ahead);
      } else {
        // 着地後転がり
        double time_after_landing = time_ahead - landing_time;
        Point landing_vel = parabolic.getPredictedVelocity2D(landing_time);
        return getRollingPredictedVelocity(landing_vel, time_after_landing);
      }
    }
  }
  return {0, 0};
}

auto BallPhysicsModel::getStopTime(const Ball & ball) const -> double
{
  switch (ball.state) {
    case Ball::State::STOPPED:
      return 0.0;
      
    case Ball::State::ROLLING:
      return getRollingStopTime(ball.vel);
      
    case Ball::State::FLYING: {
      auto parabolic = ball.getParabolicPhysics();
      auto [landing_pos, landing_time] = parabolic.getGroundIntersection();
      Point landing_vel = parabolic.getPredictedVelocity2D(landing_time);
      double rolling_stop_time = getRollingStopTime(landing_vel);
      return landing_time + rolling_stop_time;
    }
  }
  return 0.0;
}

auto BallPhysicsModel::getMaxDistance(const Ball & ball) const -> double
{
  switch (ball.state) {
    case Ball::State::STOPPED:
      return 0.0;
      
    case Ball::State::ROLLING:
      return getRollingMaxDistance(ball.vel);
      
    case Ball::State::FLYING: {
      auto parabolic = ball.getParabolicPhysics();
      auto [landing_pos, landing_time] = parabolic.getGroundIntersection();
      Point landing_vel = parabolic.getPredictedVelocity2D(landing_time);
      
      double distance_to_landing = (landing_pos - ball.pos).norm();
      double rolling_distance = getRollingMaxDistance(landing_vel);
      
      return distance_to_landing + rolling_distance;
    }
  }
  return 0.0;
}

auto BallPhysicsModel::getRollingStopTime(const Point & velocity) const -> double
{
  double speed = velocity.norm();
  if (speed == 0) return 0;
  return speed / config_.deceleration;
}

auto BallPhysicsModel::getRollingMaxDistance(const Point & velocity) const -> double
{
  double speed = velocity.norm();
  if (speed == 0) return 0;
  double stop_time = getRollingStopTime(velocity);
  return speed * stop_time - 0.5 * config_.deceleration * stop_time * stop_time;
}

auto BallPhysicsModel::getRollingPredictedPosition(const Point & position, const Point & velocity, double time_ahead) const -> Point
{
  double speed = velocity.norm();
  if (speed == 0) {
    return position;
  }
  
  Point direction = velocity.normalized();
  double stop_time = getRollingStopTime(velocity);
  
  if (time_ahead >= stop_time) {
    double max_distance = getRollingMaxDistance(velocity);
    return position + direction * max_distance;
  } else {
    double distance_traveled = speed * time_ahead - 0.5 * config_.deceleration * time_ahead * time_ahead;
    return position + direction * distance_traveled;
  }
}

auto BallPhysicsModel::getRollingPredictedVelocity(const Point & velocity, double time_ahead) const -> Point
{
  double speed = velocity.norm();
  if (speed == 0) {
    return {0, 0};
  }
  
  Point direction = velocity.normalized();
  double stop_time = getRollingStopTime(velocity);
  
  if (time_ahead >= stop_time) {
    return {0, 0};
  } else {
    double current_speed = speed - config_.deceleration * time_ahead;
    return direction * current_speed;
  }
}

// ファクトリーの実装
std::shared_ptr<BallPhysicsModel> BallPhysicsModelFactory::instance_ = nullptr;

auto BallPhysicsModelFactory::getInstance() -> std::shared_ptr<BallPhysicsModel>
{
  if (!instance_) {
    instance_ = std::make_shared<BallPhysicsModel>();
  }
  return instance_;
}

auto BallPhysicsModelFactory::createWithConfig(const BallPhysicsModel::Config & config) -> std::shared_ptr<BallPhysicsModel>
{
  instance_ = std::make_shared<BallPhysicsModel>(config);
  return instance_;
}
}  // namespace crane