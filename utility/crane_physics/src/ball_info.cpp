// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_physics/ball_info.hpp>
#include <crane_physics/ball_physics_model.hpp>
#include <crane_msgs/msg/ball_physics_config.hpp>

namespace crane
{

// Ball クラスの実装

auto Ball::getPredictedPosition(double time_ahead) const -> Point
{
  if (!physics_model_) {
    // フォールバック：物理モデルがない場合は現在位置を返す
    return pos;
  }
  return physics_model_->predictPosition(pos, vel, state, pos_z, vel_z, time_ahead);
}

auto Ball::getPredictedVelocity(double time_ahead) const -> Point
{
  if (!physics_model_) {
    return {0, 0};
  }
  return physics_model_->predictVelocity(pos, vel, state, pos_z, vel_z, time_ahead);
}

auto Ball::getStopTime() const -> double
{
  if (!physics_model_) {
    return 0.0;
  }
  return physics_model_->getStopTime(vel, state, vel_z);
}

auto Ball::getMaxDistance() const -> double
{
  if (!physics_model_) {
    return 0.0;
  }
  return physics_model_->getMaxDistance(pos, vel, state, pos_z, vel_z);
}

auto Ball::getRollingStopTime() const -> double
{
  double speed = vel.norm();
  if (speed == 0) return 0;
  if (!physics_model_) {
    return speed / 0.5;  // デフォルト減速度を使用
  }
  return speed / physics_model_->getDeceleration();
}

auto Ball::getRollingMaxDistance() const -> double
{
  double speed = vel.norm();
  if (speed == 0) return 0;
  double decel = physics_model_ ? physics_model_->getDeceleration() : 0.5;
  double stop_time = getRollingStopTime();
  return speed * stop_time - 0.5 * decel * stop_time * stop_time;
}

auto Ball::getRollingMaxDistanceFromVelocity(const Point & velocity) const -> double
{
  double speed = velocity.norm();
  if (speed == 0) return 0;
  double decel = physics_model_ ? physics_model_->getDeceleration() : 0.5;
  double stop_time = speed / decel;
  return speed * stop_time - 0.5 * decel * stop_time * stop_time;
}

auto Ball::getRollingPredictedPosition(double time_ahead) const -> Point
{
  double speed = vel.norm();
  if (speed == 0) {
    return pos;
  }

  Point direction = vel.normalized();
  double stop_time = getRollingStopTime();
  double decel = physics_model_ ? physics_model_->getDeceleration() : 0.5;

  if (time_ahead >= stop_time) {
    double max_distance = getRollingMaxDistance();
    return pos + direction * max_distance;
  } else {
    double distance_traveled = speed * time_ahead - 0.5 * decel * time_ahead * time_ahead;
    return pos + direction * distance_traveled;
  }
}

auto Ball::getRollingPredictedVelocity(double time_ahead) const -> Point
{
  double speed = vel.norm();
  if (speed == 0) {
    return {0, 0};
  }

  Point direction = vel.normalized();
  double stop_time = getRollingStopTime();

  if (time_ahead >= stop_time) {
    return {0, 0};
  } else {
    double decel = physics_model_ ? physics_model_->getDeceleration() : 0.5;
    double current_speed = speed - decel * time_ahead;
    return direction * current_speed;
  }
}

auto Ball::getRollingTimeToReachDistance(double distance) const -> std::optional<double>
{
  double speed = vel.norm();
  if (speed <= 0) {
    return distance == 0 ? std::make_optional(0.0) : std::nullopt;
  }

  double decel = physics_model_ ? physics_model_->getDeceleration() : 0.5;
  double stop_time = speed / decel;
  double max_distance = speed * stop_time - 0.5 * decel * stop_time * stop_time;

  if (distance > max_distance) {
    return std::nullopt;
  }

  double a = -0.5 * decel;
  double b = speed;
  double c = -distance;

  double discriminant = b * b - 4 * a * c;
  if (discriminant < 0) {
    return std::nullopt;
  }

  double sqrt_discriminant = sqrt(discriminant);
  double t1 = (-b + sqrt_discriminant) / (2 * a);
  double t2 = (-b - sqrt_discriminant) / (2 * a);

  if (t1 > 0 && t1 <= stop_time) {
    return t1;
  } else if (t2 > 0 && t2 <= stop_time) {
    return t2;
  }

  return std::nullopt;
}

// fromMsgメソッドでBallPhysicsModelを作成
void Ball::createPhysicsModelFromMsg(const crane_msgs::msg::BallPhysicsConfig & physics_config)
{
  BallPhysicsModel::Config config;
  config.deceleration = physics_config.deceleration;
  config.gravity = physics_config.gravity;
  config.air_resistance = physics_config.air_resistance;
  config.height_threshold = physics_config.height_threshold;
  config.speed_threshold = physics_config.speed_threshold;
  config.stop_threshold = physics_config.stop_threshold;
  
  physics_model_ = std::make_shared<BallPhysicsModel>(config);
}

}  // namespace crane