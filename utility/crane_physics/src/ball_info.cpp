// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_msgs/msg/ball_info.hpp>
#include <crane_msgs/msg/ball_physics_config.hpp>
#include <crane_physics/ball_info.hpp>
#include <crane_physics/ball_physics_model.hpp>

namespace crane
{

// Ball クラスのコンストラクタ実装
Ball::Ball() : physics_model_(std::make_shared<BallPhysicsModel>(BallPhysicsModel::createDefault()))
{
}

Ball::Ball(std::shared_ptr<BallPhysicsModel> model) : physics_model_(model)
{
  if (!physics_model_) {
    physics_model_ = std::make_shared<BallPhysicsModel>(BallPhysicsModel::createDefault());
  }
}

// Ball クラスのメソッド実装
auto Ball::setPhysicsModel(std::shared_ptr<BallPhysicsModel> model) -> void
{
  physics_model_ =
    model ? model : std::make_shared<BallPhysicsModel>(BallPhysicsModel::createDefault());
}

auto Ball::getPhysicsModel() const -> std::shared_ptr<BallPhysicsModel> { return physics_model_; }

// Ball クラスのその他実装

auto Ball::getPredictedPosition(double time_ahead) const -> Point
{
  return physics_model_->predictPosition(pos, vel, state, pos_z, vel_z, time_ahead);
}

auto Ball::getPredictedVelocity(double time_ahead) const -> Point
{
  return physics_model_->predictVelocity(pos, vel, state, pos_z, vel_z, time_ahead);
}

auto Ball::getStopTime() const -> double { return physics_model_->getStopTime(vel, state, vel_z); }

auto Ball::getMaxDistance() const -> double
{
  return physics_model_->getMaxDistance(pos, vel, state, pos_z, vel_z);
}

auto Ball::getRollingStopTime() const -> double
{
  double speed = vel.norm();
  if (speed == 0) return 0;
  return speed / physics_model_->getDeceleration();
}

auto Ball::getRollingMaxDistance() const -> double
{
  double speed = vel.norm();
  if (speed == 0) return 0;
  double decel = physics_model_->getDeceleration();
  double stop_time = getRollingStopTime();
  return speed * stop_time - 0.5 * decel * stop_time * stop_time;
}

auto Ball::getRollingMaxDistanceFromVelocity(const Point & velocity) const -> double
{
  double speed = velocity.norm();
  if (speed == 0) return 0;
  double decel = physics_model_->getDeceleration();
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
  double decel = physics_model_->getDeceleration();

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
    double decel = physics_model_->getDeceleration();
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

  double decel = physics_model_->getDeceleration();
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

// fromMsgメソッドでBallPhysicsModelを設定
void Ball::updatePhysicsConfigFromMsg(const crane_msgs::msg::BallPhysicsConfig & physics_config)
{
  BallPhysicsModel::Config config;
  config.deceleration = physics_config.deceleration;
  config.gravity = physics_config.gravity;
  config.air_resistance = physics_config.air_resistance;
  config.height_threshold = physics_config.height_threshold;
  config.speed_threshold = physics_config.speed_threshold;
  config.stop_threshold = physics_config.stop_threshold;

  physics_model_->setConfig(config);
}

// テンプレート関数の実装
template <typename BallInfoMsg>
void Ball::toMsg(BallInfoMsg & msg) const
{
  // 位置・速度
  msg.position.x = pos.x();
  msg.position.y = pos.y();
  msg.position.z = pos_z;
  msg.velocity.x = vel.x();
  msg.velocity.y = vel.y();
  msg.velocity.z = vel_z;
  msg.velocity_norm = vel.norm();

  // 検出状態
  msg.detected = detected;

  // ボール状態
  switch (state) {
    case State::STOPPED:
      msg.state = BallInfoMsg::STOPPED;  // STOPPED
      break;
    case State::ROLLING:
      msg.state = BallInfoMsg::ROLLING;  // ROLLING
      break;
    case State::FLYING:
      msg.state = BallInfoMsg::FLYING;  // FLYING
      break;
  }

  // BallPhysicsModel設定
  const auto & config = physics_model_->getConfig();
  msg.physics_config.deceleration = config.deceleration;
  msg.physics_config.gravity = config.gravity;
  msg.physics_config.air_resistance = config.air_resistance;
  msg.physics_config.height_threshold = config.height_threshold;
  msg.physics_config.speed_threshold = config.speed_threshold;
  msg.physics_config.stop_threshold = config.stop_threshold;
}

template <typename BallInfoMsg>
void Ball::fromMsg(const BallInfoMsg & msg)
{
  // 位置・速度
  pos << msg.position.x, msg.position.y;
  pos_z = msg.position.z;
  vel << msg.velocity.x, msg.velocity.y;
  vel_z = msg.velocity.z;

  // 検出状態
  detected = msg.detected;

  // ボール状態
  switch (msg.state) {
    case BallInfoMsg::STOPPED:  // STOPPED
      state = State::STOPPED;
      break;
    case BallInfoMsg::ROLLING:  // ROLLING
      state = State::ROLLING;
      break;
    case BallInfoMsg::FLYING:  // FLYING
      state = State::FLYING;
      break;
    default:
      state = State::STOPPED;  // デフォルトは停止
      break;
  }

  // BallPhysicsModel設定から物理モデルを設定
  updatePhysicsConfigFromMsg(msg.physics_config);
}

// 明示的なテンプレートインスタンス化
template void Ball::toMsg<crane_msgs::msg::BallInfo>(crane_msgs::msg::BallInfo & msg) const;
template void Ball::fromMsg<crane_msgs::msg::BallInfo>(const crane_msgs::msg::BallInfo & msg);

}  // namespace crane
