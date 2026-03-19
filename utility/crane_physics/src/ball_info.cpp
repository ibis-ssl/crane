// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_msgs/msg/ball_info.hpp>
#include <crane_msgs/msg/ball_physics_config.hpp>
#include <crane_physics/ball_info.hpp>
#include <crane_physics/ball_physics_model.hpp>
#include <rclcpp/rclcpp.hpp>

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
  // deceleration <= 0 はメッセージが未設定（ゼロ初期化）と判断し、現在の設定を維持する
  // deceleration=0 のまま適用すると stop_time=inf → NaN が伝播する
  if (physics_config.deceleration <= 0.0) {
    RCLCPP_WARN_ONCE(
      rclcpp::get_logger("Ball"),
      "BallPhysicsConfig.deceleration <= 0 (%.3f): message is uninitialized, keeping current "
      "config (deceleration=%.3f)",
      physics_config.deceleration, physics_model_->getDeceleration());
    return;
  }

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

// ParabolicPhysics クラスの実装
Ball::ParabolicPhysics::ParabolicPhysics(const Ball & ball)
: initial_position_(Point3D(ball.pos.x(), ball.pos.y(), ball.pos_z)),
  initial_velocity_(Point3D(ball.vel.x(), ball.vel.y(), ball.vel_z)),
  gravity_(-9.81)  // デフォルト重力値を使用
{
}

Ball::ParabolicPhysics::ParabolicPhysics(
  Point3D initial_position, Point3D initial_velocity, double gravity)
: initial_position_(initial_position), initial_velocity_(initial_velocity), gravity_(gravity)
{
}

auto Ball::ParabolicPhysics::getPredictedPosition3D(double time_ahead) const -> Point3D
{
  Point3D position;
  position.x() = initial_position_.x() + initial_velocity_.x() * time_ahead;
  position.y() = initial_position_.y() + initial_velocity_.y() * time_ahead;
  position.z() = initial_position_.z() + initial_velocity_.z() * time_ahead +
                 0.5 * gravity_ * time_ahead * time_ahead;

  return position;
}

auto Ball::ParabolicPhysics::getPredictedVelocity2D(double /*time_ahead*/) const -> Point
{
  // XとY速度は一定（空気抵抗なし）
  // Z速度は重力により変化するが、2D速度のみを返す
  return {initial_velocity_.x(), initial_velocity_.y()};
}

auto Ball::ParabolicPhysics::getGroundIntersection() const -> std::pair<Point, double>
{
  double z0 = initial_position_.z();
  double vz0 = initial_velocity_.z();

  if (std::abs(z0) < 1e-6) {
    return {Point(initial_position_.x(), initial_position_.y()), 0.0};
  }

  double a = 0.5 * gravity_;
  double b = vz0;
  double c = z0;

  double discriminant = b * b - 4 * a * c;

  if (discriminant < 0) {
    double t_peak = -vz0 / gravity_;
    if (t_peak < 0) t_peak = 0;

    Point peak_xy(
      initial_position_.x() + initial_velocity_.x() * t_peak,
      initial_position_.y() + initial_velocity_.y() * t_peak);
    return {peak_xy, t_peak};
  }

  double sqrt_discriminant = std::sqrt(discriminant);
  double t1 = (-b + sqrt_discriminant) / (2 * a);
  double t2 = (-b - sqrt_discriminant) / (2 * a);

  double landing_time;
  if (t1 > 1e-6 && t2 > 1e-6) {
    landing_time = std::min(t1, t2);
  } else if (t1 > 1e-6) {
    landing_time = t1;
  } else if (t2 > 1e-6) {
    landing_time = t2;
  } else {
    return {Point(initial_position_.x(), initial_position_.y()), 0.0};
  }

  Point landing_position(
    initial_position_.x() + initial_velocity_.x() * landing_time,
    initial_position_.y() + initial_velocity_.y() * landing_time);

  return {landing_position, landing_time};
}

void Ball::ParabolicPhysics::estimateInitialVelocityFromPointLog()
{
  if (point_log.size() < 2) {
    // データが不十分な場合は推定できない
    initial_velocity_ = Point3D::Zero();
    // 1つのデータポイントがある場合は初期位置を更新
    if (point_log.size() == 1) {
      initial_position_ = point_log[0].position;
    }
    return;
  }

  // 時系列でソート
  ranges::sort(
    point_log, [](const Point3DStamped & a, const Point3DStamped & b) { return a.time < b.time; });

  // 最小時刻を基準時刻とする
  double t0 = point_log[0].time;
  Point3D p0 = point_log[0].position;

  // 初期位置を更新（全てのケースで確実に実行）
  initial_position_ = p0;

  // 最小二乗法で初期速度を推定
  // 放物運動の方程式: p(t) = p0 + v0*t + 0.5*g*t^2
  // ここでg = (0, 0, -9.81) (重力加速度)

  // 連立方程式を解くためのマトリックス設定
  // A * v0 = b の形で解く
  // 各データポイントについて: p_i - p0 - 0.5*g*(t_i-t0)^2*[0,0,1] = v0*(t_i-t0)

  size_t n = point_log.size();
  if (n < 3) {
    // 3点未満の場合は線形近似
    Point3D p1 = point_log[1].position;
    double t1 = point_log[1].time;
    double dt = t1 - t0;

    if (dt > 1e-6) {
      initial_velocity_ = (p1 - p0) / dt;
      // Z方向の重力補正
      initial_velocity_.z() += 0.5 * (-gravity_) * dt;
    } else {
      initial_velocity_ = Point3D::Zero();
    }
    return;
  }

  // 最小二乗法による推定
  double sum_t = 0, sum_t2 = 0;
  Point3D sum_dp = Point3D::Zero();
  Point3D sum_t_dp = Point3D::Zero();

  for (size_t i = 1; i < n; ++i) {
    double dt = point_log[i].time - t0;
    Point3D dp = point_log[i].position - p0;

    // Z方向の重力補正
    dp.z() -= 0.5 * gravity_ * dt * dt;

    sum_t += dt;
    sum_t2 += dt * dt;
    sum_dp = sum_dp + dp;
    sum_t_dp = sum_t_dp + dp * dt;
  }

  double n_points = static_cast<double>(n - 1);
  double denominator = n_points * sum_t2 - sum_t * sum_t;

  if (std::abs(denominator) < 1e-10) {
    // 数値的に不安定な場合は最初の2点を使用
    Point3D p1 = point_log[1].position;
    double t1 = point_log[1].time;
    double dt = t1 - t0;

    if (dt > 1e-6) {
      initial_velocity_ = (p1 - p0) / dt;
      initial_velocity_.z() += 0.5 * (-gravity_) * dt;
    } else {
      initial_velocity_ = Point3D::Zero();
    }
    return;
  }

  // 最小二乗解を計算
  Point3D numerator = sum_t_dp * n_points - sum_dp * sum_t;
  initial_velocity_ = numerator / denominator;
}

Ball::ParabolicPhysics::Point3DStamped Ball::ParabolicPhysics::getGroundPoint()
{
  // 放物運動の方程式: z(t) = z0 + vz0*t + 0.5*g*t^2
  // 着地条件: z(t) = 0
  // 0 = z0 + vz0*t - 4.905*t^2 (g = -9.81なので0.5*g = -4.905)

  double z0 = initial_position_.z();
  double vz0 = initial_velocity_.z();

  // 既に地面にいる場合
  if (std::abs(z0) < 1e-6) {
    return {initial_position_, 0.0};
  }

  // 二次方程式: -4.905*t^2 + vz0*t + z0 = 0
  // at^2 + bt + c = 0の形に変換
  double a = 0.5 * gravity_;  // -4.905
  double b = vz0;
  double c = z0;

  double discriminant = b * b - 4 * a * c;

  // 解が存在しない場合（地面に到達しない）
  if (discriminant < 0) {
    // 最高点での位置を返す（近似的な着地点として）
    double t_peak = -vz0 / gravity_;
    if (t_peak < 0) t_peak = 0;  // 負の時間は物理的に意味がない

    Point3D peak_position;
    peak_position.x() = initial_position_.x() + initial_velocity_.x() * t_peak;
    peak_position.y() = initial_position_.y() + initial_velocity_.y() * t_peak;
    peak_position.z() =
      initial_position_.z() + initial_velocity_.z() * t_peak + 0.5 * gravity_ * t_peak * t_peak;

    return {peak_position, t_peak};
  }

  double sqrt_discriminant = std::sqrt(discriminant);
  double t1 = (-b + sqrt_discriminant) / (2 * a);
  double t2 = (-b - sqrt_discriminant) / (2 * a);

  // 正の時間で最初に地面に到達する時間を選択
  double landing_time;
  if (t1 > 1e-6 && t2 > 1e-6) {
    landing_time = std::min(t1, t2);
  } else if (t1 > 1e-6) {
    landing_time = t1;
  } else if (t2 > 1e-6) {
    landing_time = t2;
  } else {
    // 両方とも負または零の場合、既に地面より下にいる
    return {initial_position_, 0.0};
  }

  // 着地位置を計算
  Point3D landing_position;
  landing_position.x() = initial_position_.x() + initial_velocity_.x() * landing_time;
  landing_position.y() = initial_position_.y() + initial_velocity_.y() * landing_time;
  landing_position.z() = 0.0;  // 地面なのでz=0

  return {landing_position, landing_time};
}

}  // namespace crane
