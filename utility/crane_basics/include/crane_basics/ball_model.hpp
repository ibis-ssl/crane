// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__BALL_MODEL_HPP_
#define CRANE_BASICS__BALL_MODEL_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <optional>
#include <range/v3/all.hpp>
#include <utility>
#include <vector>

namespace crane
{
class DecelerationBallPhysics
{
private:
  Point initial_position_;
  Point initial_velocity_;
  double deceleration_;
  double initial_speed_;  // 初期速度の大きさ
  Point direction_;       // 進行方向の単位ベクトル

public:
  DecelerationBallPhysics(Point initial_position, Point initial_velocity, double deceleration = 0.5)
  : initial_position_(initial_position),
    initial_velocity_(initial_velocity),
    deceleration_(deceleration)
  {
    initial_speed_ = initial_velocity_.norm();
    if (initial_speed_ > 0) {
      direction_ = initial_velocity_.normalized();
    } else {
      direction_ = Point(0, 0);  // 速度がゼロの場合
    }
  }

  // ボールが完全に停止するまでの時間を計算
  double getStopTime() const
  {
    if (initial_speed_ == 0) return 0;
    return initial_speed_ / deceleration_;
  }

  // ボールが停止するまでに移動する最大距離を計算
  double getMaxDistance() const
  {
    if (initial_speed_ == 0) return 0;
    double stop_time = getStopTime();
    return initial_speed_ * stop_time - 0.5 * deceleration_ * stop_time * stop_time;
  }

  // 指定時間後のボール位置を計算（2次元）
  Point getPositionAt(double time) const
  {
    if (initial_speed_ == 0) {
      return initial_position_;
    }

    double stop_time = getStopTime();

    if (time >= stop_time) {
      // 既に停止している場合
      double max_distance = getMaxDistance();
      return initial_position_ + direction_ * max_distance;
    } else {
      // まだ動いている場合
      double distance_traveled = initial_speed_ * time - 0.5 * deceleration_ * time * time;
      return initial_position_ + direction_ * distance_traveled;
    }
  }

  // 指定時間後のボール速度を計算（2次元）
  Point getVelocity(double time) const
  {
    if (initial_speed_ == 0) {
      return Point(0, 0);
    }

    double stop_time = getStopTime();

    if (time >= stop_time) {
      // 既に停止している場合
      return Point(0, 0);
    } else {
      // まだ動いている場合：v = v0 - a*t
      double current_speed = initial_speed_ - deceleration_ * time;
      return direction_ * current_speed;
    }
  }

  static std::optional<double> getTimeToReachDistance(
    double distance, double initial_speed, double deceleration = 0.5)
  {
    if (initial_speed <= 0) {
      return distance == 0 ? std::make_optional(0.0) : std::nullopt;
    }

    // 最大到達距離チェック
    double stop_time = initial_speed / deceleration;
    double max_distance = initial_speed * stop_time - 0.5 * deceleration * stop_time * stop_time;

    if (distance > max_distance) {
      return std::nullopt;
    }

    // 二次方程式を解く: -0.5 * deceleration * t^2 + initial_speed * t - distance = 0
    double a = -0.5 * deceleration;
    double b = initial_speed;
    double c = -distance;

    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
      return std::nullopt;
    }

    double sqrt_discriminant = sqrt(discriminant);
    double t1 = (-b + sqrt_discriminant) / (2 * a);
    double t2 = (-b - sqrt_discriminant) / (2 * a);

    // より早く到達する時間を選ぶ（物理的に意味のある解）
    if (t1 > 0 && t1 <= stop_time) {
      return t1;
    } else if (t2 > 0 && t2 <= stop_time) {
      return t2;
    }

    return std::nullopt;
  }

  // 指定距離に到達する時間を計算
  std::optional<double> getTimeToReachDistance(double distance) const
  {
    return DecelerationBallPhysics::getTimeToReachDistance(distance, initial_speed_, deceleration_);
  }

  // 指定位置に到達する時間を計算（2次元）
  std::optional<double> getTimeToReachPosition(Point target_position) const
  {
    if (initial_speed_ == 0) {
      return (target_position - initial_position_).norm() == 0 ? std::make_optional(0.0)
                                                               : std::nullopt;
    }

    // 目標位置が進行方向上にあるかチェック
    Point to_target = target_position - initial_position_;
    double distance_to_target = to_target.norm();

    if (distance_to_target == 0) {
      return 0.0;  // 既に目標位置にいる
    }

    Point target_direction = to_target.normalized();

    // 進行方向と目標方向の内積をチェック（同じ方向かどうか）
    double dot_product = direction_.dot(target_direction);

    // 許容誤差を設けて、ほぼ同じ方向かチェック
    const double epsilon = 1e-6;
    if (dot_product < 1.0 - epsilon) {
      return std::nullopt;  // 目標位置が進行方向上にない
    }

    return getTimeToReachDistance(distance_to_target);
  }
};

inline auto getFutureBallPosition(
  Point ball_pos, Point ball_vel, double t, double deceleration = 0.5) -> Point
{
  DecelerationBallPhysics physics(ball_pos, ball_vel, deceleration);
  return physics.getPositionAt(t);
}

/**
 * ボールが指定された距離に到達するのにかかる時間を計算します
 * @param distance_to_target 目標までの距離
 * @param current_ball_vel 現在のボール速度の大きさ
 * @param deceleration 減速度（デフォルトは0.5）
 * @return ボールが目標距離に到達するまでの時間
 */
inline auto getBallReachTime(
  double distance_to_target, double current_ball_vel, double deceleration = 0.5)
  -> std::optional<double>
{
  return DecelerationBallPhysics::getTimeToReachDistance(
    distance_to_target, current_ball_vel, deceleration);
}

inline auto generateSequence(double start, double end, double step) -> std::vector<double>
{
  int size = (end - start) / step + 1;
  return ranges::views::iota(0, size) |
         ranges::views::transform([&](int i) -> double { return start + i * step; }) |
         ranges::to<std::vector>();
}

inline auto getBallSequence(double t_horizon, double t_step, Point ball_pos, Point ball_vel)
  -> std::vector<std::pair<Point, double>>
{
  auto t_ball_sequence = generateSequence(0.0, t_horizon, t_step);
  return t_ball_sequence | ranges::views::transform([&](double t) {
           return std::make_pair(getFutureBallPosition(ball_pos, ball_vel, t), t);
         }) |
         ranges::views::transform(
           [](const auto & p) { return std::make_pair(p.first, p.second); }) |
         ranges::to<std::vector>();
}
}  // namespace crane
#endif  // CRANE_BASICS__BALL_MODEL_HPP_
