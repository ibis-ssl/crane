// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__BALL_MODEL_HPP_
#define CRANE_BASICS__BALL_MODEL_HPP_

#include <algorithm>
#include <cmath>
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
    constexpr double epsilon = 1e-6;
    if (dot_product < 1.0 - epsilon) {
      return std::nullopt;  // 目標位置が進行方向上にない
    }

    return getTimeToReachDistance(distance_to_target);
  }
};

struct ParabolicBallPhysics
{
  struct Point3DStamped
  {
    Point3D position;
    double time;
  };

  ParabolicBallPhysics(Point3D initial_position, Point3D initial_velocity)
  : initial_position_(initial_position), initial_velocity_(initial_velocity)
  {
  }

  Point3D initial_position_;
  Point3D initial_velocity_;
  std::vector<Point3DStamped> point_log;

  void estimateInitialVelocityFromPointLog()
  {
    if (point_log.size() < 2) {
      // データが不十分な場合は推定できない
      initial_velocity_ = Point3D::Zero();
      return;
    }

    // 時系列でソート
    ranges::sort(point_log, [](const Point3DStamped & a, const Point3DStamped & b) {
      return a.time < b.time;
    });

    // 最小時刻を基準時刻とする
    double t0 = point_log[0].time;
    Point3D p0 = point_log[0].position;

    // 最小二乗法で初期速度を推定
    // 放物運動の方程式: p(t) = p0 + v0*t + 0.5*g*t^2
    // ここでg = (0, 0, -9.81) (重力加速度)
    constexpr double gravity = -9.81;

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
        initial_velocity_.z() += 0.5 * (-gravity) * dt;
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
      dp.z() -= 0.5 * gravity * dt * dt;

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
        initial_velocity_.z() += 0.5 * (-gravity) * dt;
      } else {
        initial_velocity_ = Point3D::Zero();
      }
      return;
    }

    // 最小二乗解を計算
    Point3D numerator = sum_t_dp * n_points - sum_dp * sum_t;
    initial_velocity_ = numerator / denominator;

    // 初期位置も更新
    initial_position_ = p0;
  }

  Point3DStamped getGroundPoint()
  {
    // 放物運動の方程式: z(t) = z0 + vz0*t + 0.5*g*t^2
    // 着地条件: z(t) = 0
    // 0 = z0 + vz0*t - 4.905*t^2 (g = -9.81なので0.5*g = -4.905)

    constexpr double gravity = -9.81;
    double z0 = initial_position_.z();
    double vz0 = initial_velocity_.z();

    // 既に地面にいる場合
    if (std::abs(z0) < 1e-6) {
      return {initial_position_, 0.0};
    }

    // 二次方程式: -4.905*t^2 + vz0*t + z0 = 0
    // at^2 + bt + c = 0の形に変換
    double a = 0.5 * gravity;  // -4.905
    double b = vz0;
    double c = z0;

    double discriminant = b * b - 4 * a * c;

    // 解が存在しない場合（地面に到達しない）
    if (discriminant < 0) {
      // 最高点での位置を返す（近似的な着地点として）
      double t_peak = -vz0 / gravity;
      if (t_peak < 0) t_peak = 0;  // 負の時間は物理的に意味がない

      Point3D peak_position;
      peak_position.x() = initial_position_.x() + initial_velocity_.x() * t_peak;
      peak_position.y() = initial_position_.y() + initial_velocity_.y() * t_peak;
      peak_position.z() =
        initial_position_.z() + initial_velocity_.z() * t_peak + 0.5 * gravity * t_peak * t_peak;

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
