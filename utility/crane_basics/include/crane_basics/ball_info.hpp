// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__BALL_INFO_HPP_
#define CRANE_BASICS__BALL_INFO_HPP_

#include <algorithm>
#include <cmath>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <range/v3/all.hpp>
#include <utility>
#include <vector>

namespace crane
{
struct WorldModelWrapper;
struct Hysteresis
{
  Hysteresis(double lower, double upper) : lower_threshold(lower), upper_threshold(upper) {}

  double lower_threshold;

  double upper_threshold;

  bool is_high = false;

  std::function<void(void)> upper_callback = []() {};
  std::function<void(void)> lower_callback = []() {};

  auto update(double value) -> void
  {
    if (not is_high && value > upper_threshold) {
      is_high = true;
      upper_callback();
    }

    if (is_high && value < lower_threshold) {
      is_high = false;
      lower_callback();
    }
  }
};

struct Ball
{
  enum class State {
    STOPPED,
    ROLLING,
    FLYING,
  } state;

  Point pos;

  double pos_z;

  Point vel;

  double vel_z;

  bool detected;

  // ボールモデルパラメータ（後方互換性のため保持）
  double deceleration = 0.5;    // 転がり時の減速度 (m/s²)
  double gravity = -9.81;       // 重力加速度 (m/s²)
  double air_resistance = 0.0;  // 空気抵抗係数 (将来使用)

  [[nodiscard]] auto isMoving(double threshold_velocity = 0.01) const -> bool
  {
    return vel.norm() > threshold_velocity;
  }

  [[nodiscard]] auto isStopped(double threshold_velocity = 0.01) const -> bool
  {
    return not isMoving(threshold_velocity);
  }

  [[nodiscard]] auto isMovingTowards(
    const Point & p, double angle_threshold_deg = 60.0, double near_threshold = 0.2) const -> bool
  {
    if ((pos - p).norm() < near_threshold) {
      return false;
    } else {
      Vector2 dir = (p - pos).normalized();
      return dir.dot(vel.normalized()) > cos(angle_threshold_deg * M_PI / 180.0);
    }
  }

  [[nodiscard]] auto isMovingAwayFrom(
    const Point & p, double angle_threshold_deg = 60.0, double near_threshold = 0.2) const -> bool
  {
    if ((pos - p).norm() < near_threshold) {
      return false;
    } else {
      Vector2 dir = (p - pos).normalized();
      // 内積が負の場合、ボールはその点から離れている
      return dir.dot(vel.normalized()) < -cos(angle_threshold_deg * M_PI / 180.0);
    }
  }

  // 状態対応ボール物理計算関数（後方互換性維持、逆依存なし）
  [[nodiscard]] auto getPredictedPosition(double time_ahead) const -> Point
  {
    // BallPhysicsModelが設定されていても、逆依存を避けるため直接は呼び出さない
    // 代わりに元の実装を使用して後方互換性を維持
    switch (state) {
      case State::STOPPED:
        return pos;
      case State::ROLLING:
        return getRollingPredictedPosition(time_ahead);
      case State::FLYING: {
        auto parabolic = ParabolicPhysics{*this};
        auto [landing_pos, landing_time] = parabolic.getGroundIntersection();
        if (time_ahead <= landing_time) {
          Point3D pos_3d = parabolic.getPredictedPosition3D(time_ahead);
          return {pos_3d.x(), pos_3d.y()};
        } else {
          double time_after_landing = time_ahead - landing_time;
          Point landing_vel = parabolic.getPredictedVelocity2D(landing_time);
          Ball landing_ball;
          landing_ball.pos = landing_pos;
          landing_ball.vel = landing_vel;
          landing_ball.state = State::ROLLING;
          return landing_ball.getRollingPredictedPosition(time_after_landing);
        }
      }
    }
    return pos;
  }

  [[nodiscard]] auto getPredictedVelocity(double time_ahead) const -> Point
  {
    // 逆依存を避けるため、元の実装を使用
    switch (state) {
      case State::STOPPED:
        return {0, 0};
      case State::ROLLING:
        return getRollingPredictedVelocity(time_ahead);
      case State::FLYING: {
        auto parabolic = ParabolicPhysics{*this};
        auto [landing_pos, landing_time] = parabolic.getGroundIntersection();
        if (time_ahead <= landing_time) {
          return parabolic.getPredictedVelocity2D(time_ahead);
        } else {
          double time_after_landing = time_ahead - landing_time;
          Point landing_vel = parabolic.getPredictedVelocity2D(landing_time);
          Ball landing_ball;
          landing_ball.pos = landing_pos;
          landing_ball.vel = landing_vel;
          landing_ball.state = State::ROLLING;
          return landing_ball.getRollingPredictedVelocity(time_after_landing);
        }
      }
    }
    return {0, 0};
  }

  [[nodiscard]] auto getTimeToReachClosestPointFrom(const Point & target_position) const
    -> std::optional<double>
  {
    switch (state) {
      case State::STOPPED:
        return std::make_optional(0.0);

      case State::ROLLING:
        return getRollingTimeToReachClosestPointFrom(target_position);

      case State::FLYING: {
        auto parabolic = ParabolicPhysics{*this};
        auto [landing_pos, landing_time] = parabolic.getGroundIntersection();

        // 軌道上の目標に最も近い点を探す
        // まず飛行中の軌道をチェック
        auto flight_time = getFlyingTimeToClosestPointFrom(target_position);
        if (flight_time && *flight_time <= landing_time) {
          return flight_time;
        }

        // 着地後の軌道をチェック（転がりフェーズ）
        Ball landing_ball;
        landing_ball.pos = landing_pos;
        landing_ball.vel = parabolic.getPredictedVelocity2D(landing_time);
        landing_ball.state = State::ROLLING;

        auto rolling_time = landing_ball.getRollingTimeToReachClosestPointFrom(target_position);
        if (rolling_time) {
          return landing_time + *rolling_time;
        }

        // 有効な転がり時間がない場合、着地時間を最近点として返す
        return landing_time;
      }
    }
    return std::nullopt;  // フォールバック
  }

  [[nodiscard]] auto getStopTime() const -> double
  {
    switch (state) {
      case State::STOPPED:
        return 0.0;

      case State::ROLLING:
        return getRollingStopTime();

      case State::FLYING: {
        auto parabolic = ParabolicPhysics{*this};
        auto [landing_pos, landing_time] = parabolic.getGroundIntersection();
        Point landing_vel = parabolic.getPredictedVelocity2D(landing_time);

        double rolling_stop_time = landing_vel.norm() / deceleration;
        return landing_time + rolling_stop_time;
      }
    }
    return 0.0;
  }

  [[nodiscard]] auto getMaxDistance() const -> double
  {
    switch (state) {
      case State::STOPPED:
        return 0.0;

      case State::ROLLING:
        return getRollingMaxDistance();

      case State::FLYING: {
        auto parabolic = ParabolicPhysics{*this};
        auto [landing_pos, landing_time] = parabolic.getGroundIntersection();
        Point landing_vel = parabolic.getPredictedVelocity2D(landing_time);

        double distance_to_landing = (landing_pos - pos).norm();
        double rolling_distance = getRollingMaxDistanceFromVelocity(landing_vel);

        return distance_to_landing + rolling_distance;
      }
    }
    return 0.0;
  }

private:
  // 転がり物理計算用ヘルパー関数（2D減速モデル）
  [[nodiscard]] auto getRollingStopTime() const -> double
  {
    double speed = vel.norm();
    if (speed == 0) return 0;
    return speed / deceleration;
  }

  [[nodiscard]] auto getRollingMaxDistance() const -> double
  {
    double speed = vel.norm();
    if (speed == 0) return 0;
    double stop_time = getRollingStopTime();
    return speed * stop_time - 0.5 * deceleration * stop_time * stop_time;
  }

  [[nodiscard]] auto getRollingMaxDistanceFromVelocity(const Point & velocity) const -> double
  {
    double speed = velocity.norm();
    if (speed == 0) return 0;
    double stop_time = speed / deceleration;
    return speed * stop_time - 0.5 * deceleration * stop_time * stop_time;
  }

  [[nodiscard]] auto getRollingPredictedPosition(double time_ahead) const -> Point
  {
    double speed = vel.norm();
    if (speed == 0) {
      return pos;
    }

    Point direction = vel.normalized();
    double stop_time = getRollingStopTime();

    if (time_ahead >= stop_time) {
      double max_distance = getRollingMaxDistance();
      return pos + direction * max_distance;
    } else {
      double distance_traveled = speed * time_ahead - 0.5 * deceleration * time_ahead * time_ahead;
      return pos + direction * distance_traveled;
    }
  }

  [[nodiscard]] auto getRollingPredictedVelocity(double time_ahead) const -> Point
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
      double current_speed = speed - deceleration * time_ahead;
      return direction * current_speed;
    }
  }

  [[nodiscard]] auto getRollingTimeToReachClosestPointFrom(const Point & target_position) const
    -> std::optional<double>
  {
    double speed = vel.norm();
    if (speed == 0) {
      return std::make_optional(0.0);
    }

    Point to_target = target_position - pos;
    Point ball_direction = vel.normalized();

    // 目標をボールの軌道線に投影
    double projection_length = to_target.dot(ball_direction);

    // 投影が負の場合、最近点は現在位置
    if (projection_length <= 0) {
      return std::make_optional(0.0);
    }

    // 投影された距離に到達する時間を取得
    return getRollingTimeToReachDistance(projection_length);
  }

  [[nodiscard]] auto getFlyingTimeToClosestPointFrom(const Point & target_position) const
    -> std::optional<double>
  {
    // 放物軌道について、ボールが目標に最も近い時間を見つける
    auto parabolic = ParabolicPhysics{*this};

    // 最小距離を見つけるために数値的アプローチを使用
    // 軌道を一定間隔でサンプリングして最近点を見つける
    double min_distance = std::numeric_limits<double>::max();
    double best_time = 0.0;

    auto [landing_pos, landing_time] = parabolic.getGroundIntersection();
    constexpr double time_step = 0.01;  // 10ms間隔

    for (double t = 0.0; t <= landing_time; t += time_step) {
      Point3D pos_3d = parabolic.getPredictedPosition3D(t);
      Point pos_2d(pos_3d.x(), pos_3d.y());
      double distance = (pos_2d - target_position).norm();

      if (distance < min_distance) {
        min_distance = distance;
        best_time = t;
      }
    }

    // 最適時間周辺でより小さなステップで結果を精密化
    double start_time = std::max(0.0, best_time - time_step);
    double end_time = std::min(landing_time, best_time + time_step);
    constexpr double fine_step = 0.001;  // 精密化のための1ms間隔

    for (double t = start_time; t <= end_time; t += fine_step) {
      Point3D pos_3d = parabolic.getPredictedPosition3D(t);
      Point pos_2d(pos_3d.x(), pos_3d.y());
      double distance = (pos_2d - target_position).norm();

      if (distance < min_distance) {
        min_distance = distance;
        best_time = t;
      }
    }

    return std::make_optional(best_time);
  }

  [[nodiscard]] auto getRollingTimeToReachDistance(double distance) const -> std::optional<double>
  {
    double speed = vel.norm();
    if (speed <= 0) {
      return distance == 0 ? std::make_optional(0.0) : std::nullopt;
    }

    double stop_time = speed / deceleration;
    double max_distance = speed * stop_time - 0.5 * deceleration * stop_time * stop_time;

    if (distance > max_distance) {
      return std::nullopt;
    }

    double a = -0.5 * deceleration;
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

  [[nodiscard]] auto getTimeToTravelDistance(double distance) const -> std::optional<double>
  {
    switch (state) {
      case State::STOPPED:
        return distance == 0 ? std::make_optional(0.0) : std::nullopt;

      case State::ROLLING:
        return getRollingTimeToReachDistance(distance);

      case State::FLYING: {
        // 飛行中のボールについて、軌道距離が目標と等しくなる時間を二分探索で見つける
        auto parabolic = ParabolicPhysics{*this};
        auto [landing_pos, landing_time] = parabolic.getGroundIntersection();

        double max_distance = getMaxDistance();
        if (distance > max_distance) {
          return std::nullopt;
        }

        // 累積距離が目標距離と等しくなる時間を二分探索
        double t_min = 0.0;
        double t_max = getStopTime();
        constexpr double epsilon = 1e-6;

        for (int iter = 0; iter < 100; ++iter) {
          double t_mid = (t_min + t_max) / 2.0;

          // t_midでの累積距離を計算
          double cumulative_distance = 0.0;
          if (t_mid <= landing_time) {
            // まだ飛行中 - 3D軌道距離を計算
            Point3D pos_3d = parabolic.getPredictedPosition3D(t_mid);
            cumulative_distance = (Point(pos_3d.x(), pos_3d.y()) - pos).norm();
          } else {
            // 着地して転がり中
            double distance_to_landing = (landing_pos - pos).norm();
            double time_after_landing = t_mid - landing_time;
            Point landing_vel = parabolic.getPredictedVelocity2D(landing_time);

            // 転がり計算用の一時的なボールを作成
            Ball rolling_ball;
            rolling_ball.pos = landing_pos;
            rolling_ball.vel = landing_vel;
            rolling_ball.state = State::ROLLING;
            rolling_ball.deceleration = deceleration;

            auto rolling_distance_opt =
              rolling_ball.getRollingTimeToReachDistance(distance - distance_to_landing);
            if (rolling_distance_opt && *rolling_distance_opt <= time_after_landing) {
              cumulative_distance = distance;  // 完全一致
            } else {
              Point rolling_pos = rolling_ball.getRollingPredictedPosition(time_after_landing);
              cumulative_distance = distance_to_landing + (rolling_pos - landing_pos).norm();
            }
          }

          if (std::abs(cumulative_distance - distance) < epsilon) {
            return t_mid;
          }

          if (cumulative_distance < distance) {
            t_min = t_mid;
          } else {
            t_max = t_mid;
          }
        }

        // 最良の近似値を返す
        return (t_min + t_max) / 2.0;
      }
    }
    return std::nullopt;
  }

  // 飛行ボール用の3D放物運動物理計算
  class ParabolicPhysics
  {
  public:
    struct Point3DStamped
    {
      Point3D position;
      double time;
    };

  private:
    Point3D initial_position_;
    Point3D initial_velocity_;
    double gravity_;

  public:
    std::vector<Point3DStamped> point_log;

    explicit ParabolicPhysics(const Ball & ball)
    : initial_position_(Point3D(ball.pos.x(), ball.pos.y(), ball.pos_z)),
      initial_velocity_(Point3D(ball.vel.x(), ball.vel.y(), ball.vel_z)),
      gravity_(ball.gravity)
    {
    }

    ParabolicPhysics(Point3D initial_position, Point3D initial_velocity, double gravity = -9.81)
    : initial_position_(initial_position), initial_velocity_(initial_velocity), gravity_(gravity)
    {
    }

    [[nodiscard]] auto getPredictedPosition3D(double time_ahead) const -> Point3D
    {
      Point3D position;
      position.x() = initial_position_.x() + initial_velocity_.x() * time_ahead;
      position.y() = initial_position_.y() + initial_velocity_.y() * time_ahead;
      position.z() = initial_position_.z() + initial_velocity_.z() * time_ahead +
                     0.5 * gravity_ * time_ahead * time_ahead;

      return position;
    }

    [[nodiscard]] auto getPredictedVelocity2D(double /*time_ahead*/) const -> Point
    {
      // XとY速度は一定（空気抵抗なし）
      // Z速度は重力により変化するが、2D速度のみを返す
      return {initial_velocity_.x(), initial_velocity_.y()};
    }

    [[nodiscard]] auto getGroundIntersection() const -> std::pair<Point, double>
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

      // 初期位置も更新
      initial_position_ = p0;
    }

    Point3DStamped getGroundPoint()
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
  };

public:
  [[nodiscard]] auto getParabolicPhysics() const -> ParabolicPhysics
  {
    return ParabolicPhysics{*this};
  }

  // 軌道セグメント作成と最近点計算用の便利関数
  [[nodiscard]] auto getTrajectorySegmentByTime(double time_horizon) const -> Segment
  {
    Point end_point;
    switch (state) {
      case State::STOPPED:
        // 停止しているボールについて、現在位置にゼロ長セグメントを作成
        end_point = pos;
        break;

      case State::ROLLING:
        // 転がりボールについて、物理対応予測を使用
        end_point = getPredictedPosition(time_horizon);
        break;

      case State::FLYING:
        // 飛行ボールについて、着地と転がりフェーズを考慮
        end_point = getPredictedPosition(time_horizon);
        break;
    }
    return Segment(pos, end_point);
  }

  [[nodiscard]] auto getTrajectorySegmentByDistance(double distance) const -> Segment
  {
    Point end_point;
    switch (state) {
      case State::STOPPED:
        // 停止ボールについて、可能であれば速度方向にセグメントを作成
        if (vel.norm() < 1e-6) {
          // 速度情報なし、最小セグメントを作成
          end_point = pos + Point(std::min(distance, 0.1), 0);
        } else {
          Vector2 direction = vel.normalized();
          end_point = pos + direction * distance;
        }
        break;

      case State::ROLLING:
        // 指定距離を移動するのに必要な時間を計算
        if (auto time_to_distance = getTimeToTravelDistance(distance)) {
          end_point = getPredictedPosition(*time_to_distance);
        } else {
          // 距離に到達できない、最大到達可能位置を使用
          end_point = getPredictedPosition(getStopTime());
        }
        break;

      case State::FLYING:
        // 飛行ボールについて、軌道に沿って距離を移動する時間を計算
        if (auto time_to_distance = getTimeToTravelDistance(distance)) {
          end_point = getPredictedPosition(*time_to_distance);
        } else {
          // 距離が大きすぎる場合は着地位置を使用
          auto parabolic = ParabolicPhysics{*this};
          auto [landing_pos, landing_time] = parabolic.getGroundIntersection();
          end_point = landing_pos;
        }
        break;
    }
    return Segment(pos, end_point);
  }

  [[nodiscard]] auto getClosestPointToTrajectory(
    const Point & position, double time_horizon = 10.0) const -> ClosestPoint
  {
    Segment trajectory = getTrajectorySegmentByTime(time_horizon);
    return getClosestPointAndDistance(position, trajectory);
  }

  // 後方互換性エイリアス
  [[nodiscard]] auto getTrajectorySegment(double time_horizon) const -> Segment
  {
    return getTrajectorySegmentByTime(time_horizon);
  }

  // 状態遷移サポート付きボールシーケンス生成
  [[nodiscard]] auto getBallSequence(double t_horizon, double t_step) const
    -> std::vector<std::pair<Point, double>>
  {
    std::vector<std::pair<Point, double>> sequence;

    if (t_step <= 0 || t_horizon <= 0) {
      return sequence;
    }

    // 時間シーケンスを生成
    auto time_sequence = generateSequence(0.0, t_horizon, t_step);

    // 潜在的な遷移を持つ異なる状態を処理
    switch (state) {
      case State::STOPPED:
        // ボールは動かない
        for (double t : time_sequence) {
          sequence.emplace_back(pos, t);
        }
        break;

      case State::ROLLING:
        // シンプルな転がり物理計算
        for (double t : time_sequence) {
          sequence.emplace_back(getPredictedPosition(t), t);
        }
        break;

      case State::FLYING: {
        // より複雑：飛行 → 着地 → 転がり遷移
        auto parabolic = ParabolicPhysics{*this};
        auto [landing_pos, landing_time] = parabolic.getGroundIntersection();
        Point landing_vel = parabolic.getPredictedVelocity2D(landing_time);

        for (double t : time_sequence) {
          if (t <= landing_time) {
            // まだ飛行中 - 3D放物運動を2Dに投影
            Point3D pos_3d = parabolic.getPredictedPosition3D(t);
            sequence.emplace_back(Point(pos_3d.x(), pos_3d.y()), t);
          } else {
            // 着地して転がり中
            double time_after_landing = t - landing_time;

            // 転がり計算用の一時的なボール状態を作成
            Ball rolling_ball;
            rolling_ball.pos = landing_pos;
            rolling_ball.vel = landing_vel;
            rolling_ball.state = State::ROLLING;
            rolling_ball.deceleration = deceleration;  // 同じパラメータを使用

            Point rolling_pos = rolling_ball.getRollingPredictedPosition(time_after_landing);
            sequence.emplace_back(rolling_pos, t);
          }
        }
      } break;
    }

    return sequence;
  }

  // 時間シーケンス生成用ヘルパー関数（互換性のためpublic）
  [[nodiscard]] static auto generateSequence(double start, double end, double step)
    -> std::vector<double>
  {
    std::vector<double> sequence;
    if (step <= 0) return sequence;

    int size = static_cast<int>((end - start) / step) + 1;
    sequence.reserve(size);

    for (int i = 0; i < size; ++i) {
      double time_val = start + i * step;
      if (time_val <= end) {
        sequence.push_back(time_val);
      }
    }

    return sequence;
  }

  // ROS2メッセージとの変換関数
  template <typename BallInfoMsg>
  void toMsg(BallInfoMsg & msg) const
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

    // モデルパラメータ
    msg.deceleration = static_cast<float>(deceleration);
    msg.gravity = static_cast<float>(gravity);
    msg.air_resistance = static_cast<float>(air_resistance);
  }

  template <typename BallInfoMsg>
  void fromMsg(const BallInfoMsg & msg)
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

    // モデルパラメータ
    deceleration = msg.deceleration;
    gravity = msg.gravity;
    air_resistance = msg.air_resistance;
  }

private:
  Hysteresis ball_speed_hysteresis = Hysteresis(0.1, 0.6);
  friend class WorldModelWrapper;
};
}  // namespace crane
#endif  // CRANE_BASICS__BALL_INFO_HPP_
