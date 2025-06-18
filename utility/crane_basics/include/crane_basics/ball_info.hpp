// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__BALL_INFO_HPP_
#define CRANE_BASICS__BALL_INFO_HPP_

#include <algorithm>
#include <cmath>
#include <crane_basics/boost_geometry.hpp>
#include <crane_basics/geometry_operations.hpp>
#include <functional>
#include <limits>
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

  // Ball model parameters
  double deceleration = 0.5;    // Rolling deceleration (m/s²)
  double gravity = -9.81;       // Gravity acceleration (m/s²)
  double air_resistance = 0.0;  // Air resistance coefficient (future use)

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

  // State-aware ball physics functions
  [[nodiscard]] auto getPredictedPosition(double time_ahead) const -> Point
  {
    switch (state) {
      case State::STOPPED:
        return pos;

      case State::ROLLING:
        return getRollingPredictedPosition(time_ahead);

      case State::FLYING: {
        auto parabolic = ParabolicPhysics{*this};
        auto [landing_pos, landing_time] = parabolic.getGroundIntersection();

        if (time_ahead <= landing_time) {
          // Still in air - use 3D parabolic motion
          Point3D pos_3d = parabolic.getPredictedPosition3D(time_ahead);
          return {pos_3d.x(), pos_3d.y()};
        } else {
          // Landed and now rolling
          double time_after_landing = time_ahead - landing_time;
          Point landing_vel = parabolic.getPredictedVelocity2D(landing_time);

          // Create temporary ball state for rolling calculation after landing
          Ball landing_ball;
          landing_ball.pos = landing_pos;
          landing_ball.vel = landing_vel;
          landing_ball.state = State::ROLLING;

          return landing_ball.getRollingPredictedPosition(time_after_landing);
        }
      }
    }
    return pos;  // fallback
  }

  [[nodiscard]] auto getPredictedVelocity(double time_ahead) const -> Point
  {
    switch (state) {
      case State::STOPPED:
        return {0, 0};

      case State::ROLLING:
        return getRollingPredictedVelocity(time_ahead);

      case State::FLYING: {
        auto parabolic = ParabolicPhysics{*this};
        auto [landing_pos, landing_time] = parabolic.getGroundIntersection();

        if (time_ahead <= landing_time) {
          // Still in air
          return parabolic.getPredictedVelocity2D(time_ahead);
        } else {
          // Landed and now rolling
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
    return {0, 0};  // fallback
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

        // Find closest point on trajectory to target
        // First check trajectory while flying
        auto flight_time = getFlyingTimeToClosestPointFrom(target_position);
        if (flight_time && *flight_time <= landing_time) {
          return flight_time;
        }

        // Check trajectory after landing (rolling phase)
        Ball landing_ball;
        landing_ball.pos = landing_pos;
        landing_ball.vel = parabolic.getPredictedVelocity2D(landing_time);
        landing_ball.state = State::ROLLING;

        auto rolling_time = landing_ball.getRollingTimeToReachClosestPointFrom(target_position);
        if (rolling_time) {
          return landing_time + *rolling_time;
        }

        // If no valid rolling time, return landing time as closest point
        return landing_time;
      }
    }
    return std::nullopt;  // fallback
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
    return 0.0;  // fallback
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
    return 0.0;  // fallback
  }

private:
  // Helper functions for rolling physics (2D deceleration model)
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

    // Project target onto ball's trajectory line
    double projection_length = to_target.dot(ball_direction);

    // If projection is negative, closest point is current position
    if (projection_length <= 0) {
      return std::make_optional(0.0);
    }

    // Get time to reach the projected distance
    return getRollingTimeToReachDistance(projection_length);
  }

  [[nodiscard]] auto getFlyingTimeToClosestPointFrom(const Point & target_position) const
    -> std::optional<double>
  {
    // For parabolic trajectory, find time when ball is closest to target
    auto parabolic = ParabolicPhysics{*this};

    // Use numerical approach to find minimum distance
    // Sample trajectory at regular intervals to find closest point
    double min_distance = std::numeric_limits<double>::max();
    double best_time = 0.0;

    auto [landing_pos, landing_time] = parabolic.getGroundIntersection();
    constexpr double time_step = 0.01;  // 10ms intervals

    for (double t = 0.0; t <= landing_time; t += time_step) {
      Point3D pos_3d = parabolic.getPredictedPosition3D(t);
      Point pos_2d(pos_3d.x(), pos_3d.y());
      double distance = (pos_2d - target_position).norm();

      if (distance < min_distance) {
        min_distance = distance;
        best_time = t;
      }
    }

    // Refine the result with smaller steps around the best time
    double start_time = std::max(0.0, best_time - time_step);
    double end_time = std::min(landing_time, best_time + time_step);
    constexpr double fine_step = 0.001;  // 1ms intervals for refinement

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

  // 3D parabolic physics for flying balls
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
      // X and Y velocities remain constant (no air resistance)
      // Z velocity changes due to gravity but we only return 2D velocity
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

  // Convenient functions for trajectory segment creation and closest point calculations
  [[nodiscard]] auto getTrajectorySegment(double time_horizon) const -> Segment
  {
    Point end_point;
    switch (state) {
      case State::STOPPED:
        // For stopped ball, create a zero-length segment at current position
        end_point = pos;
        break;

      case State::ROLLING:
        // For rolling ball, use physics-aware prediction
        end_point = getPredictedPosition(time_horizon);
        break;

      case State::FLYING:
        // For flying ball, consider landing and rolling phases
        end_point = getPredictedPosition(time_horizon);
        break;
    }
    return Segment(pos, end_point);
  }

  [[nodiscard]] auto getClosestPointToTrajectory(
    const Point & position, double time_horizon = 10.0) const -> ClosestPoint
  {
    Segment trajectory = getTrajectorySegment(time_horizon);
    return getClosestPointAndDistance(position, trajectory);
  }

  // Ball sequence generation with state transition support
  [[nodiscard]] auto getBallSequence(double t_horizon, double t_step) const
    -> std::vector<std::pair<Point, double>>
  {
    std::vector<std::pair<Point, double>> sequence;

    if (t_step <= 0 || t_horizon <= 0) {
      return sequence;
    }

    // Generate time sequence
    auto time_sequence = generateSequence(0.0, t_horizon, t_step);

    // Handle different states with potential transitions
    switch (state) {
      case State::STOPPED:
        // Ball doesn't move
        for (double t : time_sequence) {
          sequence.emplace_back(pos, t);
        }
        break;

      case State::ROLLING:
        // Simple rolling physics
        for (double t : time_sequence) {
          sequence.emplace_back(getPredictedPosition(t), t);
        }
        break;

      case State::FLYING: {
        // More complex: flying -> landing -> rolling transition
        auto parabolic = ParabolicPhysics{*this};
        auto [landing_pos, landing_time] = parabolic.getGroundIntersection();
        Point landing_vel = parabolic.getPredictedVelocity2D(landing_time);

        for (double t : time_sequence) {
          if (t <= landing_time) {
            // Still flying - use 3D parabolic motion projected to 2D
            Point3D pos_3d = parabolic.getPredictedPosition3D(t);
            sequence.emplace_back(Point(pos_3d.x(), pos_3d.y()), t);
          } else {
            // Landed and now rolling
            double time_after_landing = t - landing_time;

            // Create temporary ball state for rolling calculation
            Ball rolling_ball;
            rolling_ball.pos = landing_pos;
            rolling_ball.vel = landing_vel;
            rolling_ball.state = State::ROLLING;
            rolling_ball.deceleration = deceleration;  // Use same parameters

            Point rolling_pos = rolling_ball.getRollingPredictedPosition(time_after_landing);
            sequence.emplace_back(rolling_pos, t);
          }
        }
      } break;
    }

    return sequence;
  }

  // Helper function to generate time sequence (public for compatibility)
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

private:
  Hysteresis ball_speed_hysteresis = Hysteresis(0.1, 0.6);
  friend class WorldModelWrapper;
};
}  // namespace crane
#endif  // CRANE_BASICS__BALL_INFO_HPP_
