// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

export module crane_basics:ball_info;

import :boost_geometry;
import <functional>;

namespace crane
{
class WorldModelWrapper;  // Forward declaration
}

export namespace crane
{
// struct WorldModelWrapper; // Forward declaration moved above

export struct Hysteresis
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

export struct Ball
{
  Point pos;

  Point vel;

  bool is_curve;

  bool detected;

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

private:
  Hysteresis ball_speed_hysteresis = Hysteresis(0.1, 0.6);
  friend class WorldModelWrapper;
};
}  // namespace crane
