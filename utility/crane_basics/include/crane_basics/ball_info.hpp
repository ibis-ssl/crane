// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__BALL_INFO_HPP_
#define CRANE_BASICS__BALL_INFO_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <functional>

namespace crane
{
struct WorldModelWrapper;
struct Hysteresis
{
  Hysteresis(double lower, double upper) : lower_threshold(lower), upper_threshold(upper) {}

  double lower_threshold, upper_threshold;

  bool is_high = false;

  std::function<void(void)> upper_callback = []() {};
  std::function<void(void)> lower_callback = []() {};

  void update(double value);
};

struct Ball
{
  Point pos;

  Point vel;

  bool is_curve;

  [[nodiscard]] bool isMoving(double threshold_velocity = 0.01) const
  {
    return vel.norm() > threshold_velocity;
  }

  [[nodiscard]] bool isStopped(double threshold_velocity = 0.01) const
  {
    return not isMoving(threshold_velocity);
  }

  [[nodiscard]] bool isMovingTowards(
    const Point & p, double angle_threshold_deg = 60.0, double near_threshold = 0.2) const;

  [[nodiscard]] bool isMovingAwayFrom(
    const Point & p, double angle_threshold_deg = 60.0, double near_threshold = 0.2) const;

private:
  Hysteresis ball_speed_hysteresis = Hysteresis(0.1, 0.6);
  friend class WorldModelWrapper;
};
}  // namespace crane
#endif  // CRANE_BASICS__BALL_INFO_HPP_
