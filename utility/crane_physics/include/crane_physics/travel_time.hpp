// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PHYSICS__TRAVEL_TIME_HPP_
#define CRANE_PHYSICS__TRAVEL_TIME_HPP_

#include <crane_physics/robot_info.hpp>
#include <memory>

namespace crane
{
inline auto getTravelTimeTrapezoidal(
  std::shared_ptr<RobotInfo> robot, Point target, const double max_acceleration,
  const double max_velocity, const double v_end = 0.0) -> double
{
  const Vector2 d = (target - robot->pose.pos);
  const double L = d.norm();
  const double v_in = (L > 1e-9) ? std::max(0.0, robot->vel.linear.dot(d / L)) : 0.0;

  const double a = std::max(1e-6, max_acceleration);
  const double b = std::max(1e-6, max_acceleration);

  const double num = 2.0 * a * b * L + b * v_in * v_in + a * v_end * v_end;
  const double den = a + b;
  const double v_peak = std::sqrt(std::max(0.0, num / den));

  if (v_peak <= max_velocity + 1e-9) {
    const double t_acc = std::max(0.0, (v_peak - v_in) / a);
    const double t_dec = std::max(0.0, (v_peak - v_end) / b);
    return t_acc + t_dec;
  } else {
    const double s_acc = std::max(0.0, (max_velocity * max_velocity - v_in * v_in) / (2.0 * a));
    const double s_dec = std::max(0.0, (max_velocity * max_velocity - v_end * v_end) / (2.0 * b));
    const double s_cruise = std::max(0.0, L - s_acc - s_dec);
    const double t_acc = std::max(0.0, (max_velocity - v_in) / a);
    const double t_dec = std::max(0.0, (max_velocity - v_end) / b);
    const double t_cruise = s_cruise / max_velocity;
    return t_acc + t_cruise + t_dec;
  }
}
}  // namespace crane
#endif  // CRANE_PHYSICS__TRAVEL_TIME_HPP_
