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
inline auto getTravelTime(std::shared_ptr<RobotInfo> robot, Point target) -> double
{
  // 現在速度で割るだけ
  return (target - robot->pose.pos).norm() / robot->vel.linear.norm();
}

// 旧4引数版は廃止。以下のデフォルト引数付き関数を使用してください。

// 始端速度はロボットの現在速度から自動取得、終端速度を指定可能な台形プロファイル時間
inline auto getSegmentTime(
  double distance, double v_in, double v_out, double alpha_acc, double alpha_dec, double vmax)
  -> double
{
  const double a = std::max(1e-6, alpha_acc);
  const double b = std::max(1e-6, alpha_dec);
  const double L = std::max(0.0, distance);
  v_in = std::max(0.0, v_in);
  v_out = std::max(0.0, v_out);
  const double vm = std::max(1e-6, vmax);

  const double num = 2.0 * a * b * L + b * v_in * v_in + a * v_out * v_out;
  const double den = a + b;
  const double v_peak = std::sqrt(std::max(0.0, num / den));

  if (v_peak <= vm + 1e-9) {
    const double t_acc = std::max(0.0, (v_peak - v_in) / a);
    const double t_dec = std::max(0.0, (v_peak - v_out) / b);
    return t_acc + t_dec;
  } else {
    const double s_acc = std::max(0.0, (vm * vm - v_in * v_in) / (2.0 * a));
    const double s_dec = std::max(0.0, (vm * vm - v_out * v_out) / (2.0 * b));
    const double s_cruise = std::max(0.0, L - s_acc - s_dec);
    const double t_acc = std::max(0.0, (vm - v_in) / a);
    const double t_dec = std::max(0.0, (vm - v_out) / b);
    const double t_cruise = s_cruise / vm;
    return t_acc + t_cruise + t_dec;
  }
}

inline auto getTravelTimeTrapezoidal(
  std::shared_ptr<RobotInfo> robot, Point target, const double max_acceleration,
  const double max_velocity, const double v_end = 0.0) -> double
{
  const Vector2 d = (target - robot->pose.pos);
  const double L = d.norm();
  const double v_in = (L > 1e-9) ? std::max(0.0, robot->vel.linear.dot(d / L)) : 0.0;
  return getSegmentTime(
    L, v_in, std::max(0.0, v_end), max_acceleration, max_acceleration, max_velocity);
}
}  // namespace crane
#endif  // CRANE_PHYSICS__TRAVEL_TIME_HPP_
