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
inline auto getFutureBallPosition(
  Point ball_pos, Point ball_vel, double t, double deceleration = 0.5) -> Point
{
  // 指定時間までに停止する場合
  if (ball_vel.norm() - deceleration * t < 0.) {
    double stop_time = ball_vel.norm() / deceleration;
    return ball_pos + ball_vel * stop_time -
           0.5 * stop_time * stop_time * deceleration * ball_vel.normalized();
  } else {
    return ball_pos + ball_vel * t - 0.5 * t * t * deceleration * ball_vel.normalized();
  }
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
