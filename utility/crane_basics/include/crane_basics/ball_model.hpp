// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__BALL_MODEL_HPP_
#define CRANE_BASICS__BALL_MODEL_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <optional>
#include <vector>

namespace crane
{
inline std::optional<Point> getFutureBallPosition(
  Point ball_pos, Point ball_vel, double t, double deceleration = 0.5)
{
  if (ball_vel.norm() - deceleration * t < 0.) {
    return std::nullopt;
  } else {
    return ball_pos + ball_vel * t - 0.5 * t * t * deceleration * ball_vel.normalized();
  }
}

inline std::vector<double> generateSequence(double start, double end, double step)
{
  int size = (end - start) / step + 1;
  std::vector<double> sequence(size);
  double current = start;
  std::generate_n(sequence.begin(), size, [&current, step]() mutable {
    double temp = current;
    current += step;
    return temp;
  });
  return sequence;
}

inline std::vector<std::pair<Point, double>> getBallSequence(
  double t_horizon, double t_step, Point ball_pos, Point ball_vel)
{
  std::vector<double> t_ball_sequence = generateSequence(0.0, t_horizon, t_step);
  std::vector<std::pair<Point, double>> ball_sequence;
  for (auto t_ball : t_ball_sequence) {
    if (auto p_ball = getFutureBallPosition(ball_pos, ball_vel, t_ball); p_ball.has_value()) {
      ball_sequence.push_back({p_ball.value(), t_ball});
    }
  }
  return ball_sequence;
}
}  // namespace crane
#endif  // CRANE_BASICS__BALL_MODEL_HPP_
