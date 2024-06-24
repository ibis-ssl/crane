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
}  // namespace crane
#endif  // CRANE_BASICS__BALL_MODEL_HPP_
