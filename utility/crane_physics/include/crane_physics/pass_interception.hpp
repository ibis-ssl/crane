// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PHYSICS__PASS_INTERCEPTION_HPP_
#define CRANE_PHYSICS__PASS_INTERCEPTION_HPP_

#include <algorithm>
#include <cmath>
#include <crane_physics/pass_kick.hpp>
#include <crane_physics/travel_time.hpp>
#include <limits>

namespace crane
{
/// feasibility と迎撃評価で共有する直進ボールの運動条件。
struct StraightPassFlight
{
  double initial_speed;
  double deceleration;
};

/// 敵の到達時間 - ボール到達時間の最小値。非正なら迎撃可能。
/// 終点だけでなく経路全体を最大64分割し、敵の射影点も必ず評価する。
/// 敵は触れるだけでパスを阻止できるため、受球のための回り込みは要求しない。
inline auto straightPassInterceptionSlack(
  const Point & origin, const Point & target, const StraightPassFlight & flight,
  const Point & enemy_pos, const Vector2 & enemy_vel, double max_acceleration, double max_velocity)
  -> double
{
  const double distance = (target - origin).norm();
  if (
    !std::isfinite(distance) || distance < 1e-6 || !std::isfinite(flight.initial_speed) ||
    flight.initial_speed <= 0.0 || !std::isfinite(flight.deceleration) ||
    flight.deceleration < 0.0 ||
    !std::isfinite(rollingTravelTime(distance, flight.initial_speed, flight.deceleration))) {
    return -std::numeric_limits<double>::infinity();
  }
  const Vector2 direction = (target - origin) / distance;
  const auto slack_at = [&](double along) {
    const Point point = origin + along * direction;
    const double robot_time =
      getTravelTimeTrapezoidal(enemy_pos, enemy_vel, point, max_acceleration, max_velocity);
    const double ball_time = rollingTravelTime(along, flight.initial_speed, flight.deceleration);
    return robot_time - ball_time;
  };
  double worst = slack_at(std::clamp((enemy_pos - origin).dot(direction), 0.0, distance));
  const int divisions = static_cast<int>(std::clamp(std::ceil(distance / 0.25), 1.0, 64.0));
  for (int i = 0; i <= divisions; ++i) {
    worst = std::min(worst, slack_at(distance * i / divisions));
  }
  return worst;
}
}  // namespace crane

#endif  // CRANE_PHYSICS__PASS_INTERCEPTION_HPP_
