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
/// 経路のうち判定に使う区間を原点からの距離 [m] で絞る。既定は経路全体。
///
/// 味方の横取り判定では終端側を外して使う。受領点そのものを含めると、
/// 受領点の近くに味方が立っているだけで「先着する」と出て候補が消える。
/// 実測では 563 候補中 427 がこれで落ち、計画が成立しなくなった。
/// 見たいのは「経路を横切って途中で奪う味方」なので、終端の近傍は対象外にする。
struct PassPathRange
{
  double along_min = 0.0;
  /// 負なら経路長をそのまま使う。
  double along_max = -1.0;
};

inline auto straightPassInterceptionSlack(
  const Point & origin, const Point & target, const StraightPassFlight & flight,
  const Point & enemy_pos, const Vector2 & enemy_vel, double max_acceleration, double max_velocity,
  const PassPathRange & range = {}) -> double
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
  const double along_min = std::clamp(range.along_min, 0.0, distance);
  const double along_max =
    std::clamp(range.along_max < 0.0 ? distance : range.along_max, along_min, distance);
  if (along_max <= along_min) {
    return std::numeric_limits<double>::infinity();
  }
  const double span = along_max - along_min;
  double worst = slack_at(std::clamp((enemy_pos - origin).dot(direction), along_min, along_max));
  const int divisions = static_cast<int>(std::clamp(std::ceil(span / 0.25), 1.0, 64.0));
  for (int i = 0; i <= divisions; ++i) {
    worst = std::min(worst, slack_at(along_min + span * i / divisions));
  }
  return worst;
}
}  // namespace crane

#endif  // CRANE_PHYSICS__PASS_INTERCEPTION_HPP_
