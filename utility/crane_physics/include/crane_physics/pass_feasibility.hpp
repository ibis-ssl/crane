// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PHYSICS__PASS_FEASIBILITY_HPP_
#define CRANE_PHYSICS__PASS_FEASIBILITY_HPP_

#include <crane_physics/pass_kick.hpp>
#include <crane_physics/travel_time.hpp>

namespace crane
{
/// 受領点フィージビリティ判定のパラメータ
struct ReceiveFeasibilityParams
{
  double receiver_max_acceleration = 3.0;                      ///< 受け手の最大加速度 [m/s^2]
  double receiver_max_velocity = 4.0;                          ///< 受け手の最大速度 [m/s]
  double desired_arrival_speed = 1.5;                          ///< 望ましい受領点到達速度 [m/s]
  double ball_deceleration = pass_kick::kDefaultDeceleration;  ///< ボール減速度 [m/s^2]
  double min_initial_speed = 1.0;                              ///< キック初速下限 [m/s]
  double max_initial_speed = 6.5;                              ///< キック初速上限 [m/s]
  double margin = 0.3;  ///< 受け手が先着するための安全余裕 [s]
};

/// 受領点フィージビリティの結果
struct ReceiveFeasibility
{
  bool feasible = false;              ///< 受け手がボールより先に受領点へ到達できるか
  double ball_travel_time = 0.0;      ///< ボール到達所要時間 [s]
  double receiver_travel_time = 0.0;  ///< 受け手到達所要時間 [s]
  double kick_speed = 0.0;            ///< クランプ後のキック初速 [m/s]（planStraightPass 由来）
  bool ball_reachable = false;        ///< クランプ後初速でボールが受領点に届くか
};

/**
 * @brief 受け手先着の可否を表す純粋な比較述語
 *
 * receiver_travel_time + margin <= ball_travel_time を判定する。マージンと比較方向を
 * 一点に集約し、単体テストで固定できるようにするための最小の seam。
 */
[[nodiscard]] inline auto isReceiveFeasible(
  double receiver_travel_time, double ball_travel_time, double margin) -> bool
{
  return receiver_travel_time + margin <= ball_travel_time;
}

/**
 * @brief 受領点が「受け手先着かつボール到達可能」かを閉形式で判定する（純関数）
 *
 * ボール到達は planStraightPass（直進転がりモデル）、受け手到達は Bang-Bang 台形
 * プロファイルで見積もる。ratePassCandidate の前段の安価な早期棄却として用いる。
 * ボールが届かない（reachable=false）場合は travel_time が無限大になり得るため、
 * ball_reachable を先に確認してからマージン比較する。
 */
[[nodiscard]] inline auto feasibleReceivePoint(
  const Point & pass_origin, const Point & receive_point, const Point & receiver_pos,
  const Vector2 & receiver_vel, const ReceiveFeasibilityParams & params) -> ReceiveFeasibility
{
  ReceiveFeasibility r;
  const double distance = (receive_point - pass_origin).norm();
  const auto plan = planStraightPass(
    distance, params.desired_arrival_speed, params.ball_deceleration, params.min_initial_speed,
    params.max_initial_speed);
  r.ball_travel_time = plan.travel_time;
  r.ball_reachable = plan.reachable;
  r.kick_speed = plan.initial_speed;
  r.receiver_travel_time = getTravelTimeTrapezoidal(
    receiver_pos, receiver_vel, receive_point, params.receiver_max_acceleration,
    params.receiver_max_velocity);
  r.feasible = r.ball_reachable &&
               isReceiveFeasible(r.receiver_travel_time, r.ball_travel_time, params.margin);
  return r;
}
}  // namespace crane

#endif  // CRANE_PHYSICS__PASS_FEASIBILITY_HPP_
