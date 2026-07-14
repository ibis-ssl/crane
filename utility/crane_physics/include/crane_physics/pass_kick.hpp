// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PHYSICS__PASS_KICK_HPP_
#define CRANE_PHYSICS__PASS_KICK_HPP_

#include <algorithm>
#include <cmath>
#include <crane_physics/pass.hpp>
#include <crane_physics/robot_info.hpp>
#include <limits>
#include <vector>

namespace crane
{
namespace pass_kick
{
/// ボール減速度の既定値 [m/s^2]（BallPhysicsModel と整合。実行時は ball().getPhysicsModel() 参照）
constexpr double kDefaultDeceleration = 0.7;

/// 直進パスの計画結果
struct StraightPlan
{
  double initial_speed = 0.0;  ///< キック初速 [m/s]（setKickStraightTargetSpeed へ渡す値）
  double arrival_speed = 0.0;  ///< 受領点での到達速度 [m/s]
  double travel_time = 0.0;    ///< 受領点までの転がり所要時間 [s]
  bool reachable = false;      ///< クランプ後の初速で受領点に到達できるか
};

/// パスキック（直進 or チップ）の計画結果
struct KickPlan
{
  bool is_chip = false;        ///< チップキックを使うか
  double chip_distance = 0.0;  ///< is_chip 時のチップ着地距離 [m]
  StraightPlan straight;       ///< is_chip=false 時の直進計画
  bool feasible = false;       ///< 有効な計画が得られたか
};
}  // namespace pass_kick

/**
 * @brief 必要初速: 距離 d 先で到達速度 v_arr にするためのキック初速
 *
 * 一定減速 a のもとで v(d)^2 = v0^2 - 2 a d より v0 = sqrt(v_arr^2 + 2 a d)。
 */
inline auto requiredInitialSpeed(double distance, double arrival_speed, double deceleration)
  -> double
{
  const double d = std::max(0.0, distance);
  const double a = std::max(0.0, deceleration);
  const double v_arr = std::max(0.0, arrival_speed);
  return std::sqrt(v_arr * v_arr + 2.0 * a * d);
}

/**
 * @brief 到達速度: 初速 v0 のボールが距離 d 先で持つ速度
 *
 * sqrt(max(0, v0^2 - 2 a d))。停止して届かない場合は 0。
 */
inline auto arrivalSpeed(double distance, double initial_speed, double deceleration) -> double
{
  const double d = std::max(0.0, distance);
  const double a = std::max(0.0, deceleration);
  const double v0 = std::max(0.0, initial_speed);
  const double v_sq = v0 * v0 - 2.0 * a * d;
  return v_sq > 0.0 ? std::sqrt(v_sq) : 0.0;
}

/**
 * @brief 転がり時間: 初速 v0 のボールが距離 d を転がる所要時間 [s]
 *
 * v_arr = v0 - a t より t = (v0 - v_arr)/a。停止して届かない場合は無限大。
 */
inline auto rollingTravelTime(double distance, double initial_speed, double deceleration) -> double
{
  const double d = std::max(0.0, distance);
  const double v0 = std::max(0.0, initial_speed);
  const double a = std::max(0.0, deceleration);
  if (d < 1e-9) {
    return 0.0;
  }
  if (a < 1e-9) {
    return v0 > 1e-9 ? d / v0 : std::numeric_limits<double>::infinity();
  }
  const double v_arr_sq = v0 * v0 - 2.0 * a * d;
  if (v_arr_sq <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return (v0 - std::sqrt(v_arr_sq)) / a;
}

/**
 * @brief 直進パスの初速・到達速度・所要時間を計画する
 *
 * 望ましい到達速度から必要初速を逆算し、[min, max] にクランプする。
 * クランプにより到達速度・所要時間は再計算される。
 *
 * @param distance             パス距離 [m]
 * @param desired_arrival_speed 望ましい受領点到達速度 [m/s]
 * @param deceleration          ボール減速度 [m/s^2]
 * @param min_initial_speed     初速下限 [m/s]
 * @param max_initial_speed     初速上限 [m/s]
 */
inline auto planStraightPass(
  double distance, double desired_arrival_speed, double deceleration, double min_initial_speed,
  double max_initial_speed) -> pass_kick::StraightPlan
{
  pass_kick::StraightPlan plan;
  const double a = std::max(0.0, deceleration);
  const double lo = std::min(min_initial_speed, max_initial_speed);
  const double hi = std::max(min_initial_speed, max_initial_speed);
  const double v0 = std::clamp(requiredInitialSpeed(distance, desired_arrival_speed, a), lo, hi);
  plan.initial_speed = v0;
  plan.arrival_speed = arrivalSpeed(distance, v0, a);
  plan.travel_time = rollingTravelTime(distance, v0, a);
  plan.reachable = plan.arrival_speed > 0.0 && std::isfinite(plan.travel_time);
  return plan;
}

/**
 * @brief パスキック（直進/チップ）の統合計画
 *
 * getPassAnalysis でパスライン上の敵遮蔽を判定し、遮蔽があればチップ、
 * なければ planStraightPass による直進を計画する。
 *
 * @param ball                  パス起点（ボール位置）
 * @param target                受領点
 * @param their_robots          敵ロボット
 * @param desired_arrival_speed 望ましい受領点到達速度 [m/s]
 * @param deceleration          ボール減速度 [m/s^2]
 * @param min_initial_speed     直進初速下限 [m/s]
 * @param max_initial_speed     直進初速上限 [m/s]
 * @param block_distance        パスライン遮蔽と見なす敵距離 [m]
 * @param chip_margin           チップ着地距離に足す余裕 [m]
 */
inline auto planPassKick(
  const Point & ball, const Point & target, std::vector<RobotInfo::SharedPtr> their_robots,
  double desired_arrival_speed, double deceleration, double min_initial_speed,
  double max_initial_speed, double block_distance = 0.2, double chip_margin = 0.2)
  -> pass_kick::KickPlan
{
  pass_kick::KickPlan plan;
  const auto analysis = getPassAnalysis(ball, target, std::move(their_robots), block_distance);
  if (analysis.need_chip) {
    plan.is_chip = true;
    plan.chip_distance = analysis.required_chip_distance + chip_margin;
    plan.feasible = plan.chip_distance > 0.0;
    return plan;
  }
  const double distance = (target - ball).norm();
  plan.is_chip = false;
  plan.straight = planStraightPass(
    distance, desired_arrival_speed, deceleration, min_initial_speed, max_initial_speed);
  plan.feasible = plan.straight.reachable;
  return plan;
}
}  // namespace crane

#endif  // CRANE_PHYSICS__PASS_KICK_HPP_
