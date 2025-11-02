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
/**
 * @brief 台形速度プロファイルの中間計算結果を保持する構造体
 *
 * getTravelTimeTrapezoidalとgetPredictedPositionTrapezoidalで共通する計算を
 * 一度だけ実行し、結果を再利用することでパフォーマンスを改善する。
 */
struct TrapezoidalProfile
{
  double distance;           ///< 目標までの距離 [m]
  double initial_vel;        ///< 目標方向の初速度成分 [m/s]
  double accel_time;         ///< 加速時間 [s]
  double decel_time;         ///< 減速時間 [s]
  double accel_distance;     ///< 加速距離 [m]
  double decel_distance;     ///< 減速距離 [m]
  bool has_cruise_phase;     ///< 定速区間の有無
  double cruise_distance;    ///< 定速区間の距離 [m] (定速区間ありの場合)
  double cruise_time;        ///< 定速時間 [s] (定速区間ありの場合)
  double total_travel_time;  ///< 総移動時間 [s]
};

inline auto getTravelTime(std::shared_ptr<RobotInfo> robot, Point target) -> double
{
  // 現在速度で割るだけ
  return (target - robot->pose.pos).norm() / robot->vel.linear.norm();
}

/**
 * @brief 台形速度プロファイルの全ての中間計算を実行
 *
 * @param current_pos 現在位置
 * @param current_vel 現在速度ベクトル
 * @param target 目標位置
 * @param max_acceleration 最大加速度 [m/s^2]
 * @param max_velocity 最大速度 [m/s]
 * @return TrapezoidalProfile 計算結果を含む構造体
 */
inline auto calculateTrapezoidalProfile(
  const Point & current_pos, const Vector2 & current_vel, const Point & target,
  const double max_acceleration, const double max_velocity) -> TrapezoidalProfile
{
  TrapezoidalProfile profile;

  // 初期計算
  profile.distance = (target - current_pos).norm();
  profile.initial_vel = (target - current_pos).normalized().dot(current_vel);

  // 加速・減速にかかる時間
  profile.accel_time = (max_velocity - profile.initial_vel) / max_acceleration;
  profile.decel_time = max_velocity / max_acceleration;

  // 加速・減速にかかる距離
  profile.accel_distance = (profile.initial_vel + max_velocity) * profile.accel_time / 2;
  profile.decel_distance = max_velocity * profile.decel_time / 2;

  // 定速区間の有無判定
  profile.has_cruise_phase = (profile.accel_distance + profile.decel_distance < profile.distance);

  if (profile.has_cruise_phase) {
    // 定速区間が存在する場合
    profile.cruise_distance = profile.distance - (profile.accel_distance + profile.decel_distance);
    profile.cruise_time = profile.cruise_distance / max_velocity;
    profile.total_travel_time = profile.accel_time + profile.cruise_time + profile.decel_time;
  } else {
    // 加速距離と減速距離の合計が移動距離を超える場合、定速区間はない
    profile.cruise_distance = 0.0;
    profile.cruise_time = 0.0;
    // d_acc = v0 * t1 + 0.5 * a * t1^2
    // v_max = v0 + a * t1
    // d_dec = v_max^2 / (2 * a) = (v0 + a * t1)^2 / (2 * a)
    //       = (a^2 * t1^2 + 2 * a * v0 * t1 + v0^2) / (2 * a)
    // d_acc = t1^2 * ( 0.5 * a ) + t1 * (v0    ) + (0                    )
    // d_dec = t1^2 * ( 0.5 * a ) + t1 * (v0    ) + (0.5 * v0^2 / a       )
    // dist  = t1^2 * ( a       ) + t1 * (2 * v0) + (0.5 * v0^2 / a       )
    // 0     = t1^2 * ( a       ) + t1 * (2 * v0) + (0.5 * v0^2 / a - dist)
    // t1 = (-v0 + sqrt((v0)^2 - a * ((0.5 * v0^2 / a - dist)))) /  a
    //    = (-v0 + sqrt(v0^2 - 0.5 * v0^2 + a * dist ))) / a
    //    = (-v0 + sqrt(0.5 * v0^2 + a * dist)) / a
    // t2 = v_max / a = (v0 + a * t1) / a
    // tM = t1 + t2
    //    =  (-v0 + sqrt(0.5 * v0^2 + a * dist)) / a + (v0 + a * t1) / a
    //    =  (-v0 + sqrt(0.5 * v0^2 + a * dist) + v0 + -v0 + sqrt(0.5 * v0^2 + a * dist))) / a
    //    =  ( - v0 + 2 sqrt(0.5 * v0^2 + a * dist)) / a
    profile.total_travel_time =
      (-profile.initial_vel + 2 * sqrt(
                                    0.5 * profile.initial_vel * profile.initial_vel +
                                    max_acceleration * profile.distance)) /
      max_acceleration;
  }

  return profile;
}

inline auto getTravelTimeTrapezoidal(
  const Point & current_pos, const Vector2 & current_vel, const Point & target,
  const double max_acceleration, const double max_velocity) -> double
{
  return calculateTrapezoidalProfile(
           current_pos, current_vel, target, max_acceleration, max_velocity)
    .total_travel_time;
}

inline auto getTravelTimeTrapezoidal(
  std::shared_ptr<RobotInfo> robot, Point target, const double max_acceleration,
  const double max_velocity) -> double
{
  return getTravelTimeTrapezoidal(
    robot->pose.pos, robot->vel.linear, target, max_acceleration, max_velocity);
}

/**
 * @brief 台形速度プロファイルで指定時間後の予測位置を計算
 *
 * @param current_pos 現在位置
 * @param current_vel 現在速度ベクトル
 * @param target_pos 目標位置
 * @param time 予測時間 [s]
 * @param max_acceleration 最大加速度 [m/s^2]
 * @param max_velocity 最大速度 [m/s]
 * @return 予測位置
 */
inline auto getPredictedPositionTrapezoidal(
  const Point & current_pos, const Vector2 & current_vel, const Point & target_pos,
  const double time, const double max_acceleration, const double max_velocity) -> Point
{
  // 台形速度プロファイルを一度だけ計算（重複計算を排除）
  auto profile = calculateTrapezoidalProfile(
    current_pos, current_vel, target_pos, max_acceleration, max_velocity);

  // 指定時間が移動時間以上なら目標位置に到達
  if (time >= profile.total_travel_time) {
    return target_pos;
  }

  // 指定時間が0以下なら現在位置
  if (time <= 0.0) {
    return current_pos;
  }

  Vector2 direction = (target_pos - current_pos).normalized();
  double traveled_distance = 0.0;

  if (!profile.has_cruise_phase) {
    // 定速区間なし：全体が加速→減速
    double t_half = std::sqrt(2.0 * profile.distance / max_acceleration);
    if (time <= t_half) {
      // 加速フェーズ
      traveled_distance = profile.initial_vel * time + 0.5 * max_acceleration * time * time;
    } else {
      // 減速フェーズ
      traveled_distance = profile.distance - 0.5 * max_acceleration *
                                               (profile.total_travel_time - time) *
                                               (profile.total_travel_time - time);
    }
  } else {
    // 定速区間あり
    if (time <= profile.accel_time) {
      // 加速区間
      traveled_distance = profile.initial_vel * time + 0.5 * max_acceleration * time * time;
    } else if (time <= profile.accel_time + profile.cruise_time) {
      // 加速→定速区間
      double cruise_time_elapsed = time - profile.accel_time;
      traveled_distance = profile.accel_distance + max_velocity * cruise_time_elapsed;
    } else {
      // 加速→定速→減速区間
      double decel_time_elapsed = time - profile.accel_time - profile.cruise_time;
      double decel_dist = max_velocity * decel_time_elapsed -
                          0.5 * max_acceleration * decel_time_elapsed * decel_time_elapsed;
      traveled_distance = profile.accel_distance + profile.cruise_distance + decel_dist;
    }
  }

  // 安全のため目標位置を超えないようにクリップ
  traveled_distance = std::min(traveled_distance, profile.distance);

  return current_pos + direction * traveled_distance;
}
}  // namespace crane
#endif  // CRANE_PHYSICS__TRAVEL_TIME_HPP_
