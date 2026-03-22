// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PHYSICS__TRAVEL_TIME_HPP_
#define CRANE_PHYSICS__TRAVEL_TIME_HPP_

#include <crane_physics/bang_bang_trajectory.hpp>
#include <crane_physics/robot_info.hpp>
#include <memory>

namespace crane
{

inline auto getTravelTime(std::shared_ptr<RobotInfo> robot, Point target) -> double
{
  // 現在速度で割るだけ
  return (target - robot->pose.pos).norm() / robot->vel.linear.norm();
}

/**
 * @brief 台形速度プロファイル（Bang-Bang制御）で移動時間を計算
 *
 * @param current_pos 現在位置
 * @param current_vel 現在速度ベクトル
 * @param target 目標位置
 * @param max_acceleration 最大加速度 [m/s^2]
 * @param max_velocity 最大速度 [m/s]
 * @return 移動時間 [s]
 */
inline auto getTravelTimeTrapezoidal(
  const Point & current_pos, const Vector2 & current_vel, const Point & target,
  const double max_acceleration, const double max_velocity) -> double
{
  double dist = (target - current_pos).norm();
  if (dist < 1e-6) {
    return 0.0;
  }

  Vector2 dir = (target - current_pos).normalized();
  // 目標方向への初速度成分
  double v0 = current_vel.dot(dir);

  BangBangTrajectory1D traj;
  // 初期位置0, 目標位置distとして1次元軌道を生成
  traj.generate(0.0, dist, v0, max_velocity, max_acceleration);

  return traj.getTotalTime();
}

inline auto getTravelTimeTrapezoidal(
  std::shared_ptr<RobotInfo> robot, Point target, const double max_acceleration,
  const double max_velocity) -> double
{
  return getTravelTimeTrapezoidal(
    robot->pose.pos, robot->vel.linear, target, max_acceleration, max_velocity);
}

/**
 * @brief 台形速度プロファイル（Bang-Bang制御）で指定時間後の予測位置を計算
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
  double dist = (target_pos - current_pos).norm();
  if (dist < 1e-6) {
    return target_pos;
  }

  if (time <= 0.0) {
    return current_pos;
  }

  Vector2 dir = (target_pos - current_pos).normalized();
  double v0 = current_vel.dot(dir);

  BangBangTrajectory1D traj;
  traj.generate(0.0, dist, v0, max_velocity, max_acceleration);

  // 指定時間が移動時間以上なら目標位置に到達
  if (time >= traj.getTotalTime()) {
    return target_pos;
  }

  double traveled_distance = traj.getPosition(time);
  return current_pos + dir * traveled_distance;
}

/**
 * @brief ボール追い越し時の迂回を考慮した移動時間を計算
 *
 * ロボットがボール進行方向の後方（同じ方向）からインターセプト地点へアプローチする場合、
 * ボールを追い越して正面に回り込む迂回時間を加算する。
 *
 * alignment = (intercept_point - robot_pos).normalized().dot(ball_dir) が正の場合（後方から接近）、
 * インターセプト地点の垂直方向へオフセットしたウェイポイント経由の2区間パスで時間を推定する。
 *
 * @param current_pos ロボットの現在位置
 * @param current_vel ロボットの現在速度ベクトル
 * @param intercept_point インターセプト地点
 * @param ball_velocity ボールの速度ベクトル
 * @param max_acceleration 最大加速度 [m/s^2]
 * @param max_velocity 最大速度 [m/s]
 * @param circling_radius 迂回オフセットの基準半径 [m]
 * @return 移動時間 [s]
 */
inline auto getTravelTimeWithApproachPenalty(
  const Point & current_pos, const Vector2 & current_vel, const Point & intercept_point,
  const Vector2 & ball_velocity, const double max_acceleration, const double max_velocity,
  const double circling_radius = 0.15) -> double
{
  double ball_speed = ball_velocity.norm();
  if (ball_speed < 1e-3) {
    return getTravelTimeTrapezoidal(
      current_pos, current_vel, intercept_point, max_acceleration, max_velocity);
  }

  Vector2 robot_to_intercept = intercept_point - current_pos;
  double dist = robot_to_intercept.norm();
  if (dist < 1e-6) {
    return 0.0;
  }

  Vector2 ball_dir = ball_velocity / ball_speed;
  double alignment = robot_to_intercept.normalized().dot(ball_dir);

  if (alignment <= 0.0) {
    return getTravelTimeTrapezoidal(
      current_pos, current_vel, intercept_point, max_acceleration, max_velocity);
  }

  // ウェイポイント: インターセプト地点から垂直方向 + ボール後方側にオフセット
  Vector2 perp(-ball_dir.y(), ball_dir.x());
  if ((current_pos - intercept_point).dot(perp) < 0) {
    perp = -perp;  // ロボットに近い側を選択
  }

  double offset = circling_radius * alignment;
  Point waypoint = intercept_point + perp * offset - ball_dir * offset;

  double total_dist = (waypoint - current_pos).norm() + (intercept_point - waypoint).norm();
  double v0 = current_vel.dot((waypoint - current_pos).normalized());

  BangBangTrajectory1D traj;
  traj.generate(0.0, total_dist, v0, max_velocity, max_acceleration);

  double straight_time = getTravelTimeTrapezoidal(
    current_pos, current_vel, intercept_point, max_acceleration, max_velocity);
  return std::max(traj.getTotalTime(), straight_time);
}

}  // namespace crane
#endif  // CRANE_PHYSICS__TRAVEL_TIME_HPP_
