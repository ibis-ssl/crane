// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__TRAVEL_TIME_HPP_
#define CRANE_BASICS__TRAVEL_TIME_HPP_

#include <crane_basics/robot_info.hpp>
#include <memory>

namespace crane
{
inline double getTravelTime(std::shared_ptr<RobotInfo> robot, Point target)
{
  // 現在速度で割るだけ
  return (target - robot->pose.pos).norm() / robot->vel.linear.norm();
}

inline double getTravelTimeTrapezoidal(
  std::shared_ptr<RobotInfo> robot, Point target, const double max_acceleration = 4.,
  const double max_velocity = 4.)
{
  double distance = (target - robot->pose.pos).norm();
  double initial_vel = robot->vel.linear.norm();

  // 加速・減速にかかる時間
  double accel_time = (max_velocity - initial_vel) / max_acceleration;
  double decel_time = max_velocity / max_acceleration;

  // 加速・減速にかかる距離
  double accel_distance = (initial_vel + max_velocity) * accel_time / 2;
  double decel_distance = max_velocity * decel_time / 2;

  if (accel_distance + decel_distance >= distance) {
    // 加速距離と減速距離の合計が移動距離を超える場合、定速区間はない
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
    return (-initial_vel +
            2 * sqrt(0.5 * initial_vel * initial_vel + max_acceleration * distance)) /
           max_acceleration;
  } else {
    // 定速区間が存在する場合
    double remaining_distance = distance - (accel_distance + decel_distance);
    double cruise_time = remaining_distance / max_velocity;
    return accel_time + cruise_time + decel_time;
  }
}
}  // namespace crane
#endif  // CRANE_BASICS__TRAVEL_TIME_HPP_
