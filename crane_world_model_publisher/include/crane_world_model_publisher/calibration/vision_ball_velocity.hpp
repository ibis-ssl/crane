// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__VISION_BALL_VELOCITY_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__VISION_BALL_VELOCITY_HPP_

#include <cmath>
#include <crane_physics/ball_info.hpp>
#include <optional>
#include <rclcpp/time.hpp>
#include <utility>
#include <vector>

namespace crane::calibration
{
// 速度を差分で求める時間間隔の範囲（両端を含む）
constexpr double VISION_VELOCITY_MIN_DT = 1e-5;
constexpr double VISION_VELOCITY_MAX_DT = 0.2;

struct VisionVelocity
{
  Point vel;
  double vel_z;
};

// 直前の Vision 位置（history の末尾）と今回の Vision 位置の差分から速度を求める。
// 両端とも生の位置を使う。片側だけ平滑化すると、等速でも速度の符号が反転し大きさも過小になる。
// dt が範囲外なら nullopt を返す。
inline auto computeVisionVelocity(
  const std::vector<std::pair<rclcpp::Time, Ball>> & history, const Ball & current, double dt)
  -> std::optional<VisionVelocity>
{
  if (history.empty() || !(dt >= VISION_VELOCITY_MIN_DT && dt <= VISION_VELOCITY_MAX_DT)) {
    return std::nullopt;
  }
  const Ball & prev = history.back().second;
  return VisionVelocity{(current.pos - prev.pos) / dt, (current.pos_z - prev.pos_z) / dt};
}

// dt が範囲外のときの速度と状態。長いギャップの後は停止扱い、短すぎる間隔は前回値を引き継ぐ。
inline auto holdVelocityForInvalidDt(const Ball & prev, double dt, Ball & current) -> void
{
  if (dt > VISION_VELOCITY_MAX_DT) {
    current.vel = Point(0, 0);
    current.vel_z = 0;
    current.state = Ball::State::STOPPED;
  } else {
    current.vel = prev.vel;
    current.vel_z = prev.vel_z;
    current.state = prev.state;
  }
}

inline auto classifyBallState(const Ball & ball) -> Ball::State
{
  if (ball.vel.norm() < 0.05) {
    return Ball::State::STOPPED;
  }
  if (ball.pos_z > 0.02 || std::abs(ball.vel_z) > 0.1) {
    return Ball::State::FLYING;
  }
  return Ball::State::ROLLING;
}
}  // namespace crane::calibration

#endif  // CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__VISION_BALL_VELOCITY_HPP_
