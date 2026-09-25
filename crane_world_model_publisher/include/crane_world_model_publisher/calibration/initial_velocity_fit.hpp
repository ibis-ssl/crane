// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__INITIAL_VELOCITY_FIT_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__INITIAL_VELOCITY_FIT_HPP_

#include <cmath>
#include <cstddef>
#include <vector>

namespace crane
{
struct InitialVelocityFit
{
  double initial_velocity = 0.0;
  // 固定傾きモデルの R²。傾きを固定しているので負にもなる。速度が全点同じ（ss_tot=0）なら 0
  double r_squared = 0.0;
  double residual_rms = 0.0;
};

/**
 * @brief 減速度 deceleration を固定したモデル v(t) = v0 - deceleration * t で v0 を推定する
 * @param time_points 軌道の最初の ROLLING 点を 0 とした時刻 [s]。キック時刻ではないので、
 *        v0 は最初の ROLLING 点での速度になる
 * @param velocities 各時刻の速度 [m/s]（位置 [m] の差分）
 * @param deceleration 減速度 [m/s²]（正の値で減速）
 * @return initial_velocity は残差平方和を最小にする mean(v + deceleration * t)。点が無ければ全て 0
 */
inline InitialVelocityFit fitInitialVelocityWithFixedDeceleration(
  const std::vector<double> & time_points, const std::vector<double> & velocities,
  double deceleration)
{
  InitialVelocityFit fit;
  const size_t n = time_points.size();
  if (n == 0) {
    return fit;
  }

  double v0_sum = 0.0;
  double velocity_sum = 0.0;
  for (size_t i = 0; i < n; ++i) {
    v0_sum += velocities[i] + deceleration * time_points[i];
    velocity_sum += velocities[i];
  }
  fit.initial_velocity = v0_sum / n;
  const double mean_velocity = velocity_sum / n;

  double ss_res = 0.0;
  double ss_tot = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const double residual = velocities[i] + deceleration * time_points[i] - fit.initial_velocity;
    ss_res += residual * residual;
    const double deviation = velocities[i] - mean_velocity;
    ss_tot += deviation * deviation;
  }
  fit.r_squared = (ss_tot > 0.0) ? (1.0 - ss_res / ss_tot) : 0.0;
  fit.residual_rms = std::sqrt(ss_res / n);
  return fit;
}
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__INITIAL_VELOCITY_FIT_HPP_
