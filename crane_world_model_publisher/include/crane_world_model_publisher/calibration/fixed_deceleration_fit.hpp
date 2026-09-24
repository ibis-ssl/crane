// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__FIXED_DECELERATION_FIT_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__FIXED_DECELERATION_FIT_HPP_

#include <cmath>
#include <cstddef>
#include <vector>

namespace crane
{
struct FixedDecelerationFit
{
  double v0 = 0.0;
  double rmse = 0.0;
  // グローバル減速度の評価に使える軌道か（点数・v0・RMSE の条件を満たすか）
  bool accepted = false;
};

/**
 * @brief 減速度 decel を固定したモデル v(t) = v0 - decel * t を 1 軌道に当てはめる
 * @param time_points 軌道の最初の点を 0 とした時刻 [s]（v0 は t=0 の速度になる）
 * @param velocities 各時刻の速度 [m/s]
 * @return v0 は残差平方和を最小にする mean(v + decel * t)。点数が min_data_points 未満なら
 *         accepted=false で v0・rmse は 0 のまま
 */
inline FixedDecelerationFit fitFixedDeceleration(
  const std::vector<double> & time_points, const std::vector<double> & velocities, double decel,
  size_t min_data_points)
{
  FixedDecelerationFit fit;
  const size_t n = time_points.size();
  if (n == 0 || n < min_data_points) {
    return fit;
  }

  double v0_sum = 0.0;
  for (size_t i = 0; i < n; ++i) {
    v0_sum += velocities[i] + decel * time_points[i];
  }
  fit.v0 = v0_sum / n;

  double squared_error_sum = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const double error = velocities[i] - (fit.v0 - decel * time_points[i]);
    squared_error_sum += error * error;
  }
  fit.rmse = std::sqrt(squared_error_sum / n);

  fit.accepted = fit.v0 > 0.1 && fit.rmse < 2.0;  // 合理的な初速度と誤差
  return fit;
}
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__FIXED_DECELERATION_FIT_HPP_
