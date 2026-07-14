// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <string>
#include <vector>

#include "bag_reader.hpp"

namespace crane::bag
{

/// キック力ビン（0.1刻み）の予実平均
struct KickPowerBin
{
  double power_lo = 0;
  double power_hi = 0;
  size_t count = 0;
  double mean_predicted_speed = 0;
  double mean_actual_speed = 0;
  double mean_predicted_distance = 0;
  double mean_actual_distance = 0;
};

/// straight / chip 各グループの予実誤差統計。
/// bias は符号付き平均（実測-予測、予測比%）で較正のずれ方向を示す。
/// abs_* は絶対値の分布（p50/p90 は nearest-rank）。
struct KickStatsGroup
{
  size_t count = 0;
  double speed_bias_percent = 0;
  double speed_abs_mean = 0;
  double speed_abs_p50 = 0;
  double speed_abs_p90 = 0;
  double dist_bias_percent = 0;
  double dist_abs_mean = 0;
  double dist_abs_p50 = 0;
  double dist_abs_p90 = 0;
  std::vector<KickPowerBin> power_bins;
};

struct KickStats
{
  size_t total = 0;        ///< トレース総数
  size_t with_actual = 0;  ///< 実績（actual）付きトレース数
  KickStatsGroup straight;
  KickStatsGroup chip;
};

/// /kick_prediction_traces から予実誤差統計を計算する純粋関数。
/// prediction と actual が両方あるトレースのみ集計対象。
KickStats compute_kick_stats(const std::vector<TimestampedMsg<KickPredictionTraceData>> & traces);

std::string format_kick_stats(const KickStats & s);

}  // namespace crane::bag
