// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "bag_kick_stats.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <sstream>

namespace crane::bag
{

namespace
{

/// nearest-rank percentile（q は 0-100）。空なら 0。
double percentile(std::vector<double> sorted_values, double q)
{
  if (sorted_values.empty()) {
    return 0.0;
  }
  const size_t n = sorted_values.size();
  const size_t rank = static_cast<size_t>(std::ceil(q / 100.0 * static_cast<double>(n)));
  return sorted_values[std::min(n - 1, rank > 0 ? rank - 1 : 0)];
}

KickStatsGroup build_group(const std::vector<const KickPredictionTraceData *> & traces)
{
  KickStatsGroup g;
  g.count = traces.size();
  if (traces.empty()) {
    return g;
  }

  std::vector<double> speed_abs, dist_abs;
  double speed_sum = 0, dist_sum = 0;
  std::map<int, KickPowerBin> bins;
  for (const auto * tr : traces) {
    speed_sum += tr->speed_error_percent;
    dist_sum += tr->distance_error_percent;
    speed_abs.push_back(std::abs(tr->speed_error_percent));
    dist_abs.push_back(std::abs(tr->distance_error_percent));

    const int bi = std::clamp(static_cast<int>(tr->kick_power * 10.0), 0, 9);
    auto & bin = bins[bi];
    bin.power_lo = bi / 10.0;
    bin.power_hi = (bi + 1) / 10.0;
    ++bin.count;
    bin.mean_predicted_speed += tr->predicted_ball_speed;
    bin.mean_actual_speed += tr->actual_ball_speed;
    bin.mean_predicted_distance += tr->predicted_stop_distance;
    bin.mean_actual_distance += tr->actual_stop_distance;
  }

  const double n = static_cast<double>(traces.size());
  g.speed_bias_percent = speed_sum / n;
  g.dist_bias_percent = dist_sum / n;

  std::sort(speed_abs.begin(), speed_abs.end());
  std::sort(dist_abs.begin(), dist_abs.end());
  auto mean = [](const std::vector<double> & v) {
    double s = 0;
    for (double x : v) s += x;
    return v.empty() ? 0.0 : s / static_cast<double>(v.size());
  };
  g.speed_abs_mean = mean(speed_abs);
  g.speed_abs_p50 = percentile(speed_abs, 50);
  g.speed_abs_p90 = percentile(speed_abs, 90);
  g.dist_abs_mean = mean(dist_abs);
  g.dist_abs_p50 = percentile(dist_abs, 50);
  g.dist_abs_p90 = percentile(dist_abs, 90);

  for (auto & [bi, bin] : bins) {
    const double bn = static_cast<double>(bin.count);
    bin.mean_predicted_speed /= bn;
    bin.mean_actual_speed /= bn;
    bin.mean_predicted_distance /= bn;
    bin.mean_actual_distance /= bn;
    g.power_bins.push_back(bin);
  }
  return g;
}

}  // namespace

KickStats compute_kick_stats(const std::vector<TimestampedMsg<KickPredictionTraceData>> & traces)
{
  KickStats s;
  s.total = traces.size();

  std::vector<const KickPredictionTraceData *> straight, chip;
  for (const auto & tm : traces) {
    const auto & tr = tm.msg;
    if (!tr.has_actual) {
      continue;
    }
    ++s.with_actual;
    if (!tr.has_prediction) {
      continue;
    }
    (tr.is_chip_kick ? chip : straight).push_back(&tr);
  }
  s.straight = build_group(straight);
  s.chip = build_group(chip);
  return s;
}

std::string format_kick_stats(const KickStats & s)
{
  std::ostringstream out;
  char buf[256];
  out << "=== KICK PREDICTION STATS ===\n";
  std::snprintf(buf, sizeof(buf), "traces: %zu (with actual: %zu)\n", s.total, s.with_actual);
  out << buf;

  auto print_group = [&](const char * label, const KickStatsGroup & g, bool show_speed) {
    std::snprintf(buf, sizeof(buf), "[%s] n=%zu\n", label, g.count);
    out << buf;
    if (g.count == 0) {
      return;
    }
    if (show_speed) {
      std::snprintf(
        buf, sizeof(buf), "  speed error %%   : bias %+6.1f  |mean %5.1f  p50 %5.1f  p90 %5.1f\n",
        g.speed_bias_percent, g.speed_abs_mean, g.speed_abs_p50, g.speed_abs_p90);
      out << buf;
    }
    std::snprintf(
      buf, sizeof(buf), "  distance error %%: bias %+6.1f  |mean %5.1f  p50 %5.1f  p90 %5.1f\n",
      g.dist_bias_percent, g.dist_abs_mean, g.dist_abs_p50, g.dist_abs_p90);
    out << buf;
    for (const auto & b : g.power_bins) {
      if (show_speed) {
        std::snprintf(
          buf, sizeof(buf),
          "    power %.1f-%.1f: n=%-3zu  v pred/act %.2f/%.2f m/s  d pred/act %.2f/%.2f m\n",
          b.power_lo, b.power_hi, b.count, b.mean_predicted_speed, b.mean_actual_speed,
          b.mean_predicted_distance, b.mean_actual_distance);
      } else {
        std::snprintf(
          buf, sizeof(buf), "    power %.1f-%.1f: n=%-3zu  d pred/act %.2f/%.2f m\n", b.power_lo,
          b.power_hi, b.count, b.mean_predicted_distance, b.mean_actual_distance);
      }
      out << buf;
    }
  };
  print_group("straight", s.straight, /*show_speed=*/true);
  print_group("chip", s.chip, /*show_speed=*/false);
  return out.str();
}

}  // namespace crane::bag
