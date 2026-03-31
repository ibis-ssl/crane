// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/ateb_time_parameterizer.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace crane::ateb
{

std::vector<std::pair<double, Point>> TimeParameterizer::computeArcLengthTable(
  const ElasticBand & band) const
{
  if (band.nodes.size() < 2) {
    return {};
  }

  // バンドの各ノード間の距離を累積して弧長テーブルを構築
  std::vector<std::pair<double, Point>> raw_table;
  raw_table.reserve(band.nodes.size());
  double arc = 0.0;
  raw_table.emplace_back(0.0, band.nodes.front().pos);
  for (size_t i = 1; i < band.nodes.size(); ++i) {
    const double ds = (band.nodes[i].pos - band.nodes[i - 1].pos).norm();
    arc += ds;
    raw_table.emplace_back(arc, band.nodes[i].pos);
  }

  const double total_length = raw_table.back().first;
  if (total_length < config_.min_ds) {
    return raw_table;
  }

  // 等間隔でリサンプル
  const int n = std::max(2, config_.sample_count);
  std::vector<std::pair<double, Point>> table;
  table.reserve(n);

  for (int i = 0; i < n; ++i) {
    const double target_s = total_length * i / (n - 1);
    // raw_tableから線形補間
    auto it = std::lower_bound(
      raw_table.begin(), raw_table.end(), target_s,
      [](const std::pair<double, Point> & a, double val) { return a.first < val; });

    if (it == raw_table.begin()) {
      table.emplace_back(target_s, raw_table.front().second);
    } else if (it == raw_table.end()) {
      table.emplace_back(target_s, raw_table.back().second);
    } else {
      auto prev = std::prev(it);
      const double ds = it->first - prev->first;
      if (ds < 1e-9) {
        table.emplace_back(target_s, prev->second);
      } else {
        const double alpha = (target_s - prev->first) / ds;
        const Point interp = prev->second * (1.0 - alpha) + it->second * alpha;
        table.emplace_back(target_s, interp);
      }
    }
  }

  return table;
}

double TimeParameterizer::computeCurvature(
  const Point & p0, const Point & p1, const Point & p2) const
{
  // Menger曲率: 1/R = 4*A / (|p0p1| * |p1p2| * |p0p2|)
  // where A = 三角形の面積
  const double a = (p1 - p0).norm();
  const double b = (p2 - p1).norm();
  const double c = (p2 - p0).norm();
  const double s = (a + b + c) * 0.5;
  const double area_sq = s * (s - a) * (s - b) * (s - c);
  if (area_sq <= 0.0 || a < 1e-9 || b < 1e-9 || c < 1e-9) {
    return 0.0;
  }
  const double area = std::sqrt(area_sq);
  return 4.0 * area / (a * b * c);
}

std::vector<double> TimeParameterizer::forwardPass(
  const std::vector<std::pair<double, Point>> & arc_table, double v0, double v_max,
  double a_max) const
{
  const int n = static_cast<int>(arc_table.size());
  std::vector<double> speeds(n, 0.0);
  speeds[0] = std::min(v0, v_max);

  for (int i = 1; i < n; ++i) {
    const double ds = arc_table[i].first - arc_table[i - 1].first;

    // 曲率による速度制限
    double v_curv = v_max;
    if (i > 0 && i < n - 1) {
      const double kappa =
        computeCurvature(arc_table[i - 1].second, arc_table[i].second, arc_table[i + 1].second);
      if (kappa > 1e-6) {
        // 求心加速度制限: a_c = v^2 * kappa <= a_max
        // -> v <= sqrt(a_max / kappa)
        v_curv = std::sqrt(a_max / kappa);
      }
    }

    // 動力学制限: v^2 = v_prev^2 + 2*a_max*ds
    const double v_kine = std::sqrt(speeds[i - 1] * speeds[i - 1] + 2.0 * a_max * ds);
    speeds[i] = std::min({v_max, v_curv, v_kine});
  }

  return speeds;
}

std::vector<double> TimeParameterizer::backwardPass(
  const std::vector<std::pair<double, Point>> & arc_table, double vf, double v_max,
  double a_brk) const
{
  const int n = static_cast<int>(arc_table.size());
  std::vector<double> speeds(n, 0.0);
  speeds[n - 1] = std::min(vf, v_max);

  for (int i = n - 2; i >= 0; --i) {
    const double ds = arc_table[i + 1].first - arc_table[i].first;

    // 曲率による速度制限
    double v_curv = v_max;
    if (i > 0 && i < n - 1) {
      const double kappa =
        computeCurvature(arc_table[i - 1].second, arc_table[i].second, arc_table[i + 1].second);
      if (kappa > 1e-6) {
        v_curv = std::sqrt(a_brk / kappa);
      }
    }

    const double v_kine = std::sqrt(speeds[i + 1] * speeds[i + 1] + 2.0 * a_brk * ds);
    speeds[i] = std::min({v_max, v_curv, v_kine});
  }

  return speeds;
}

TimeOptimalTrajectory TimeParameterizer::buildTrajectory(
  const std::vector<std::pair<double, Point>> & arc_table, const std::vector<double> & speeds) const
{
  const int n = static_cast<int>(arc_table.size());
  TimeOptimalTrajectory traj;
  traj.points.reserve(n);

  double t = 0.0;
  for (int i = 0; i < n; ++i) {
    TimePoint tp;
    tp.t = t;
    tp.pos = arc_table[i].second;
    tp.vel = Vector2::Zero();

    if (i < n - 1) {
      // 接線方向を計算
      const Vector2 dir = (arc_table[i + 1].second - arc_table[i].second);
      const double dir_norm = dir.norm();
      const Vector2 tangent = (dir_norm > 1e-9) ? dir / dir_norm : Vector2(1.0, 0.0);
      tp.vel = tangent * speeds[i];

      // 時間積分: dt = 2*ds / (v[i] + v[i+1])
      const double ds = arc_table[i + 1].first - arc_table[i].first;
      const double v_avg = 0.5 * (speeds[i] + speeds[i + 1]);
      const double dt = (v_avg > 1e-9) ? ds / v_avg : ds / 0.01;
      t += dt;
    } else {
      // 終端点
      const Vector2 dir = (arc_table[n - 1].second - arc_table[n - 2].second);
      const double dir_norm = dir.norm();
      const Vector2 tangent = (dir_norm > 1e-9) ? dir / dir_norm : Vector2(1.0, 0.0);
      tp.vel = tangent * speeds[n - 1];
    }

    traj.points.push_back(tp);
  }

  traj.total_time = t;
  return traj;
}

TimeOptimalTrajectory TimeParameterizer::parameterize(
  const ElasticBand & band, const Vector2 & initial_velocity, const Vector2 & terminal_velocity,
  double max_vel, double max_acc, double max_brk) const
{
  if (!band.isValid()) {
    return TimeOptimalTrajectory{};
  }

  const double v_max = (max_vel > 0.0) ? max_vel : config_.max_velocity;
  const double a_max = (max_acc > 0.0) ? max_acc : config_.max_acceleration;
  const double a_brk = (max_brk > 0.0) ? max_brk : config_.max_deceleration;

  const auto arc_table = computeArcLengthTable(band);
  if (arc_table.size() < 2) {
    return TimeOptimalTrajectory{};
  }

  // 初速・終速の大きさ（速度の方向は弾性バンドの接線方向が決定する）
  const double v0 = initial_velocity.norm();
  const double vf = terminal_velocity.norm();

  const auto forward = forwardPass(arc_table, v0, v_max, a_max);
  const auto backward = backwardPass(arc_table, vf, v_max, a_brk);

  const int n = static_cast<int>(arc_table.size());
  std::vector<double> speeds(n);
  for (int i = 0; i < n; ++i) {
    speeds[i] = std::min(forward[i], backward[i]);
  }

  return buildTrajectory(arc_table, speeds);
}

}  // namespace crane::ateb
