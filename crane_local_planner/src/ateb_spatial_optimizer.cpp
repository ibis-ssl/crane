// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/ateb_spatial_optimizer.hpp"

#include <algorithm>
#include <cmath>

namespace crane::ateb
{

ElasticBand SpatialOptimizer::initializeBand(
  const HomotopyClass & homotopy, const Point & start, const Point & goal) const
{
  ElasticBand band;
  const int n = config_.band_node_count;

  // ホモトピー経路をn個のノードにリサンプル
  // まずホモトピー経路の弧長テーブルを構築
  const auto & waypoints = homotopy.waypoints;
  if (waypoints.empty()) {
    // フォールバック: 直線
    band.nodes.resize(n);
    for (int i = 0; i < n; ++i) {
      const double alpha = static_cast<double>(i) / (n - 1);
      band.nodes[i].pos = start * (1.0 - alpha) + goal * alpha;
      band.nodes[i].fixed = (i == 0 || i == n - 1);
    }
    return band;
  }

  // 弧長テーブル
  std::vector<double> arc;
  arc.reserve(waypoints.size());
  arc.push_back(0.0);
  for (size_t i = 1; i < waypoints.size(); ++i) {
    arc.push_back(arc.back() + (waypoints[i] - waypoints[i - 1]).norm());
  }
  const double total_len = arc.back();

  band.nodes.resize(n);
  for (int i = 0; i < n; ++i) {
    const double target_s = total_len * i / (n - 1);
    // 線形補間で対応するウェイポイントを計算
    auto it = std::lower_bound(arc.begin(), arc.end(), target_s);
    if (it == arc.begin()) {
      band.nodes[i].pos = waypoints.front();
    } else if (it == arc.end()) {
      band.nodes[i].pos = waypoints.back();
    } else {
      const size_t idx = static_cast<size_t>(std::distance(arc.begin(), it));
      const double ds = arc[idx] - arc[idx - 1];
      const double alpha = (ds > 1e-9) ? (target_s - arc[idx - 1]) / ds : 0.0;
      band.nodes[i].pos = waypoints[idx - 1] * (1.0 - alpha) + waypoints[idx] * alpha;
    }
    band.nodes[i].fixed = (i == 0 || i == n - 1);
  }

  // 始点・終点を正確にセット
  band.nodes.front().pos = start;
  band.nodes.back().pos = goal;

  return band;
}

void SpatialOptimizer::buildResidualAndJacobian(
  const ElasticBand & band, const std::vector<Obstacle> & obstacles, Eigen::VectorXd & residuals,
  Eigen::MatrixXd & jacobian) const
{
  const int n = static_cast<int>(band.nodes.size());
  const int n_free = n - 2;       // start/goal固定
  const int n_vars = n_free * 2;  // 2D位置

  const int n_smooth = 2 * n_free;
  // 障害物残差の上限（実際の違反数はこれ以下）
  const int n_obs_max = n_free * static_cast<int>(obstacles.size());
  const int n_len = 2 * (n - 1);

  const int n_res_max = n_smooth + n_obs_max + n_len;
  residuals.resize(n_res_max);
  residuals.setZero();
  jacobian.resize(n_res_max, n_vars);
  jacobian.setZero();

  int row = 0;

  // 1. 滑らかさ残差: w_s * (node[i-1] + node[i+1] - 2*node[i])
  for (int i = 1; i < n - 1; ++i) {
    const int fi = i - 1;  // 自由変数インデックス (0-indexed)
    const Vector2 res_vec =
      config_.smoothness_weight *
      (band.nodes[i - 1].pos + band.nodes[i + 1].pos - 2.0 * band.nodes[i].pos);

    residuals[row] = res_vec.x();
    residuals[row + 1] = res_vec.y();

    // ヤコビアン: d(res)/d(node[i]) = -2*w_s * I
    jacobian(row, fi * 2) = -2.0 * config_.smoothness_weight;
    jacobian(row + 1, fi * 2 + 1) = -2.0 * config_.smoothness_weight;

    // ヤコビアン: d(res)/d(node[i-1]) = w_s * I (if free)
    if (i - 1 >= 1 && i - 1 <= n - 2) {
      const int fi_prev = (i - 1) - 1;
      jacobian(row, fi_prev * 2) = config_.smoothness_weight;
      jacobian(row + 1, fi_prev * 2 + 1) = config_.smoothness_weight;
    }

    // ヤコビアン: d(res)/d(node[i+1]) = w_s * I (if free)
    if (i + 1 >= 1 && i + 1 <= n - 2) {
      const int fi_next = (i + 1) - 1;
      jacobian(row, fi_next * 2) = config_.smoothness_weight;
      jacobian(row + 1, fi_next * 2 + 1) = config_.smoothness_weight;
    }

    row += 2;
  }

  // 2. 障害物残差: w_o * max(0, d_min - dist(node, obs))
  for (int i = 1; i < n - 1; ++i) {
    const int fi = i - 1;
    for (const auto & obs : obstacles) {
      const double d = obs.distance(band.nodes[i].pos);
      const double d_min = config_.obstacle_safety_margin;
      if (d < d_min) {
        // 残差: w_o * (d_min - d)
        residuals[row] = config_.obstacle_weight * (d_min - d);

        // ヤコビアン: d(res)/d(node[i]) = -w_o * grad_d
        const Vector2 grad = obs.distanceGradient(band.nodes[i].pos);
        jacobian(row, fi * 2) = -config_.obstacle_weight * grad.x();
        jacobian(row, fi * 2 + 1) = -config_.obstacle_weight * grad.y();

        ++row;
      }
    }
  }

  // 3. 経路長残差: w_l * (node[i+1] - node[i])
  for (int i = 0; i < n - 1; ++i) {
    const Vector2 seg = band.nodes[i + 1].pos - band.nodes[i].pos;
    residuals[row] = config_.path_length_weight * seg.x();
    residuals[row + 1] = config_.path_length_weight * seg.y();

    // ヤコビアン
    if (i >= 1 && i <= n - 2) {
      const int fi = i - 1;
      jacobian(row, fi * 2) = -config_.path_length_weight;
      jacobian(row + 1, fi * 2 + 1) = -config_.path_length_weight;
    }
    if (i + 1 >= 1 && i + 1 <= n - 2) {
      const int fi_next = (i + 1) - 1;
      jacobian(row, fi_next * 2) = config_.path_length_weight;
      jacobian(row + 1, fi_next * 2 + 1) = config_.path_length_weight;
    }

    row += 2;
  }

  // 実際の残差数にリサイズ（障害物違反が上限未満の場合）
  residuals.conservativeResize(row);
  jacobian.conservativeResize(row, n_vars);
}

double SpatialOptimizer::optimizeStep(
  ElasticBand & band, const std::vector<Obstacle> & obstacles) const
{
  Eigen::VectorXd residuals;
  Eigen::MatrixXd jacobian;
  buildResidualAndJacobian(band, obstacles, residuals, jacobian);

  if (jacobian.rows() == 0 || jacobian.cols() == 0) {
    return 0.0;
  }

  // Normal equations: (J^T J + lambda * I) * delta = -J^T * r
  const int n_vars = jacobian.cols();
  const Eigen::MatrixXd JtJ = jacobian.transpose() * jacobian;
  const Eigen::MatrixXd A = JtJ + config_.lma_damping * Eigen::MatrixXd::Identity(n_vars, n_vars);
  const Eigen::VectorXd b = -jacobian.transpose() * residuals;

  const Eigen::VectorXd delta = A.ldlt().solve(b);

  // ノード位置を更新（固定ノードはスキップ）
  double max_move = 0.0;
  const int n = static_cast<int>(band.nodes.size());
  for (int i = 1; i < n - 1; ++i) {
    const int fi = i - 1;
    const Vector2 move(delta[fi * 2], delta[fi * 2 + 1]);
    band.nodes[i].pos += move;
    max_move = std::max(max_move, move.norm());
  }
  return max_move;
}

ElasticBand SpatialOptimizer::optimize(
  const HomotopyClass & homotopy, const Point & start, const Point & goal,
  const std::vector<Obstacle> & obstacles) const
{
  ElasticBand band = initializeBand(homotopy, start, goal);

  for (int iter = 0; iter < config_.max_iterations; ++iter) {
    const double max_move = optimizeStep(band, obstacles);
    if (max_move < config_.convergence_threshold) {
      break;
    }
  }

  // コストを計算（経路長）
  band.total_cost = 0.0;
  for (size_t i = 1; i < band.nodes.size(); ++i) {
    band.total_cost += (band.nodes[i].pos - band.nodes[i - 1].pos).norm();
  }

  return band;
}

ElasticBand SpatialOptimizer::reoptimize(
  const ElasticBand & initial_band, const std::vector<Obstacle> & obstacles) const
{
  ElasticBand band = initial_band;

  for (int iter = 0; iter < config_.max_iterations; ++iter) {
    const double max_move = optimizeStep(band, obstacles);
    if (max_move < config_.convergence_threshold) {
      break;
    }
  }

  band.total_cost = 0.0;
  for (size_t i = 1; i < band.nodes.size(); ++i) {
    band.total_cost += (band.nodes[i].pos - band.nodes[i - 1].pos).norm();
  }

  return band;
}

}  // namespace crane::ateb
