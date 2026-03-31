// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/ateb_cbf_filter.hpp"

#include <algorithm>
#include <cmath>

namespace crane::ateb
{

std::vector<CBFFilter::HalfPlaneConstraint> CBFFilter::buildConstraints(
  const Point & ego_pos, const std::vector<std::pair<Point, Vector2>> & dynamic_obstacles,
  const std::vector<Obstacle> & static_obstacles) const
{
  std::vector<HalfPlaneConstraint> constraints;
  constraints.reserve(dynamic_obstacles.size() + static_obstacles.size());

  // 動的障害物（他ロボット）
  for (const auto & [obs_pos, obs_vel] : dynamic_obstacles) {
    const Vector2 diff = ego_pos - obs_pos;
    const double dist_sq = diff.squaredNorm();
    const double safe_dist = config_.robot_radius * 2.0 + config_.safety_margin;
    const double safe_dist_sq = safe_dist * safe_dist;

    // バリア関数: h = ||p_ego - p_j||^2 - safe_dist^2
    const double h = dist_sq - safe_dist_sq;

    // CBF制約: 2*(p_ego - p_j)^T * u >= -alpha*h + 2*(p_ego - p_j)^T * v_j
    const Vector2 normal = 2.0 * diff;
    const double bound = -config_.alpha * h + 2.0 * diff.dot(obs_vel);

    constraints.push_back({normal, bound});
  }

  // 静的障害物
  for (const auto & obs : static_obstacles) {
    const double dist = obs.distance(ego_pos);
    const double safe_dist = config_.robot_radius + config_.safety_margin;

    // バリア関数: h = dist - safe_dist
    const double h = dist - safe_dist;

    // 勾配方向（外向き法線）
    const Vector2 grad = obs.distanceGradient(ego_pos);

    // CBF制約: grad^T * u >= -alpha * h
    constraints.push_back({grad, -config_.alpha * h});
  }

  return constraints;
}

Vector2 CBFFilter::solveProjectionQP(
  const Vector2 & u_nom, const std::vector<HalfPlaneConstraint> & constraints) const
{
  Vector2 u = u_nom;

  for (int iter = 0; iter < config_.max_iterations; ++iter) {
    bool all_satisfied = true;

    for (const auto & constraint : constraints) {
      const double lhs = constraint.normal.dot(u);
      if (lhs < constraint.bound) {
        all_satisfied = false;
        // u を制約超平面上に射影する
        // 最小修正: u += lambda * normal
        // lambda = (bound - normal^T*u) / ||normal||^2
        const double normal_sq = constraint.normal.squaredNorm();
        if (normal_sq < 1e-12) continue;
        const double lambda = (constraint.bound - lhs) / normal_sq;
        u += lambda * constraint.normal;
      }
    }

    if (all_satisfied) break;
  }

  // 最大修正量でクランプ
  const Vector2 correction = u - u_nom;
  const double correction_norm = correction.norm();
  if (correction_norm > config_.max_correction) {
    u = u_nom + correction * (config_.max_correction / correction_norm);
  }

  return u;
}

Vector2 CBFFilter::filter(
  const Point & ego_pos, const Vector2 & u_nominal,
  const std::vector<std::pair<Point, Vector2>> & dynamic_obstacles,
  const std::vector<Obstacle> & static_obstacles) const
{
  const auto constraints = buildConstraints(ego_pos, dynamic_obstacles, static_obstacles);

  if (constraints.empty()) {
    return u_nominal;
  }

  return solveProjectionQP(u_nominal, constraints);
}

}  // namespace crane::ateb
