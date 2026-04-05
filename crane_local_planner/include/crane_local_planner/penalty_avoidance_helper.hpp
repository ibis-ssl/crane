// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__PENALTY_AVOIDANCE_HELPER_HPP_
#define CRANE_LOCAL_PLANNER__PENALTY_AVOIDANCE_HELPER_HPP_

#include <algorithm>
#include <cmath>
#include <crane_geometry/boost_geometry.hpp>
#include <optional>

namespace crane
{

enum class PenaltyBypassSide { TOP, BOTTOM };

inline auto toString(PenaltyBypassSide side) -> const char *
{
  return side == PenaltyBypassSide::TOP ? "TOP" : "BOTTOM";
}

struct PenaltyBypassDecision
{
  bool crossing_detected = false;
  bool target_overridden = false;
  PenaltyBypassSide selected_side = PenaltyBypassSide::TOP;
  Point waypoint = Point::Zero();
  double top_cost = 0.0;
  double bottom_cost = 0.0;
};

inline auto intersectsSegmentAABB(const Point & a, const Point & b, const Box & box) -> bool
{
  const double xmin = std::min(box.min_corner().x(), box.max_corner().x());
  const double xmax = std::max(box.min_corner().x(), box.max_corner().x());
  const double ymin = std::min(box.min_corner().y(), box.max_corner().y());
  const double ymax = std::max(box.min_corner().y(), box.max_corner().y());

  auto inside = [&](const Point & p) {
    return xmin <= p.x() && p.x() <= xmax && ymin <= p.y() && p.y() <= ymax;
  };
  if (inside(a) || inside(b)) {
    return true;
  }

  const double dx = b.x() - a.x();
  const double dy = b.y() - a.y();
  double t0 = 0.0;
  double t1 = 1.0;
  constexpr double EPS = 1e-9;

  auto clip_axis = [&](double p, double q_min, double q_max) {
    if (std::abs(p) < EPS) {
      return q_min <= 0.0 && 0.0 <= q_max;
    }
    const double t_min = q_min / p;
    const double t_max = q_max / p;
    const double t_enter = std::min(t_min, t_max);
    const double t_exit = std::max(t_min, t_max);
    t0 = std::max(t0, t_enter);
    t1 = std::min(t1, t_exit);
    return t0 <= t1;
  };

  if (!clip_axis(dx, xmin - a.x(), xmax - a.x())) return false;
  if (!clip_axis(dy, ymin - a.y(), ymax - a.y())) return false;
  return true;
}

inline auto computePenaltyBypassDecision(
  const Point & current_pos, const Point & target_pos, const Box & penalty_area,
  const Point & goal_pos, const Point & penalty_area_size, double penalty_area_offset,
  double surrounding_offset, bool force_waypoint_on_crossing) -> PenaltyBypassDecision
{
  (void)penalty_area_size;
  PenaltyBypassDecision decision;
  Box expanded = penalty_area;
  expanded.min_corner() -= Point(penalty_area_offset, penalty_area_offset);
  expanded.max_corner() += Point(penalty_area_offset, penalty_area_offset);

  if (!intersectsSegmentAABB(current_pos, target_pos, expanded)) {
    return decision;
  }

  decision.crossing_detected = true;

  // Waypoint must be outside the expanded penalty area to avoid getting
  // re-overridden by the same crossing logic on the next cycle.
  const double bypass_x = (goal_pos.x() < 0.0) ? (expanded.max_corner().x() + surrounding_offset)
                                               : (expanded.min_corner().x() - surrounding_offset);
  const Point top_corner(bypass_x, expanded.max_corner().y() + surrounding_offset);
  const Point bottom_corner(bypass_x, expanded.min_corner().y() - surrounding_offset);

  decision.top_cost = (top_corner - current_pos).norm() + (target_pos - top_corner).norm();
  decision.bottom_cost = (bottom_corner - current_pos).norm() + (target_pos - bottom_corner).norm();

  PenaltyBypassSide selected = (decision.top_cost <= decision.bottom_cost)
                                 ? PenaltyBypassSide::TOP
                                 : PenaltyBypassSide::BOTTOM;

  // If the waypoint on the selected side would still require crossing the PA to reach the target,
  // prefer the other side (which avoids the re-crossing).
  const Point selected_wp = (selected == PenaltyBypassSide::TOP) ? top_corner : bottom_corner;
  const Point other_wp = (selected == PenaltyBypassSide::TOP) ? bottom_corner : top_corner;
  if (
    intersectsSegmentAABB(selected_wp, target_pos, expanded) &&
    !intersectsSegmentAABB(other_wp, target_pos, expanded)) {
    selected =
      (selected == PenaltyBypassSide::TOP) ? PenaltyBypassSide::BOTTOM : PenaltyBypassSide::TOP;
  }

  decision.selected_side = selected;
  decision.waypoint = selected == PenaltyBypassSide::TOP ? top_corner : bottom_corner;
  decision.target_overridden = force_waypoint_on_crossing;
  return decision;
}

}  // namespace crane

#endif  // CRANE_LOCAL_PLANNER__PENALTY_AVOIDANCE_HELPER_HPP_
