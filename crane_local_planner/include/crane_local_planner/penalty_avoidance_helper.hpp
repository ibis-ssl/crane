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
  bool used_locked_side = false;
  PenaltyBypassSide selected_side = PenaltyBypassSide::TOP;
  Point waypoint = Point::Zero();
  double top_cost = 0.0;
  double bottom_cost = 0.0;
};

inline auto computePenaltyBypassDecision(
  const Point & current_pos, const Point & target_pos, const Box & penalty_area,
  const Point & goal_pos, const Point & penalty_area_size, double penalty_area_offset,
  double surrounding_offset, const std::optional<PenaltyBypassSide> & locked_side,
  double side_switch_margin, bool force_waypoint_on_crossing) -> PenaltyBypassDecision
{
  PenaltyBypassDecision decision;
  Box expanded = penalty_area;
  expanded.min_corner() -= Point(penalty_area_offset, penalty_area_offset);
  expanded.max_corner() += Point(penalty_area_offset, penalty_area_offset);

  const Segment move_line(current_pos, target_pos);
  if (!bg::intersects(move_line, expanded)) {
    return decision;
  }

  decision.crossing_detected = true;

  const double x_offset = std::copysign(penalty_area_size.x() + surrounding_offset, -goal_pos.x());
  const double half_height = penalty_area_size.y() * 0.5 + surrounding_offset;
  const Point top_corner = goal_pos + Point(x_offset, half_height);
  const Point bottom_corner = goal_pos + Point(x_offset, -half_height);

  decision.top_cost = (top_corner - current_pos).norm() + (target_pos - top_corner).norm();
  decision.bottom_cost = (bottom_corner - current_pos).norm() + (target_pos - bottom_corner).norm();

  PenaltyBypassSide selected = (decision.top_cost <= decision.bottom_cost)
                                 ? PenaltyBypassSide::TOP
                                 : PenaltyBypassSide::BOTTOM;
  if (locked_side.has_value()) {
    const double alternative_margin = std::abs(decision.top_cost - decision.bottom_cost);
    if (alternative_margin < side_switch_margin) {
      selected = locked_side.value();
      decision.used_locked_side = true;
    }
  }
  decision.selected_side = selected;
  decision.waypoint = selected == PenaltyBypassSide::TOP ? top_corner : bottom_corner;
  decision.target_overridden = force_waypoint_on_crossing;
  return decision;
}

}  // namespace crane

#endif  // CRANE_LOCAL_PLANNER__PENALTY_AVOIDANCE_HELPER_HPP_
