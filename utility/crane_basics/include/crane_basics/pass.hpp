// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__PASS_HPP_
#define CRANE_BASICS__PASS_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_physics/robot_info.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/view/transform.hpp>

namespace crane
{
namespace pass
{
struct Analysis
{
  bool need_chip = false;
  double required_chip_distance = 0.0;
};

struct ClosestInfo
{
  ClosestPoint closest_point;
  double ball_to_close_point;
  RobotInfo::SharedPtr closest_robot;
};
}  // namespace pass

inline auto getPassAnalysis(
  const Point & ball, const Point & target, std::vector<RobotInfo::SharedPtr> their_robots,
  const double block_distance = 0.2) -> pass::Analysis
{
  if (their_robots.empty()) {
    return pass::Analysis();
  }

  Segment ball_segment(ball, target);

  auto closest_info_array =
    their_robots | ranges::views::transform([&](const auto & robot) {
      pass::ClosestInfo closest;
      closest.closest_point = getClosestPointAndDistance(robot->pose.pos, ball_segment);
      closest.ball_to_close_point = (closest.closest_point.closest_point - ball).norm();
      closest.closest_robot = robot;
      return closest;
    }) |
    ranges::to<std::vector>();

  // 大きい -> 小さい
  ranges::sort(closest_info_array, [](const auto & a, const auto & b) {
    return a.ball_to_close_point > b.ball_to_close_point;
  });

  for (const auto & closest : closest_info_array) {
    if (closest.closest_point.distance < block_distance) {
      pass::Analysis analysis;
      analysis.need_chip = true;
      analysis.required_chip_distance = closest.ball_to_close_point;
      return analysis;
    }
  }
  return pass::Analysis();
}
}  // namespace crane

#endif  // CRANE_BASICS__PASS_HPP_
