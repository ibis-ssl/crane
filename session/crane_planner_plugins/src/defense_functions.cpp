// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/defense_functions.hpp>

namespace crane
{
auto getPenaltyAreaCorners(
  double offset_x, double offset_y, const WorldModelWrapper::SharedPtr & world_model)
  -> std::tuple<Point, Point, Point, Point>
{
  // デフェンスエリアを囲みし4つの点
  Point p1;
  p1 << world_model->goal.x(), world_model->penalty_area_size.y() * 0.5 + offset_y;
  Point p2 = p1;
  if (world_model->goal.x() > 0) {
    p2.x() -= (world_model->penalty_area_size.x() + offset_x);
  } else {
    p2.x() += (world_model->penalty_area_size.x() + offset_x);
  }
  Point p3(p2.x(), -p2.y());
  Point p4(p1.x(), p3.y());
  return {p1, p2, p3, p4};
}

auto getDefenseLinePointParameterThresholds(
  double offset_x, double offset_y, const WorldModelWrapper::SharedPtr & world_model)
  -> std::tuple<double, double, double>
{
  const double threshold1 = world_model->penalty_area_size.x() + offset_x;
  // p2 -> p3: world_model->penalty_area_size.y() + OFFSET_Y * 2
  const double threshold2 = world_model->penalty_area_size.y() + offset_y * 2 + threshold1;
  // p3 -> p4: world_model->penalty_area_size.x() + OFFSET_X
  const double threshold3 = world_model->penalty_area_size.x() + offset_x + threshold2;
  return {threshold1, threshold2, threshold3};
}

auto getDefenseLinePoint(double parameter, const WorldModelWrapper::SharedPtr & world_model)
  -> Point
{
  const double OFFSET_X = 0.1;
  const double OFFSET_Y = 0.1;
  auto [p1, p2, p3, p4] = getPenaltyAreaCorners(OFFSET_X, OFFSET_Y, world_model);

  const auto [threshold1, threshold2, threshold3] =
    getDefenseLinePointParameterThresholds(OFFSET_X, OFFSET_Y, world_model);

  if (parameter >= 0. && parameter < threshold1) {
    return p1 + (p2 - p1).normalized() * parameter;
  } else if (parameter < threshold2) {
    return p2 + (p3 - p2).normalized() * (parameter - threshold1);
  } else if (parameter < threshold3) {
    return p3 + (p4 - p3).normalized() * (parameter - threshold2);
  } else {
    std::stringstream what;
    what << "Invalid parameter range for DefenderPlanner::getDefenseLinePoint: " << parameter;
    what << "with thresholds: " << threshold1 << ", " << threshold2 << ", " << threshold3;
    throw std::runtime_error(what.str());
  }
}

auto getDefenseLinePointParameter(
  const Segment & target_segment, const WorldModelWrapper::SharedPtr & world_model)
  -> std::optional<double>
{
  const double OFFSET_X = 0.1;
  const double OFFSET_Y = 0.1;
  auto [p1, p2, p3, p4] = getPenaltyAreaCorners(OFFSET_X, OFFSET_Y, world_model);

  const double threshold1 = world_model->penalty_area_size.x() + OFFSET_X;
  // p2 -> p3: world_model->penalty_area_size.y() + OFFSET_Y * 2
  const double threshold2 = world_model->penalty_area_size.y() + OFFSET_Y * 2 + threshold1;

  std::vector<Point> intersections;

  if (intersections = getIntersections(Segment{p1, p2}, target_segment);
      not intersections.empty()) {
    return std::abs(intersections[0].x() - p1.x());
  } else if (intersections = getIntersections(Segment{p2, p3}, target_segment);
             not intersections.empty()) {
    return std::abs(intersections[0].y() - p2.y()) + threshold1;
  } else if (intersections = getIntersections(Segment{p3, p4}, target_segment);
             not intersections.empty()) {
    return std::abs(intersections[0].x() - p3.x()) + threshold2;
  } else {
    return std::nullopt;
  }
}
}  // namespace crane
