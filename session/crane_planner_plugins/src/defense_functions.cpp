// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/defense_functions.hpp>

namespace crane
{
auto getDefenseLinePointParameterThresholds(
  double offset_x, double offset_y, const WorldModelWrapper::SharedPtr & world_model)
  -> std::tuple<double, double, double>
{
  const double threshold1 = world_model->penalty_area_size.x() + offset_x + 0.5;
  // p2 -> p3: world_model->penalty_area_size.y() + OFFSET_Y * 2
  const double threshold2 = world_model->penalty_area_size.y() + offset_y * 2 + threshold1;
  // p3 -> p4: world_model->penalty_area_size.x() + OFFSET_X
  const double threshold3 = world_model->penalty_area_size.x() + offset_x + threshold2 + 0.5;
  return {threshold1, threshold2, threshold3};
}

auto getDefenseLinePoint(double parameter, const WorldModelWrapper::SharedPtr & world_model)
  -> Point
{
  const double OFFSET_X = 0.1;
  const double OFFSET_Y = 0.1;
  auto [p1, p2, p3, p4] = world_model->getPenaltyAreaCorners(OFFSET_X, OFFSET_Y);

  const auto [threshold1, threshold2, threshold3] =
    getDefenseLinePointParameterThresholds(OFFSET_X, OFFSET_Y, world_model);

  if (parameter >= 0. && parameter < threshold1) {
    return p1 + (p2 - p1).normalized() * parameter;
  } else if (parameter < threshold2) {
    return p2 + (p3 - p2).normalized() * (parameter - threshold1);
  } else if (parameter < threshold3) {
    return p3 + (p4 - p3).normalized() * (parameter - threshold2);
  } else {
    if (parameter < 0.) {
      return p1 + (p2 - p1).normalized() * parameter;
    } else if (parameter > threshold3) {
      return p3 + (p4 - p3).normalized() * (parameter - threshold2);
    } else {
      std::stringstream what;
      what << "Invalid parameter range for getDefenseLinePoint: " << parameter;
      what << "with thresholds: " << threshold1 << ", " << threshold2 << ", " << threshold3;
      throw std::runtime_error(what.str());
    }
  }
}

auto getDefenseLinePointParameter(
  const Segment & target_segment, const WorldModelWrapper::SharedPtr & world_model)
  -> std::optional<double>
{
  const double OFFSET_X = 0.1;
  const double OFFSET_Y = 0.1;
  auto [p1, p2, p3, p4] = world_model->getPenaltyAreaCorners(OFFSET_X, OFFSET_Y);

  const double threshold1 = world_model->penalty_area_size.x() + OFFSET_X + 0.5;
  // p2 -> p3: world_model->penalty_area_size.y() + OFFSET_Y * 2
  const double threshold2 = world_model->penalty_area_size.y() + OFFSET_Y * 2 + threshold1;

  if (auto intersections = getIntersections(Segment{p1, p2}, target_segment);
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

auto getForwardDefenseRatio(
  const Segment & ball_line, const WorldModelWrapper::SharedPtr & world_model)
  -> std::optional<double>
{
  const Vector2 segment_vec = (ball_line.second - ball_line.first).normalized();
  const auto ball_line_long_behind = Segment(ball_line.first - segment_vec * 20, ball_line.second);
  const auto ball_line_long_forward = Segment(ball_line.first, ball_line.second + segment_vec * 20);

  auto get_intersection_to_area =
    [](
      const Segment & target_segment,
      std::tuple<const Point &, const Point &, const Point &, const Point &> areas)
    -> std::optional<Point> {
    if (auto intersections =
          getIntersections(Segment{std::get<0>(areas), std::get<1>(areas)}, target_segment);
        not intersections.empty()) {
      return intersections[0];
    } else if (intersections =
                 getIntersections(Segment{std::get<1>(areas), std::get<2>(areas)}, target_segment);
               not intersections.empty()) {
      return intersections[0];
    } else if (intersections =
                 getIntersections(Segment{std::get<2>(areas), std::get<3>(areas)}, target_segment);
               not intersections.empty()) {
      return intersections[0];
    } else {
      return std::nullopt;
    }
  };

  auto [our_penalty_area_1, our_penalty_area_2, our_penalty_area_3, our_penalty_area_4] =
    world_model->getPenaltyAreaCorners(0.0, 0.0);
  const auto intersect_to_penalty_area = get_intersection_to_area(
    ball_line_long_forward,
    std::make_tuple(
      our_penalty_area_1, our_penalty_area_2, our_penalty_area_3, our_penalty_area_4));
  if (not intersect_to_penalty_area) {
    return std::nullopt;
  }

  auto [our_area_p1, our_area_p2, our_area_p3, our_area_p4] = world_model->getOurAreaCorners();
  const auto intersect_to_field_area = get_intersection_to_area(
    ball_line_long_behind, std::make_tuple(our_area_p1, our_area_p2, our_area_p3, our_area_p4));
  if (not intersect_to_field_area) {
    return std::nullopt;
  }

  double distance_ball_to_penalty_area =
    bg::distance(world_model->ball.pos, intersect_to_penalty_area.value());
  double distance_ball_to_field_area =
    bg::distance(world_model->ball.pos, intersect_to_field_area.value());
  double distance_sum = distance_ball_to_penalty_area + distance_ball_to_field_area;

  // ボールからペナルティエリアまでの距離が小さいほど大きな値が返る。
  return distance_ball_to_field_area / distance_sum;
}  // namespace crane
}  // namespace crane
