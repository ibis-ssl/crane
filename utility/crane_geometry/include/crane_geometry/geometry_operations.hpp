// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GEOMETRY__GEOMETRY_OPERATIONS_HPP_
#define CRANE_GEOMETRY__GEOMETRY_OPERATIONS_HPP_

#include <cmath>  // For std::fabs and std::sqrt
#include <crane_geometry/boost_geometry.hpp>
#include <optional>
#include <vector>

namespace crane
{
inline auto isInBox(const Box & box, const Point & p) -> bool { return bg::within(p, box); }

inline auto isInBox(Box box, const Point & p, const double offset) -> bool
{
  box.max_corner() += Point(offset, offset);
  box.min_corner() -= Point(offset, offset);
  return bg::within(p, box);
}

inline auto createBox(const Point & p1, const Point & p2) -> Box
{
  Box box;
  box.min_corner() = Point(std::min(p1.x(), p2.x()), std::min(p1.y(), p2.y()));
  box.max_corner() = Point(std::max(p1.x(), p2.x()), std::max(p1.y(), p2.y()));
  return box;
}

inline auto inflateBox(const Box & box, double margin) -> Box
{
  const auto minc = box.min_corner();
  const auto maxc = box.max_corner();
  const Point pmin(minc.x() - margin, minc.y() - margin);
  const Point pmax(maxc.x() + margin, maxc.y() + margin);
  return createBox(pmin, pmax);
}

inline auto getAngle(const Vector2 & vec) -> double { return atan2(vec.y(), vec.x()); }

inline auto normalizeAngle(double angle_rad) -> double
{
  while (angle_rad > M_PI) {
    angle_rad -= 2.0f * M_PI;
  }
  while (angle_rad < -M_PI) {
    angle_rad += 2.0f * M_PI;
  }
  return angle_rad;
}

inline auto getAngleDiff(double angle_rad1, double angle_rad2) -> double
{
  angle_rad1 = normalizeAngle(angle_rad1);
  angle_rad2 = normalizeAngle(angle_rad2);
  if (abs(angle_rad1 - angle_rad2) > M_PI) {
    if (angle_rad1 - angle_rad2 > 0) {
      return angle_rad1 - angle_rad2 - 2.0f * M_PI;
    } else {
      return angle_rad1 - angle_rad2 + 2.0f * M_PI;
    }
  } else {
    return angle_rad1 - angle_rad2;
  }
}

inline auto getAngleDiff(const Pose2D & pose1, const Pose2D & pose2) -> double
{
  return getAngleDiff(pose1.theta, pose2.theta);
}

inline auto getAngleDiff(const Pose2D & pose1, const double angle_rad) -> double
{
  return getAngleDiff(pose1.theta, angle_rad);
}

inline auto getAngleDiff(const double angle_rad, const Pose2D & pose1) -> double
{
  return getAngleDiff(angle_rad, pose1.theta);
}

inline auto getIntermediateAngle(double angle_rad1, double angle_rad2) -> double
{
  angle_rad1 = normalizeAngle(angle_rad1);
  angle_rad2 = normalizeAngle(angle_rad2);
  // 差がpiを超えている場合では平均を取るだけではダメ
  if (abs(angle_rad1 - angle_rad2) > M_PI) {
    return normalizeAngle((angle_rad1 + angle_rad2 + 2.0f * M_PI) / 2.0f);
  } else {
    return (angle_rad1 + angle_rad2) / 2.0f;
  }
}

inline auto getNormVec(const double angle) -> Vector2 { return {cos(angle), sin(angle)}; }

inline auto getVerticalVec(const Point & v) -> Point
{
  Point vertical_v;
  vertical_v.x() = v.y();   // Corrected syntax
  vertical_v.y() = -v.x();  // Corrected syntax
  return vertical_v;
}

inline auto getReachTime(
  const double distance, const double v0, const double acc, const double max_vel) -> double
{
  // x = v0*t + 1/2*a*t^2 より
  double t = (sqrt(v0 * v0 + 2.0f * acc * distance) - v0) / acc;
  if (max_vel == -1.f) {
    return t;
  } else {
    double acc_end_time = (max_vel - v0) / acc;
    if (t > acc_end_time) {
      return (distance + 0.5f * std::pow(max_vel - v0, 2.f) / acc) / max_vel;
    } else {
      return t;
    }
  }
}

// 始端/終端速度を考慮した到達時間（台形/三角プロファイル）
inline auto getReachTime(
  double distance, double v_in, double v_out, double alpha_acc, double alpha_dec, double max_vel)
  -> double
{
  const double a = std::max(1e-6, alpha_acc);
  const double b = std::max(1e-6, alpha_dec);
  const double L = std::max(0.0, distance);
  v_in = std::max(0.0, v_in);
  v_out = std::max(0.0, v_out);
  const double vm = std::max(1e-6, max_vel);

  const double num = 2.0 * a * b * L + b * v_in * v_in + a * v_out * v_out;
  const double den = a + b;
  const double v_peak = std::sqrt(std::max(0.0, num / den));

  if (v_peak <= vm + 1e-9) {
    const double t_acc = std::max(0.0, (v_peak - v_in) / a);
    const double t_dec = std::max(0.0, (v_peak - v_out) / b);
    return t_acc + t_dec;
  } else {
    const double s_acc = std::max(0.0, (vm * vm - v_in * v_in) / (2.0 * a));
    const double s_dec = std::max(0.0, (vm * vm - v_out * v_out) / (2.0 * b));
    const double s_cruise = std::max(0.0, L - s_acc - s_dec);
    const double t_acc = std::max(0.0, (vm - v_in) / a);
    const double t_dec = std::max(0.0, (vm - v_out) / b);
    const double t_cruise = s_cruise / vm;
    return t_acc + t_cruise + t_dec;
  }
}

inline auto getIntersections(const Segment & segment1, const Segment & segment2)
  -> std::vector<Point>
{
  std::vector<Point> intersections;
  bg::intersection(segment1, segment2, intersections);
  return intersections;
}

inline auto getIntersections(const Circle & circle, const Segment & segment) -> std::vector<Point>
{
  std::vector<Point> intersections;
  double distance = bg::distance(circle, segment);
  if (distance > circle.radius) {
    // 交差しない
    return intersections;
  } else {
    // 交差する
    // 交点を求める
    Vector2 norm_vec = getVerticalVec(segment.second - segment.first).normalized();
    if (
      ((circle.center + norm_vec) - segment.first).norm() >
      ((circle.center - norm_vec) - segment.first).norm()) {
      norm_vec = -norm_vec;
    }
    double d = std::sqrt(circle.radius * circle.radius - distance * distance);
    Vector2 seg_norm = (segment.second - segment.first).normalized();
    Point p1 = circle.center + norm_vec * distance + seg_norm * d;
    Point p2 = circle.center + norm_vec * distance - seg_norm * d;

    // 交点が線分上にあるか確認
    if (
      (p1 - segment.first).dot(segment.second - segment.first) > 0 &&
      (p1 - segment.second).dot(segment.first - segment.second) > 0) {
      intersections.push_back(p1);
    }

    if (
      (p2 - segment.first).dot(segment.second - segment.first) > 0 &&
      (p2 - segment.second).dot(segment.first - segment.second) > 0) {
      intersections.push_back(p2);
    }

    return intersections;
  }
}

template <typename Geometry1, typename Geometry2>
inline auto getIntersections(const Geometry1 & geometry1, const Geometry2 & geometry2)
  -> std::vector<Point>
{
  std::vector<Point> intersections;
  bg::intersection(geometry1, geometry2, intersections);
  return intersections;
}

template <typename Geometry1, typename Geometry2>
inline auto getClosestPointAndDistance(const Geometry1 & geometry1, const Geometry2 & geometry2)
  -> ClosestPoint
{
  ClosestPoint result;
  bg::closest_point(geometry1, geometry2, result);
  return result;
}

inline auto polylineLength(const std::vector<Point> & points) -> double
{
  if (points.size() < 2) return 0.0;
  double sum = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    sum += (points[i] - points[i - 1]).norm();
  }
  return sum;
}

inline auto getCircle(const Point & p1, const Point & p2, const Point & p3) -> std::optional<Circle>
{
  // Using the formula from https://en.wikipedia.org/wiki/Circumscribed_circle#Cartesian_coordinates_2
  double D =
    2 * (p1.x() * (p2.y() - p3.y()) + p2.x() * (p3.y() - p1.y()) + p3.x() * (p1.y() - p2.y()));

  if (std::fabs(D) < 1e-9) {  // Points are collinear
    return std::nullopt;
  }

  Circle circle;
  double p1_sq = p1.x() * p1.x() + p1.y() * p1.y();
  double p2_sq = p2.x() * p2.x() + p2.y() * p2.y();
  double p3_sq = p3.x() * p3.x() + p3.y() * p3.y();

  circle.center.x() =
    (p1_sq * (p2.y() - p3.y()) + p2_sq * (p3.y() - p1.y()) + p3_sq * (p1.y() - p2.y())) /
    D;  // Corrected syntax
  circle.center.y() =
    (p1_sq * (p3.x() - p2.x()) + p2_sq * (p1.x() - p3.x()) + p3_sq * (p2.x() - p1.x())) /
    D;  // Corrected syntax

  circle.radius = std::sqrt(
    (circle.center.x() - p1.x()) * (circle.center.x() - p1.x()) +
    (circle.center.y() - p1.y()) * (circle.center.y() - p1.y()));
  return circle;
}

inline auto getSeparatedPoints(const Segment & segment1, int separated_num) -> std::vector<Point>
{
  std::vector<Point> points;
  Vector2 segment_vec = (segment1.second - segment1.first).normalized();
  for (int i = 0; i < separated_num - 1; ++i) {
    points.push_back(
      segment1.first + segment_vec * (i + 1) / static_cast<double>(separated_num + 1));
  }
  return points;
}
}  // namespace crane

#endif  // CRANE_GEOMETRY__GEOMETRY_OPERATIONS_HPP_
