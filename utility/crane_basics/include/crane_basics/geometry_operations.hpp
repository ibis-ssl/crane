// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__GEOMETRY_OPERATIONS_HPP_
#define CRANE_BASICS__GEOMETRY_OPERATIONS_HPP_

#include <vector>

#include "boost_geometry.hpp"

namespace crane
{
inline auto isInBox(const Box & box, const Point & p) -> bool { return bg::within(p, box); }

inline auto isInBox(Box box, const Point & p, const double offset) -> bool
{
  box.max_corner() += Point(offset, offset);
  box.min_corner() -= Point(offset, offset);
  return bg::within(p, box);
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

inline auto getAngleDiff(Pose2D pose1, Pose2D pose2) -> double
{
  return getAngleDiff(pose1.theta, pose2.theta);
}

inline auto getAngleDiff(Pose2D pose1, double angle_rad) -> double
{
  return getAngleDiff(pose1.theta, angle_rad);
}

inline auto getAngleDiff(double angle_rad, Pose2D pose1) -> double
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

inline auto getVerticalVec(Point v) -> Point
{
  Point vertical_v;
  vertical_v << v.y(), -v.x();
  return vertical_v;
}

inline auto getReachTime(double distance, double v0, double acc, double max_vel) -> double
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

inline auto getIntersections(const Segment & segment1, const Segment & segment2) -> std::vector<Point>
{
  std::vector<Point> intersections;
  bg::intersection(segment1, segment2, intersections);
  return intersections;
}

inline auto getIntersections(const Circle & circle, const Segment & segment) -> std::vector<Point>
{
  std::vector<Point> intersections;
  double distance = bg::distance(circle, segment);
  if(distance > circle.radius) {
    // 交差しない
    return intersections;
  }else {
    // 交差する
    // 交点を求める
    Vector2 norm_vec = getVerticalVec(segment.second - segment.first).normalized();
    if (
      ((circle.center + norm_vec) - segment.first).norm() >
      ((circle.center - norm_vec) - segment.first).norm()) {
      norm_vec = -norm_vec;
    }
    double d = sqrt(circle.radius * circle.radius - distance * distance);
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
}  // namespace crane

#endif  // CRANE_BASICS__GEOMETRY_OPERATIONS_HPP_
