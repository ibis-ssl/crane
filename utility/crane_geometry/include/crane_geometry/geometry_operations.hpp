// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GEOMETRY__GEOMETRY_OPERATIONS_HPP_
#define CRANE_GEOMETRY__GEOMETRY_OPERATIONS_HPP_

#include "boost_geometry.hpp"

namespace crane
{
inline bool isInRect(const Rect & rect, const Point & p)
{
  if (
    p.x() >= rect.min.x() && p.x() <= rect.max.x() && p.y() >= rect.min.y() &&
    p.y() <= rect.max.y()) {
    return true;
  }
  return false;
}

inline double getAngle(const Vector2 & vec) { return atan2(vec.y(), vec.x()); }

inline double normalizeAngle(double angle_rad)
{
  while (angle_rad > M_PI) {
    angle_rad -= 2.0f * M_PI;
  }
  while (angle_rad < -M_PI) {
    angle_rad += 2.0f * M_PI;
  }
  return angle_rad;
}

inline double getAngleDiff(double angle_rad1, double angle_rad2)
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

inline double getIntermediateAngle(double angle_rad1, double angle_rad2)
{
  angle_rad1 = normalizeAngle(angle_rad1);
  angle_rad2 = normalizeAngle(angle_rad2);
  //差がpiを超えている場合では平均を取るだけではダメ
  if (abs(angle_rad1 - angle_rad2) > M_PI) {
    return normalizeAngle((angle_rad1 + angle_rad2 + 2.0f * M_PI) / 2.0f);
  } else {
    return (angle_rad1 + angle_rad2) / 2.0f;
  }
}

inline Vector2 getNormVec(const double angle) { return {cos(angle), sin(angle)}; }

inline Point getVerticalVec(Point v)
{
  Point vertical_v;
  vertical_v << v.y(), -v.x();
  return vertical_v;
}

inline double getReachTime(double distance, double v0, double acc, double max_vel)
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
}  // namespace crane

#endif  // CRANE_GEOMETRY__GEOMETRY_OPERATIONS_HPP_
