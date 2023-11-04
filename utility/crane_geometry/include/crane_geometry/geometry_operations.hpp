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

inline float normalizeAngle(float angle_rad)
{
  while (angle_rad > M_PI) {
    angle_rad -= 2.0f * M_PI;
  }
  while (angle_rad < -M_PI) {
    angle_rad += 2.0f * M_PI;
  }
  return angle_rad;
}

inline float getAngleDiff(float angle_rad1, float angle_rad2)
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
}  // namespace crane

#endif  // CRANE_GEOMETRY__GEOMETRY_OPERATIONS_HPP_
