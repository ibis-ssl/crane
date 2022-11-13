// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_UTILITY__GEOMETRY_OPERATIONS_HPP_
#define CRANE_UTILITY__GEOMETRY_OPERATIONS_HPP_

#include "crane_geometry/boost_geometry.hpp"

namespace crane
{
bool isInRect(const Rect & rect, const Point & p)
{
  if (
    p.x() >= rect.min.x() && p.x() <= rect.max.x() && p.y() >= rect.min.y() &&
    p.y() <= rect.max.y()) {
    return true;
  }
  return false;
}

double getAngle(const Vector2 & vec) { return atan2(vec.y(), vec.x()); }

}  // namespace crane

#endif  // CRANE_UTILITY__GEOMETRY_OPERATIONS_HPP_
