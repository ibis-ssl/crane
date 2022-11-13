// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_UTILITY__BOOST_GEOMETRY_HPP_
#define CRANE_UTILITY__BOOST_GEOMETRY_HPP_

#include "Eigen/Core"
#include "boost/geometry.hpp"
#include "boost/geometry/algorithms/comparable_distance.hpp"
#include "boost/geometry/algorithms/distance.hpp"
#include "boost/geometry/extensions/algorithms/closest_point.hpp"
#include "boost/geometry/geometries/box.hpp"
#include "boost/geometry/geometries/linestring.hpp"
#include "boost/geometry/geometries/polygon.hpp"
#include "boost/geometry/geometries/segment.hpp"

namespace bg = boost::geometry;
using Vector2 = Eigen::Vector2f;
using Point = Eigen::Vector2f;
using Velocity = Eigen::Vector2f;
using Accel = Eigen::Vector2f;
using Segment = bg::model::segment<Point>;
using Polygon = bg::model::polygon<Point>;
using LineString = bg::model::linestring<Point>;
using Box = bg::model::box<Point>;
using ClosestPoint = bg::closest_point_result<Point>;

struct Circle
{
  Point center;
  float radius;
};

namespace boost::geometry
{
template <typename Geometry1>
float distance(const Circle & circle, const Geometry1 & geometry1)
{
  float dist = distance(circle.center, geometry1) - circle.radius;
  if (dist < 0) {
    return 0.0f;
  }
  return dist;
}
}  // namespace boost::geometry

struct Pose2D
{
  Point pos;
  float theta;
};

struct Velocity2D
{
  Point linear;
  float omega;
};

struct Rect
{
  Point min;
  Point max;
};

#endif  // CRANE_UTILITY__BOOST_GEOMETRY_HPP_
