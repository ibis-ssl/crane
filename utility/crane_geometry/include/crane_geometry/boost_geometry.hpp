// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GEOMETRY__BOOST_GEOMETRY_HPP_
#define CRANE_GEOMETRY__BOOST_GEOMETRY_HPP_

#include <Eigen/Core>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/comparable_distance.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/extensions/algorithms/closest_point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/segment.hpp>

#include "eigen_adapter.hpp"

namespace bg = boost::geometry;
using Vector2 = Eigen::Vector2d;
using Point = Eigen::Vector2d;
using Velocity = Eigen::Vector2d;
using Accel = Eigen::Vector2d;
using Segment = bg::model::segment<Point>;
using Polygon = bg::model::polygon<Point>;
using LineString = bg::model::linestring<Point>;
using Box = bg::model::box<Point>;
using ClosestPoint = bg::closest_point_result<Point>;

struct Circle
{
  Point center;
  double radius;
};

struct Capsule
{
  Segment segment;
  double radius;
};

namespace boost::geometry
{
template <typename Geometry1>
double distance(const Circle & circle, const Geometry1 & geometry1)
{
  double dist = distance(circle.center, geometry1) - circle.radius;
  return std::max(dist, 0.);
}

template <typename Geometry1>
double distance(const Capsule & capsule, const Geometry1 & geometry1)
{
  double dist = distance(capsule.segment, geometry1) - capsule.radius;
  return std::max(dist, 0.);
}
}  // namespace boost::geometry

struct Pose2D
{
  Point pos;
  double theta;
};

struct Velocity2D
{
  Point linear;
  double omega;
};

struct Rect
{
  Point min;
  Point max;
};

#endif  // CRANE_GEOMETRY__BOOST_GEOMETRY_HPP_
