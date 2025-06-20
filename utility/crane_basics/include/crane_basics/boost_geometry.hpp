// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__BOOST_GEOMETRY_HPP_
#define CRANE_BASICS__BOOST_GEOMETRY_HPP_

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/comparable_distance.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/extensions/algorithms/closest_point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <crane_geometry/capsule.hpp>
#include <crane_geometry/circle.hpp>
#include <crane_geometry/vector2d_adapter.hpp>
#include <crane_geometry/vector3d.hpp>
#include <crane_geometry/vector3d_adapter.hpp>

namespace crane
{
namespace bg = boost::geometry;
using Vector2 = crane::Vector2d;
using Vector3 = crane::Vector3d;
using Point = crane::Vector2d;
using Point3D = crane::Vector3d;
using Velocity = crane::Vector2d;
using Velocity3D = crane::Vector3d;
using Accel = crane::Vector2d;
using Segment = bg::model::segment<Point>;
using Polygon = bg::model::polygon<Point>;
using LineString = bg::model::linestring<Point>;
using Box = bg::model::box<Point>;
using ClosestPoint = bg::closest_point_result<Point>;
using Circle = crane::geometry::model::Circle<Point>;
using Capsule = crane::geometry::model::Capsule<Point>;

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
}  // namespace crane

#endif  // CRANE_BASICS__BOOST_GEOMETRY_HPP_
