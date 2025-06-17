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

#include "capsule.hpp"
#include "circle.hpp"
<<<<<<< add-vector3d-class -- Incoming Change
#include "eigen_adapter.hpp"
#include "vector3d.hpp"
  =======
#include "vector2d_adapter.hpp"
  >>>>>>> develop -- Current Change

  namespace crane
{
  namespace bg = boost::geometry;
<<<<<<< add-vector3d-class -- Incoming Change
  using Vector2 = Eigen::Vector2d;
  using Vector3 = crane::Vector3d;
  using Point = Eigen::Vector2d;
  using Point3D = crane::Vector3d;
  using Velocity = Eigen::Vector2d;
  using Velocity3D = crane::Vector3d;
  using Accel = Eigen::Vector2d;
=======
  using Vector2 = crane::Vector2d;
  using Point = crane::Vector2d;
  using Velocity = crane::Vector2d;
  using Accel = crane::Vector2d;
>>>>>>> develop -- Current Change
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
