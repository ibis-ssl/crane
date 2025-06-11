// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <Eigen/Core>
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
#include "eigen_adapter.hpp"

export module crane_basics:boost_geometry;

export namespace crane
{
namespace bg = boost::geometry;
export using Vector2 = Eigen::Vector2d;
export using Point = Eigen::Vector2d;
export using Velocity = Eigen::Vector2d;
export using Accel = Eigen::Vector2d;
export using Segment = bg::model::segment<Point>;
export using Polygon = bg::model::polygon<Point>;
export using LineString = bg::model::linestring<Point>;
export using Box = bg::model::box<Point>;
export using ClosestPoint = bg::closest_point_result<Point>;
export using Circle = crane::geometry::model::Circle<Point>;
export using Capsule = crane::geometry::model::Capsule<Point>;

export struct Pose2D
{
  Point pos;
  double theta;
};

export struct Velocity2D
{
  Point linear;
  double omega;
};
}  // namespace crane
