// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

export module crane_basics:boost_geometry;

// These are now module imports
import :capsule;
import :circle;
import :eigen_adapter;

// Includes for Eigen/Core and boost/geometry.hpp are removed from here,
// as they are handled by the global module fragment in crane_basics.cppm.
// If there were any specific boost/geometry or Eigen includes here
// that are NOT in the global fragment, they would need to be added there
// or imported directly if they are module-ready.
// For now, assume all necessary boost/eigen headers are in crane_basics.cppm's global fragment.

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
