// Copyright (c) 2020 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef CRANE_BT_EXECUTOR__UTILS__BOOST_GEOMETRY_HPP_
#define CRANE_BT_EXECUTOR__UTILS__BOOST_GEOMETRY_HPP_

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/comparable_distance.hpp>
#include <boost/geometry/extensions/algorithms/closest_point.hpp>
#include <Eigen/Core>


namespace bg = boost::geometry;
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
template<typename Geometry1>
float distance(const Circle & circle, const Geometry1 & geometry1)
{
  float dist = distance(circle.center, geometry1) - circle.radius;
  if (dist < 0) {
    return 0.0f;
  }
  return dist;
}
}  // namespace boost::geometry
#endif  // CRANE_BT_EXECUTOR__UTILS__BOOST_GEOMETRY_HPP_
