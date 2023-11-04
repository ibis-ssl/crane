// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GEOMETRY__EIGEN_ADAPTER_HPP_
#define CRANE_GEOMETRY__EIGEN_ADAPTER_HPP_

#include "boost_geometry.hpp"

namespace boost::geometry::traits
{
template <>
struct tag<Point>
{
  typedef point_tag type;
};
template <>
struct coordinate_type<Eigen::Vector2f>
{
  typedef float type;
};

template <>
struct coordinate_system<Eigen::Vector2f>
{
  typedef cs::cartesian type;
};

template <>
struct dimension<Eigen::Vector2f> : boost::mpl::int_<2>
{
};

template <>
struct access<Eigen::Vector2f, 0>
{
  static float get(Eigen::Vector2f const & p) { return p.x(); }

  static void set(Eigen::Vector2f & p, float const & value) { p.x() = value; }
};

template <>
struct access<Eigen::Vector2f, 1>
{
  static float get(Eigen::Vector2f const & p) { return p.y(); }

  static void set(Eigen::Vector2f & p, float const & value) { p.y() = value; }
};
}  // namespace boost::geometry::traits
#endif  // CRANE_GEOMETRY__EIGEN_ADAPTER_HPP_
