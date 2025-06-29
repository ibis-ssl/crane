// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GEOMETRY__VECTOR2D_ADAPTER_HPP_
#define CRANE_GEOMETRY__VECTOR2D_ADAPTER_HPP_

#include <Eigen/Dense>
#include <boost/geometry.hpp>

namespace boost::geometry::traits
{
template <>
struct tag<Eigen::Vector2d>
{
  using type = point_tag;
};

template <>
struct coordinate_type<Eigen::Vector2d>
{
  using type = double;
};

template <>
struct coordinate_system<Eigen::Vector2d>
{
  using type = cs::cartesian;
};

template <>
struct dimension<Eigen::Vector2d> : boost::mpl::int_<2>
{
};

template <>
struct access<Eigen::Vector2d, 0>
{
  static auto get(Eigen::Vector2d const & p) -> double { return p.x(); }

  static auto set(Eigen::Vector2d & p, double const & value) -> void { p.x() = value; }
};

template <>
struct access<Eigen::Vector2d, 1>
{
  static auto get(Eigen::Vector2d const & p) -> double { return p.y(); }

  static auto set(Eigen::Vector2d & p, double const & value) -> void { p.y() = value; }
};
}  // namespace boost::geometry::traits
#endif  // CRANE_GEOMETRY__VECTOR2D_ADAPTER_HPP_
