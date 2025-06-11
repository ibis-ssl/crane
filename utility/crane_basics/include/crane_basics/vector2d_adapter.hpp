// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__VECTOR2D_ADAPTER_HPP_
#define CRANE_BASICS__VECTOR2D_ADAPTER_HPP_

#include <crane_basics/vector2d.hpp> // Include the new Vector2d class
#include <boost/geometry.hpp>

namespace boost::geometry::traits
{
template <>
struct tag<crane::Vector2d>
{
  using type = point_tag;
};

template <>
struct coordinate_type<crane::Vector2d>
{
  using type = double;
};

template <>
struct coordinate_system<crane::Vector2d>
{
  using type = cs::cartesian;
};

template <>
struct dimension<crane::Vector2d> : boost::mpl::int_<2>
{
};

template <>
struct access<crane::Vector2d, 0>
{
  static auto get(crane::Vector2d const & p) -> double { return p.get_x(); }

  static auto set(crane::Vector2d & p, double const & value) -> void { p.set_x(value); }
};

template <>
struct access<crane::Vector2d, 1>
{
  static auto get(crane::Vector2d const & p) -> double { return p.get_y(); }

  static auto set(crane::Vector2d & p, double const & value) -> void { p.set_y(value); }
};
}  // namespace boost::geometry::traits
#endif  // CRANE_BASICS__VECTOR2D_ADAPTER_HPP_
