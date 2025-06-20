// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GEOMETRY__VECTOR3D_ADAPTER_HPP_
#define CRANE_GEOMETRY__VECTOR3D_ADAPTER_HPP_

#include <boost/geometry.hpp>
#include <crane_geometry/vector3d.hpp>  // Include the new Vector3d class

namespace boost::geometry::traits
{
template <>
struct tag<crane::Vector3d>
{
  using type = point_tag;
};

template <>
struct coordinate_type<crane::Vector3d>
{
  using type = double;
};

template <>
struct coordinate_system<crane::Vector3d>
{
  using type = cs::cartesian;
};

template <>
struct dimension<crane::Vector3d> : boost::mpl::int_<3>
{
};

template <>
struct access<crane::Vector3d, 0>
{
  static auto get(crane::Vector3d const & p) -> double { return p.x(); }

  static auto set(crane::Vector3d & p, double const & value) -> void { p.x() = value; }
};

template <>
struct access<crane::Vector3d, 1>
{
  static auto get(crane::Vector3d const & p) -> double { return p.y(); }

  static auto set(crane::Vector3d & p, double const & value) -> void { p.y() = value; }
};

template <>
struct access<crane::Vector3d, 2>
{
  static auto get(crane::Vector3d const & p) -> double { return p.z(); }

  static auto set(crane::Vector3d & p, double const & value) -> void { p.z() = value; }
};
}  // namespace boost::geometry::traits
#endif  // CRANE_GEOMETRY__VECTOR3D_ADAPTER_HPP_
