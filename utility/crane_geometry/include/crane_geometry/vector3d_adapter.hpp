// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GEOMETRY__VECTOR3D_ADAPTER_HPP_
#define CRANE_GEOMETRY__VECTOR3D_ADAPTER_HPP_

#include <Eigen/Dense>
#include <boost/geometry.hpp>

namespace boost::geometry::traits
{
template <>
struct tag<Eigen::Vector3d>
{
  using type = point_tag;
};

template <>
struct coordinate_type<Eigen::Vector3d>
{
  using type = double;
};

template <>
struct coordinate_system<Eigen::Vector3d>
{
  using type = cs::cartesian;
};

template <>
struct dimension<Eigen::Vector3d> : boost::mpl::int_<3>
{
};

template <>
struct access<Eigen::Vector3d, 0>
{
  static auto get(Eigen::Vector3d const & p) -> double { return p.x(); }

  static auto set(Eigen::Vector3d & p, double const & value) -> void { p.x() = value; }
};

template <>
struct access<Eigen::Vector3d, 1>
{
  static auto get(Eigen::Vector3d const & p) -> double { return p.y(); }

  static auto set(Eigen::Vector3d & p, double const & value) -> void { p.y() = value; }
};

template <>
struct access<Eigen::Vector3d, 2>
{
  static auto get(Eigen::Vector3d const & p) -> double { return p.z(); }

  static auto set(Eigen::Vector3d & p, double const & value) -> void { p.z() = value; }
};
}  // namespace boost::geometry::traits
#endif  // CRANE_GEOMETRY__VECTOR3D_ADAPTER_HPP_
