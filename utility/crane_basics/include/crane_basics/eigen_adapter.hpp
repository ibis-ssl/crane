// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

export module crane_basics:eigen_adapter;

// Includes for Eigen/Core and boost/geometry.hpp are removed,
// assuming they are provided by the main module's global fragment.

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
