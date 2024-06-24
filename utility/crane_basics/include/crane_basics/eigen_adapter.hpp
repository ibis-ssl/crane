// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__EIGEN_ADAPTER_HPP_
#define CRANE_BASICS__EIGEN_ADAPTER_HPP_

#include <Eigen/Core>
#include <boost/geometry.hpp>

namespace boost::geometry::traits
{
template <>
struct tag<Eigen::Vector2d>
{
  typedef point_tag type;
};

template <>
struct coordinate_type<Eigen::Vector2d>
{
  typedef double type;
};

template <>
struct coordinate_system<Eigen::Vector2d>
{
  typedef cs::cartesian type;
};

template <>
struct dimension<Eigen::Vector2d> : boost::mpl::int_<2>
{
};

template <>
struct access<Eigen::Vector2d, 0>
{
  static double get(Eigen::Vector2d const & p) { return p.x(); }

  static void set(Eigen::Vector2d & p, double const & value) { p.x() = value; }
};

template <>
struct access<Eigen::Vector2d, 1>
{
  static double get(Eigen::Vector2d const & p) { return p.y(); }

  static void set(Eigen::Vector2d & p, double const & value) { p.y() = value; }
};
}  // namespace boost::geometry::traits
#endif  // CRANE_BASICS__EIGEN_ADAPTER_HPP_
