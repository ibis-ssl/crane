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

#ifndef CRANE_BT_EXECUTOR__UTILS__EIGEN_ADAPTER_HPP_
#define CRANE_BT_EXECUTOR__UTILS__EIGEN_ADAPTER_HPP_

#include "crane_bt_executor/utils/boost_geometry.hpp"

namespace boost::geometry::traits
{
template<>
struct tag<Point>
{
  typedef point_tag type;
};
template<>
struct coordinate_type<Eigen::Vector2f>
{
  typedef float type;
};

template<>
struct coordinate_system<Eigen::Vector2f>
{
  typedef cs::cartesian type;
};

template<>
struct dimension<Eigen::Vector2f>: boost::mpl::int_<2>
{
};

template<>
struct access<Eigen::Vector2f, 0>
{
  static float get(Eigen::Vector2f const & p)
  {
    return p.x();
  }

  static void set(Eigen::Vector2f & p, float const & value)
  {
    p.x() = value;
  }
};

template<>
struct access<Eigen::Vector2f, 1>
{
  static float get(Eigen::Vector2f const & p)
  {
    return p.y();
  }

  static void set(Eigen::Vector2f & p, float const & value)
  {
    p.y() = value;
  }
};
}  // namespace boost::geometry::traits
#endif  // CRANE_BT_EXECUTOR__UTILS__EIGEN_ADAPTER_HPP_

