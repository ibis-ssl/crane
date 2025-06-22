// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GEOMETRY__CIRCLE_HPP_
#define CRANE_GEOMETRY__CIRCLE_HPP_

#include <algorithm>
#include <crane_geometry/vector2d_adapter.hpp>
#include <set>

namespace crane::geometry::model
{
template <typename PointType>
struct Circle
{
  PointType center;
  double radius;
};
}  // namespace crane::geometry::model

namespace boost::geometry::traits
{
using crane::geometry::model::Circle;
// タグ定義
template <typename PointType>
struct tag<Circle<PointType>>
{
  using type = point_tag;
};

// 座標タイプ定義
template <typename PointType>
struct coordinate_type<Circle<PointType>>
{
  using type = typename coordinate_type<PointType>::type;
};

// 座標システム定義
template <typename PointType>
struct coordinate_system<Circle<PointType>>
{
  using type = typename coordinate_system<PointType>::type;
};

// 次元定義
template <typename PointType>
struct dimension<Circle<PointType>> : boost::mpl::int_<2>
{
};

// アクセサ定義
template <typename PointType, std::size_t Dimension>
struct access<Circle<PointType>, Dimension>
{
  static inline auto get(Circle<PointType> const & c) -> typename coordinate_type<PointType>::type
  {
    return geometry::get<Dimension>(c.center);
  }

  static inline auto set(
    Circle<PointType> & c, typename coordinate_type<PointType>::type const & value) -> void
  {
    geometry::set<Dimension>(c.center, value);
  }
};
}  // namespace boost::geometry::traits

namespace boost::geometry
{
using crane::geometry::model::Circle;
template <typename PointType, typename Geometry1>
static auto distance(
  const crane::geometry::model::Circle<PointType> & circle, const Geometry1 & geometry1) -> double
{
  return std::max(0., distance(circle.center, geometry1) - circle.radius);
}
}  // namespace boost::geometry

#endif  // CRANE_GEOMETRY__CIRCLE_HPP_
