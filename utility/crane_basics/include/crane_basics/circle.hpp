// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

export module crane_basics:circle;

// #include <algorithm> // Removed, assumed from crane_basics.cppm global fragment
// #include <set> // Removed, assumed from crane_basics.cppm global fragment

import :eigen_adapter;

export namespace crane::geometry::model
{
export template <typename PointType>
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

export namespace boost::geometry
{
using crane::geometry::model::Circle;
export template <typename PointType, typename Geometry1>
static auto distance(
  const crane::geometry::model::Circle<PointType> & circle, const Geometry1 & geometry1) -> double
{
  return std::max(0., distance(circle.center, geometry1) - circle.radius);
}
}  // namespace boost::geometry
