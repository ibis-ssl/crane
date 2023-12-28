// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GEOMETRY__CAPSULE_HPP_
#define CRANE_GEOMETRY__CAPSULE_HPP_

//#include <Eigen/Core>
//#include <boost/geometry.hpp>
#include "eigen_adapter.hpp"

namespace crane::geometry::model
{
template <typename PointType>
struct Capsule
{
  boost::geometry::model::segment<PointType> segment;
  double radius;
};
}

namespace boost::geometry::traits
{
using crane::geometry::model::Capsule;
// タグ定義
template <typename PointType>
struct tag<Capsule<PointType>>
{
  typedef segment_tag type;
};

// 座標タイプ定義
template <typename PointType>
struct coordinate_type<Capsule<PointType>>
{
  typedef typename coordinate_type<PointType>::type type;
};

// 座標システム定義
template <typename PointType>
struct coordinate_system<Capsule<PointType>>
{
  typedef typename coordinate_system<PointType>::type type;
};

// 次元定義
template <typename PointType>
struct dimension<Capsule<PointType>> : boost::mpl::int_<2> {};

// アクセサ定義
//template <typename PointType, std::size_t Dimension>
//struct access<Capsule<PointType>, Dimension>
//{
//  static inline typename coordinate_type<PointType>::type get(
//    Capsule<PointType> const& c)
//  {
//    // ここにアクセサのロジックを実装
//  }
//
//  static inline void set(Capsule<PointType>& c,
//                         typename coordinate_type<PointType>::type const& value)
//  {
//    // ここにアクセサのロジックを実装
//  }
//};
}  // namespace boost::geometry::traits

namespace boost::geometry
{
using crane::geometry::model::Capsule;
template <typename PointType, typename Geometry1>
static double distance(const Capsule<PointType>& capsule,
                       const Geometry1& geometry1)
{
  return std::max(0., distance(capsule.segment, geometry1) - capsule.radius);
}
}  // namespace boost::geometry

#endif  // CRANE_GEOMETRY__CAPSULE_HPP_
