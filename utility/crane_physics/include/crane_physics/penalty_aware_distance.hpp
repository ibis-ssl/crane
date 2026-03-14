// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PHYSICS__PENALTY_AWARE_DISTANCE_HPP_
#define CRANE_PHYSICS__PENALTY_AWARE_DISTANCE_HPP_

#include <algorithm>
#include <crane_geometry/boost_geometry.hpp>

namespace crane
{

/// @brief ペナルティエリア迂回を考慮した移動距離推定
///
/// ロボットが start から target まで移動する際、経路がペナルティエリアを横切る場合は
/// ローカルプランナー（adjustForPenaltyAreaAvoidance）と同等のロジックで
/// ペナルティエリアの角を経由する迂回距離を返す。
/// 交差しない場合はユークリッド距離をそのまま返す。
///
/// @param start       ロボット現在位置
/// @param target      移動目標位置
/// @param our_pa      自陣ペナルティエリア（Box）
/// @param our_goal    自陣ゴールセンター位置
/// @param their_pa    敵陣ペナルティエリア（Box）
/// @param their_goal  敵陣ゴールセンター位置
/// @param pa_size     ペナルティエリアサイズ（x: depth, y: width）
/// @return 推定移動距離 [m]
inline double estimatePenaltyAwareDistance(
  const Point & start, const Point & target, const Box & our_pa, const Point & our_goal,
  const Box & their_pa, const Point & their_goal, const Point & pa_size)
{
  // rvo2_planner と同じオフセット値
  constexpr double PENALTY_AREA_OFFSET = 0.1;
  constexpr double SURROUNDING_OFFSET = 0.2;

  double distance = (target - start).norm();

  // Segment は2つのエリアで共通なので一度だけ構築する
  const Segment path(start, target);

  // 各ペナルティエリアに対して迂回が必要か判定し、必要なら迂回距離を採用
  auto applyDetourIfNeeded = [&](const Box & area, const Point & goal_pos) {
    // ペナルティエリアをオフセット分拡張してから交差判定
    Box expanded = area;
    expanded.min_corner() -= Point(PENALTY_AREA_OFFSET, PENALTY_AREA_OFFSET);
    expanded.max_corner() += Point(PENALTY_AREA_OFFSET, PENALTY_AREA_OFFSET);

    if (!bg::intersects(path, expanded)) {
      return;  // 経路がペナルティエリアを通らない → 変更不要
    }

    // rvo2_planner の adjustForPenaltyAreaAvoidance と同じ角計算
    // goal_pos からペナルティエリアの前端（フィールド中央側）の角2点を算出し
    // SURROUNDING_OFFSET 分外側に出た点を迂回ポイントとする
    const double x_offset = std::copysign(pa_size.x() + SURROUNDING_OFFSET, -goal_pos.x());
    const double half_height = pa_size.y() * 0.5 + SURROUNDING_OFFSET;
    const Point around_corner_1 = goal_pos + Point(x_offset, half_height);
    const Point around_corner_2 = goal_pos + Point(x_offset, -half_height);

    const double dist_via_1 = (around_corner_1 - start).norm() + (target - around_corner_1).norm();
    const double dist_via_2 = (around_corner_2 - start).norm() + (target - around_corner_2).norm();

    distance = std::min(dist_via_1, dist_via_2);
  };

  applyDetourIfNeeded(our_pa, our_goal);
  applyDetourIfNeeded(their_pa, their_goal);

  return distance;
}

}  // namespace crane

#endif  // CRANE_PHYSICS__PENALTY_AWARE_DISTANCE_HPP_
