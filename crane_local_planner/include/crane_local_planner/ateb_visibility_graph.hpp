// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__ATEB_VISIBILITY_GRAPH_HPP_
#define CRANE_LOCAL_PLANNER__ATEB_VISIBILITY_GRAPH_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <queue>
#include <vector>

#include "ateb_types.hpp"

namespace crane::ateb
{

/// 縮約可視グラフによるホモトピークラス抽出器
///
/// 障害物間の接線を計算して可視グラフを構築し、A*探索で
/// 上位N個の異なるホモトピークラスを抽出する。
///
/// 対応する障害物タイプ:
/// - Circle: 外接・内接接線（代数的にO(1)）
/// - Box: 4頂点を可視グラフノードとして使用
/// - Capsule: 2端点の半円をCircleとして扱う
class VisibilityGraph
{
public:
  struct Config
  {
    int max_homotopy_classes = 3;     ///< 抽出するホモトピークラスの最大数
    double inflation_radius = 0.090;  ///< 障害物膨張半径 [m]（ロボット半径+マージン）
  };

  void configure(const Config & config) { config_ = config; }

  /// 可視グラフを構築してホモトピークラスを抽出する
  ///
  /// @param start 始点
  /// @param goal 終点
  /// @param obstacles 障害物リスト（膨張前）
  /// @return 上位N個のホモトピークラス（コスト昇順）
  [[nodiscard]] std::vector<HomotopyClass> extract(
    const Point & start, const Point & goal, const std::vector<Obstacle> & obstacles) const;

private:
  Config config_;

  /// グラフノード（位置と元の障害物インデックス）
  struct Node
  {
    Point pos;
    int obs_idx = -1;     ///< -1: start/goal, >=0: 障害物インデックス
    int vertex_idx = -1;  ///< BOX頂点の巡回インデックス（0-3）。隣接判定に使用
  };

  /// 2点間のエッジが全障害物と交差しないか確認
  [[nodiscard]] bool isEdgeVisible(
    const Point & a, const Point & b, const std::vector<Obstacle> & inflated_obstacles) const;

  /// 膨張後の障害物からグラフノードを生成
  [[nodiscard]] std::vector<Node> generateObstacleNodes(
    const std::vector<Obstacle> & inflated_obstacles) const;

  /// A*探索（コスト順）で上位N経路を列挙する
  [[nodiscard]] std::vector<HomotopyClass> searchPaths(
    const Point & start, const Point & goal, const std::vector<Node> & nodes,
    const std::vector<Obstacle> & inflated_obstacles, int max_paths) const;
};

}  // namespace crane::ateb

#endif  // CRANE_LOCAL_PLANNER__ATEB_VISIBILITY_GRAPH_HPP_
