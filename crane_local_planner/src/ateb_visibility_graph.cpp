// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/ateb_visibility_graph.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>
#include <unordered_map>

namespace crane::ateb
{
namespace
{
/// 障害物内部の点を表面へ射影する（最大3回の反復で複数障害物の重なりに対応）
void projectOutsideObstacles(Point & p, const std::vector<Obstacle> & obstacles)
{
  for (int attempt = 0; attempt < 3; ++attempt) {
    bool any_inside = false;
    for (const auto & obs : obstacles) {
      const double d = obs.distance(p);
      if (d < 0.0) {
        p += obs.distanceGradient(p) * (-d + 1e-3);
        any_inside = true;
      }
    }
    if (!any_inside) break;
  }
}
}  // namespace

bool VisibilityGraph::isEdgeVisible(
  const Point & a, const Point & b, const std::vector<Obstacle> & inflated_obstacles) const
{
  const Segment seg(a, b);
  for (const auto & obs : inflated_obstacles) {
    switch (obs.type) {
      case Obstacle::Type::CIRCLE:
        // bg::distance(circle, seg) でCircleを第1引数にすることでカスタムオーバーロードが適用される
        // （half_radiusを引いた正しい表面距離を計算）
        if (bg::distance(obs.circle, seg) < 1e-6) {
          return false;
        }
        break;
      case Obstacle::Type::BOX:
        if (bg::intersects(seg, obs.box)) {
          return false;
        }
        break;
      case Obstacle::Type::CAPSULE:
        // 同様にCapsuleを第1引数にする
        if (bg::distance(obs.capsule, seg) < 1e-6) {
          return false;
        }
        break;
    }
  }
  return true;
}

std::vector<VisibilityGraph::Node> VisibilityGraph::generateObstacleNodes(
  const std::vector<Obstacle> & inflated_obstacles) const
{
  std::vector<Node> nodes;

  for (int obs_idx = 0; obs_idx < static_cast<int>(inflated_obstacles.size()); ++obs_idx) {
    const auto & obs = inflated_obstacles[obs_idx];
    switch (obs.type) {
      case Obstacle::Type::CIRCLE: {
        // 円の周囲に代表点を配置（左右の回避点）
        // 接線点は動的に計算するのでここでは中心のみ記録
        // 実際の接線点はsearchPaths内で生成
        nodes.push_back(
          {{obs.circle.center.x() + obs.circle.radius, obs.circle.center.y()}, obs_idx});
        nodes.push_back(
          {{obs.circle.center.x() - obs.circle.radius, obs.circle.center.y()}, obs_idx});
        nodes.push_back(
          {{obs.circle.center.x(), obs.circle.center.y() + obs.circle.radius}, obs_idx});
        nodes.push_back(
          {{obs.circle.center.x(), obs.circle.center.y() - obs.circle.radius}, obs_idx});
        break;
      }
      case Obstacle::Type::BOX: {
        // Boxの4角をBOX中心から1mmだけ外側にオフセット
        // bg::intersects(seg, box)は境界接触でtrueを返すため、角をBOX上に置くと
        // 角への全エッジが「交差」と判定されグラフ上で到達不能になる問題を回避する
        //
        // 頂点は巡回順（BL→BR→TR→TL）に配置し、隣接する頂点（辺上）のエッジのみを
        // 同一障害物内で許可する。対角線は障害物内部を横断するため除外する。
        constexpr double kCornerOffset = 1e-3;
        const Point center(
          (obs.box.min_corner().x() + obs.box.max_corner().x()) * 0.5,
          (obs.box.min_corner().y() + obs.box.max_corner().y()) * 0.5);
        const std::array<Point, 4> corners = {{
          obs.box.min_corner(),                                  // 0: BL
          {obs.box.max_corner().x(), obs.box.min_corner().y()},  // 1: BR
          obs.box.max_corner(),                                  // 2: TR
          {obs.box.min_corner().x(), obs.box.max_corner().y()},  // 3: TL
        }};
        for (int vi = 0; vi < 4; ++vi) {
          const Vector2 outward = (corners[vi] - center).normalized();
          nodes.push_back({corners[vi] + outward * kCornerOffset, obs_idx, vi});
        }
        break;
      }
      case Obstacle::Type::CAPSULE: {
        // カプセルの2端点の周囲に代表点を配置
        const auto & seg = obs.capsule.segment;
        const double r = obs.capsule.radius;
        const Vector2 dir = (seg.second - seg.first).normalized();
        const Vector2 perp(-dir.y(), dir.x());

        nodes.push_back({Point(seg.first) + perp * r, obs_idx});
        nodes.push_back({Point(seg.first) - perp * r, obs_idx});
        nodes.push_back({Point(seg.second) + perp * r, obs_idx});
        nodes.push_back({Point(seg.second) - perp * r, obs_idx});
        break;
      }
    }
  }

  return nodes;
}

std::vector<HomotopyClass> VisibilityGraph::searchPaths(
  const Point & start, const Point & goal, const std::vector<Node> & nodes,
  const std::vector<Obstacle> & inflated_obstacles, int max_paths) const
{
  // 全ノードリスト: start(0), goal(1), obstacle_nodes(2..)
  std::vector<Node> all_nodes;
  all_nodes.push_back({start, -1, -1});
  all_nodes.push_back({goal, -1, -1});
  for (const auto & n : nodes) {
    all_nodes.push_back(n);
  }

  const int n_nodes = static_cast<int>(all_nodes.size());

  // 同一BOX障害物の非隣接頂点（対角線）かどうかを判定する
  // 凸多角形の対角線は障害物内部を通過するため、エッジ候補から除外する
  auto isSameObstacleNonAdjacent = [&](int i, int j) -> bool {
    const auto & mi = all_nodes[i];
    const auto & mj = all_nodes[j];
    if (mi.obs_idx < 0 || mj.obs_idx < 0) return false;
    if (mi.obs_idx != mj.obs_idx) return false;
    if (mi.vertex_idx < 0 || mj.vertex_idx < 0) return false;
    // 巡回順で隣接 = |diff| == 1 or 3 (mod 4)
    const int diff = std::abs(mi.vertex_idx - mj.vertex_idx);
    return diff != 1 && diff != 3;  // 隣接でない → 対角線
  };

  // 有向グラフの隣接リスト（コスト付き）
  std::vector<std::vector<std::pair<int, double>>> adj(n_nodes);

  for (int i = 0; i < n_nodes; ++i) {
    for (int j = 0; j < n_nodes; ++j) {
      if (i == j) continue;
      if (isSameObstacleNonAdjacent(i, j)) continue;
      if (isEdgeVisible(all_nodes[i].pos, all_nodes[j].pos, inflated_obstacles)) {
        const double cost = (all_nodes[j].pos - all_nodes[i].pos).norm();
        adj[i].emplace_back(j, cost);
      }
    }
  }

  // A*で複数経路を列挙（単純なDijkstra + K-shortest pathの簡易版）
  struct SearchState
  {
    double cost;
    double heuristic;
    int node;
    std::vector<int> path;

    bool operator>(const SearchState & other) const
    {
      return cost + heuristic > other.cost + other.heuristic;
    }
  };

  std::vector<HomotopyClass> results;
  std::set<std::vector<int>> visited_paths;

  std::priority_queue<SearchState, std::vector<SearchState>, std::greater<SearchState>> pq;
  pq.push({0.0, (goal - start).norm(), 0, {0}});

  // 訪問回数上限（探索爆発防止）
  int iterations = 0;
  constexpr int MAX_ITERATIONS = 10000;

  while (!pq.empty() && static_cast<int>(results.size()) < max_paths &&
         iterations < MAX_ITERATIONS) {
    ++iterations;
    auto state = pq.top();
    pq.pop();

    if (state.node == 1) {
      // goalに到達
      if (!visited_paths.count(state.path)) {
        visited_paths.insert(state.path);
        HomotopyClass homotopy;
        for (int idx : state.path) {
          homotopy.waypoints.push_back(all_nodes[idx].pos);
        }
        homotopy.cost = state.cost;
        results.push_back(std::move(homotopy));
      }
      continue;
    }

    for (const auto & [next, edge_cost] : adj[state.node]) {
      // 同じノードを再訪しない（ループ防止）
      if (std::find(state.path.begin(), state.path.end(), next) != state.path.end() && next != 1) {
        continue;
      }

      SearchState next_state;
      next_state.cost = state.cost + edge_cost;
      next_state.heuristic = (goal - all_nodes[next].pos).norm();
      next_state.node = next;
      next_state.path = state.path;
      next_state.path.push_back(next);

      pq.push(std::move(next_state));
    }
  }

  // goalへの直線経路がなかった場合のフォールバック
  if (results.empty()) {
    HomotopyClass direct;
    direct.waypoints = {start, goal};
    direct.cost = (goal - start).norm();
    // 直線が障害物を横断する場合でも返す（CBFフィルタが実行時に衝突を防止する）
    // start/goal射影後もここに来る場合は構造的に回避不可能なケース
    results.push_back(std::move(direct));
  }

  return results;
}

std::vector<HomotopyClass> VisibilityGraph::extract(
  const Point & start, const Point & goal, const std::vector<Obstacle> & obstacles) const
{
  // 障害物をロボット半径+マージン分だけ膨張（skip_inflationフラグがあれば再膨張しない）
  std::vector<Obstacle> inflated;
  inflated.reserve(obstacles.size());
  for (const auto & obs : obstacles) {
    inflated.push_back(obs.skip_inflation ? obs : obs.inflated(config_.inflation_radius));
  }

  if ((goal - start).norm() < 1e-3) {
    HomotopyClass direct;
    direct.waypoints = {start, goal};
    direct.cost = (goal - start).norm();
    return {direct};
  }

  // start/goalが膨張障害物の内側にある場合は表面へ射影してから探索する
  // （直線フォールバックを返すと、BOX障害物（PA）を横断する経路が選ばれてしまう）
  Point effective_start = start;
  Point effective_goal = goal;
  projectOutsideObstacles(effective_start, inflated);
  projectOutsideObstacles(effective_goal, inflated);

  if ((effective_goal - effective_start).norm() < 1e-3) {
    HomotopyClass direct;
    direct.waypoints = {start, goal};
    direct.cost = (goal - start).norm();
    return {direct};
  }

  const auto obstacle_nodes = generateObstacleNodes(inflated);
  auto results = searchPaths(
    effective_start, effective_goal, obstacle_nodes, inflated, config_.max_homotopy_classes);

  // 結果の始点・終点を元の位置に復元（射影でずれた分を補正）
  for (auto & homotopy : results) {
    if (!homotopy.waypoints.empty()) {
      homotopy.waypoints.front() = start;
      homotopy.waypoints.back() = goal;
    }
  }

  return results;
}

}  // namespace crane::ateb
