// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/graph_planner.hpp"

#include <algorithm>
#include <unordered_set>
#include <rclcpp/rclcpp.hpp>

namespace crane
{
GraphPlanner::GraphPlanner(rclcpp::Node & node, WorldModelWrapper::SharedPtr world_model)
: node_(&node), world_(world_model)
{
  // 可視化レイヤを自前で用意
  viz_ = std::make_shared<VisualizerMessageBuilder>("graph_planner");
  node_->declare_parameter("graph_planner.max_expansion", params_.max_expansion);
  node_->declare_parameter("graph_planner.node_tangent_offset", params_.node_tangent_offset);
  node_->declare_parameter("graph_planner.node_merge_epsilon", params_.node_merge_epsilon);

  node_->declare_parameter("graph_planner.robot_radius", params_.robot_radius);
  node_->declare_parameter("graph_planner.static_margin", params_.static_margin);
  node_->declare_parameter("graph_planner.K_alpha", params_.K_alpha);
  node_->declare_parameter("graph_planner.K_v", params_.K_v);
  node_->declare_parameter("graph_planner.K_t", params_.K_t);
  node_->declare_parameter("graph_planner.far_margin_cap", params_.far_margin_cap);
  reloadParamsFromROS();
}

void GraphPlanner::reloadParamsFromROS()
{
  // 初回のみ宣言し、呼び出し毎に現在値を取得
  params_.max_expansion = node_->get_parameter("graph_planner.max_expansion").as_int();
  params_.node_tangent_offset =
    node_->get_parameter("graph_planner.node_tangent_offset").as_double();
  params_.node_merge_epsilon = node_->get_parameter("graph_planner.node_merge_epsilon").as_double();

  params_.robot_radius = node_->get_parameter("graph_planner.robot_radius").as_double();
  params_.static_margin = node_->get_parameter("graph_planner.static_margin").as_double();
  params_.K_alpha = node_->get_parameter("graph_planner.K_alpha").as_double();
  params_.K_v = node_->get_parameter("graph_planner.K_v").as_double();
  params_.K_t = node_->get_parameter("graph_planner.K_t").as_double();
  params_.far_margin_cap = node_->get_parameter("graph_planner.far_margin_cap").as_double();
}

auto GraphPlanner::intersects(const Segment & seg, const CircleObstacle & c) -> bool
{
  return !getIntersections(c, seg).empty();
}

auto GraphPlanner::intersects(const Segment & seg, const BoxObstacle & b) -> bool
{
  return bg::intersects(seg, b);
}

auto GraphPlanner::intersects(const Segment & seg, const Obstacle & ob) -> bool
{
  return std::visit([&](const auto & o) { return intersects(seg, o); }, ob);
}

auto GraphPlanner::intersectsAny(const Segment & seg, const std::vector<Obstacle> & obs) -> bool
{
  return std::any_of(obs.begin(), obs.end(), [&](const auto & ob) { return intersects(seg, ob); });
}

// 式(2)補助: 動的マージン評価で用いる予測時間 ∆t を計算
static inline double compute_dt(double r, double K_v, double K_t)
{
  return std::min(r / std::max(1e-6, K_v), K_t);
}

auto GraphPlanner::buildObstacles(const Pose2D & start) -> std::vector<Obstacle>
{
  std::vector<Obstacle> obs;

  const Point start_pos = start.pos;

  // ロボット群（味方・敵）。開始位置のロボットは識別不能のため十分近い場合は除外
  auto push_robot_circle = [&](const RobotInfo::SharedPtr & r) {
    Point c = r->pose.pos;
    // 自機とみなせるほど近い場合はスキップ
    if ((c - start_pos).norm() < params_.robot_radius * 0.5) return;

    double r_dist = (c - start_pos).norm();
    double dt = compute_dt(r_dist, params_.K_v, params_.K_t);
    // 予測変位 ld = ∆t * v_r
    Point ld = r->vel.linear * dt;
    Point pred_center = c + ld;
    // マージン lm = Ms + 1/2 Kα ∆t^2
    double lm = params_.static_margin + 0.5 * params_.K_alpha * dt * dt;
    lm = std::min(lm, params_.far_margin_cap);

    // RobotInfo::geometry() を活用して基準半径を取得（安全側で大きい方を採用）
    const double base_r = std::max(params_.robot_radius, r->geometry().radius);
    CircleObstacle co{pred_center, base_r + lm};
    obs.emplace_back(co);
  };

  for (const auto & rr : world_->ours().robots) {
    if (rr->available) push_robot_circle(rr);
  }
  for (const auto & rr : world_->theirs().robots) {
    if (rr->available) push_robot_circle(rr);
  }

  // ペナルティエリア: inflateBox でマージン付きボックスを生成
  obs.emplace_back(BoxObstacle{inflateBox(world_->getOurPenaltyArea(), params_.static_margin)});
  obs.emplace_back(BoxObstacle{inflateBox(world_->getTheirPenaltyArea(), params_.static_margin)});

  // フィールド境界: 障害物としては追加せず、展開時にフィールド外となる候補を棄却する
  return obs;
}


auto GraphPlanner::tangentPointsFromPointToCircle(const Point & p, const CircleObstacle & c)
  -> std::vector<Point>
{
  std::vector<Point> tps;
  Point u = p - c.center;
  double d2 = u.squaredNorm();
  double r = c.radius;
  if (d2 <= r * r) {
    // 点が円の内側にあると接線は引けない
    return tps;
  }
  // 標準式により接点を計算
  // T = C + (r^2/d^2) * u +/- (r * sqrt(d^2 - r^2) / d^2) * perp(u)
  double l = (r * r) / d2;
  double h = r * std::sqrt(std::max(0.0, d2 - r * r)) / d2;
  Point perp = getVerticalVec(u);

  Point t1 = c.center + l * u + h * perp;
  Point t2 = c.center + l * u - h * perp;

  // 数値衝突を避けるため、円外側へ微小オフセット
  Point n1 = (t1 - c.center).normalized();
  Point n2 = (t2 - c.center).normalized();
  t1 += n1 * 1e-3;  // 微小イプシロン
  t2 += n2 * 1e-3;

  tps.push_back(t1);
  tps.push_back(t2);
  return tps;
}

auto GraphPlanner::angleCosBetween(const Point & a, const Point & b) -> double
{
  double na = a.norm();
  double nb = b.norm();
  if (na < 1e-9 || nb < 1e-9) return 1.0;
  return a.dot(b) / (na * nb);
}

auto GraphPlanner::expandFrom(
  int from_id, const Point & from, const Point & goal, const std::vector<Obstacle> & obstacles,
  std::vector<Node> & nodes) -> std::vector<int>
{
  std::vector<int> new_node_ids;

  // 目標までの直線が空いていれば、それを優先
  Segment direct(from, goal);
  if (!intersectsAny(direct, obstacles)) {
    // ゴールをノードとして追加
    int id = static_cast<int>(nodes.size());
    nodes.push_back(Node{id, goal});
    new_node_ids.push_back(id);
    return new_node_ids;
  }

  // 直線に衝突する障害物のみを対象に接線候補を生成（オンデマンド拡張）
  std::vector<Obstacle> blocking;
  for (const auto & ob : obstacles) {
    if (intersects(direct, ob)) blocking.push_back(ob);
  }

  // 衝突障害物ごとに候補点（接線/外側オフセット）を生成
  for (const auto & ob : blocking) {
    if (std::holds_alternative<CircleObstacle>(ob)) {
      const auto & co = std::get<CircleObstacle>(ob);
      for (const auto & tp : tangentPointsFromPointToCircle(from, co)) {
        // 辺方向に僅かに進めた点を候補とする（数値安定性）
        Point dir = (tp - from).normalized();
        Point cand = tp + dir * params_.node_tangent_offset;

        // from→cand がいずれの障害物とも交差しないこと
        Segment e(from, cand);
        if (intersectsAny(e, obstacles)) continue;

        // フィールド内であること
        if (!world_->point_checker.isFieldInside(cand, 0.0)) continue;

        int id = -1;
        for (size_t k = 0; k < nodes.size(); ++k) {
          if ((nodes[k].p - cand).norm() <= params_.node_merge_epsilon) {
            id = static_cast<int>(k);
            break;
          }
        }
        if (id == -1) {
          id = static_cast<int>(nodes.size());
          nodes.push_back(Node{id, cand});
        }
        new_node_ids.push_back(id);
      }
    } else {
      // ボックス障害物: 角から外方向にオフセットした点を候補にする
      const auto & bb = std::get<BoxObstacle>(ob);
      auto minc = bb.min_corner();
      auto maxc = bb.max_corner();
      std::vector<Point> corners{
        Point(minc.x(), minc.y()), Point(maxc.x(), minc.y()), Point(maxc.x(), maxc.y()),
        Point(minc.x(), maxc.y())};
      Point center = 0.5 * (minc + maxc);
      for (const auto & c : corners) {
        Point n = (c - center).normalized();
        Point around = c + n * params_.node_tangent_offset;
        Segment e(from, around);
        if (intersectsAny(e, obstacles)) continue;
        if (!world_->point_checker.isFieldInside(around, 0.0)) continue;
        int id = -1;
        for (size_t k = 0; k < nodes.size(); ++k) {
          if ((nodes[k].p - around).norm() <= params_.node_merge_epsilon) {
            id = static_cast<int>(k);
            break;
          }
        }
        if (id == -1) {
          id = static_cast<int>(nodes.size());
          nodes.push_back(Node{id, around});
        }
        new_node_ids.push_back(id);
      }
    }
  }
  return new_node_ids;
}

// 式(1): ポリライン各区間に対する到達速度・時間を計算
auto GraphPlanner::buildWaypointsWithVelocities(
  const std::vector<Point> & path_points, const Velocity & v0, const Constraints & limits) const
  -> std::vector<Waypoint>
{
  const int Np = static_cast<int>(path_points.size());
  std::vector<Waypoint> wps;
  if (Np == 0) return wps;
  if (Np == 1) {
    wps.push_back(Waypoint{path_points.front(), Velocity::Zero()});
    return wps;
  }

  // エッジ e(n) と長さ ℓ(n) を事前計算
  const int N = Np - 1;  // エッジ数
  std::vector<Point> e(N);
  std::vector<double> L(N);
  for (int n = 0; n < N; ++n) {
    e[n] = path_points[n + 1] - path_points[n];
    L[n] = e[n].norm();
  }

  // 逆走査: ℓp(n) を式(1)の制約に基づき計算
  std::vector<double> Lp(N, 0.0);  // ℓp(n)
  for (int n = N - 1; n >= 0; --n) {
    if (n == N - 1) {
      Lp[n] = 0.0;  // Eq.(1): ℓp(N) = 0
    } else {
      double cos_next = angleCosBetween(e[n + 1], e[n]);
      cos_next = std::clamp(cos_next, -1.0, 1.0);
      double bound_from_next = (L[n + 1] + Lp[n + 1]) * std::max(0.0, cos_next);
      double vmax_bound = (limits.vmax * limits.vmax) / (2.0 * limits.alpha_dec);
      // 初期化
      Lp[n] = 0.0;
      Lp[n] = std::min(vmax_bound, bound_from_next);
      if (Lp[n] < 0.0) Lp[n] = 0.0;
    }
  }

  // 正方向走査: v0(n), vm(n), ve(n) を式(1)に基づき計算
  std::vector<double> v0n(N, 0.0), vm(N, 0.0), ve(N, 0.0);
  for (int n = 0; n < N; ++n) {
    double cos_theta;
    if (n == 0) {
      cos_theta = angleCosBetween(e[0], v0);
    } else {
      cos_theta = angleCosBetween(e[n], e[n - 1]);
    }
    cos_theta = std::clamp(cos_theta, -1.0, 1.0);
    v0n[n] = std::max(0.0, cos_theta) * (n == 0 ? v0.norm() : ve[n - 1]);

    // vm（式(1)）
    // vm = min( sqrt( 2 αa αb (ℓ + ℓp) + αb v0^2 ) / (αa + αb), vmax )
    double num = 2.0 * limits.alpha_acc * limits.alpha_dec * (L[n] + Lp[n]) +
                 limits.alpha_dec * v0n[n] * v0n[n];
    double den = limits.alpha_acc + limits.alpha_dec;
    vm[n] = std::min(std::sqrt(std::max(0.0, num)) / std::max(1e-6, den), limits.vmax);

    // ve（式(1)）: ve = min( sqrt(v0^2 + 2 αa ℓ), sqrt(2 αb ℓp) )
    double ve_acc = std::sqrt(std::max(0.0, v0n[n] * v0n[n] + 2.0 * limits.alpha_acc * L[n]));
    double ve_dec = std::sqrt(std::max(0.0, 2.0 * limits.alpha_dec * Lp[n]));
    ve[n] = std::min({vm[n], ve_acc, ve_dec});
  }

  // 各ウェイポイントでの到達速度ベクトル（当該区間の終端速度）を付与
  wps.reserve(Np);
  for (int i = 0; i < Np; ++i) {
    Velocity v = Velocity::Zero();
    if (i == 0) {
      // 初期点: v0 を第1エッジ方向へ射影し、ve[0]でクリップ
      if (N > 0 && L[0] > 1e-6) v = (e[0] / L[0]) * std::min(ve[0], limits.vmax);
    } else if (i - 1 < N && L[i - 1] > 1e-6) {
      v = (e[i - 1] / L[i - 1]) * std::min(ve[i - 1], limits.vmax);
    }
    wps.push_back(Waypoint{path_points[i], v});
  }
  return wps;
}

auto GraphPlanner::plan(
  const Pose2D & start, const Pose2D & goal, const Velocity & v0, const Constraints & limits)
  -> std::vector<Waypoint>
{
  // world_ は前提として有効

  RCLCPP_DEBUG(
    node_->get_logger(),
    "[GraphPlanner] plan start=(%.2f,%.2f) goal=(%.2f,%.2f) v0=%.2f vmax=%.2f acc=%.2f dec=%.2f",
    start.pos.x(), start.pos.y(), goal.pos.x(), goal.pos.y(), v0.norm(), limits.vmax, limits.alpha_acc,
    limits.alpha_dec);

  reloadParamsFromROS();

  // 障害物生成
  const auto obstacles = buildObstacles(start);

  // ノード: 0=始点。ゴールは必要時に追加
  std::vector<Node> nodes;
  nodes.push_back(Node{0, start.pos});

  // Dijkstra風の優先度付き待ち行列（距離/時間コストは経路依存）
  std::priority_queue<PQItem> pq;
  std::vector<PathState> states;
  states.resize(1);
  states[0].cost = 0.0;
  states[0].parent = -1;
  pq.push(PQItem{0, 0.0});

  int goal_node = -1;
  int expansions = 0;

  // 経路依存コストの算出用: 親を辿ってポリラインを復元
  auto reconstruct_path_points = [&](int end_id) {
    std::vector<Point> pts;
    for (int cur = end_id; cur >= 0;) {
      pts.push_back(nodes[cur].p);
      cur = states[cur].parent;
    }
    std::reverse(pts.begin(), pts.end());
    return pts;
  };

  auto compute_path_cost = [&](int end_id) {
    auto pts = reconstruct_path_points(end_id);
    return polylineLength(pts);
  };

  // 簡易な訪問済み抑制（粗いグリッド・ハッシュ）
  std::unordered_set<std::uint64_t> seen;  // グリッド座標の集合
  auto hash_point = [](const Point & p) -> std::uint64_t {
    // グリッド間隔: 2 cm
    std::int64_t xi = static_cast<std::int64_t>(std::llround(p.x() * 50.0));
    std::int64_t yi = static_cast<std::int64_t>(std::llround(p.y() * 50.0));
    return (static_cast<std::uint64_t>(xi) << 32) ^
           (static_cast<std::uint64_t>(yi) & 0xffffffffULL);
  };

  seen.insert(hash_point(start.pos));

  while (!pq.empty() && expansions < params_.max_expansion) {
    auto top = pq.top();
    pq.pop();
    int u = top.node_id;
    // ゴール到達判定
    if ((nodes[u].p - goal.pos).norm() < 1e-2) {
      goal_node = u;
      break;
    }

    // 近傍ノードの生成
    auto neighbors = expandFrom(u, nodes[u].p, goal.pos, obstacles, nodes);
    for (int v : neighbors) {
      // 中点がフィールド外となる辺は棄却
      Point mid = 0.5 * (nodes[u].p + nodes[v].p);
      if (!world_->point_checker.isFieldInside(mid, 0.0)) continue;

      // 近傍重複の抑制
      std::uint64_t h = hash_point(nodes[v].p);
      if (seen.find(h) != seen.end()) continue;
      seen.insert(h);

      // 親を設定し、終端vまでの経路でコストを評価
      if (static_cast<int>(states.size()) <= v) states.resize(v + 1);
      states[v].parent = u;
      states[v].cost = compute_path_cost(v);
      pq.push(PQItem{v, states[v].cost});

      expansions++;

      // 目標に十分近ければ終端にゴールノードを追加
      if ((nodes[v].p - goal.pos).norm() < 0.05) {
        int gid = static_cast<int>(nodes.size());
        nodes.push_back(Node{gid, goal.pos});
        if (static_cast<int>(states.size()) <= gid) states.resize(gid + 1);
        states[gid].parent = v;
        states[gid].cost = compute_path_cost(gid);
        pq.push(PQItem{gid, states[gid].cost});
      }
    }
  }

  RCLCPP_DEBUG(
    node_->get_logger(), "[GraphPlanner] expansions=%d nodes=%zu goal_node=%d", expansions,
    nodes.size(), goal_node);

  if (goal_node == -1) {
    // 見つからない場合は目標に最も近いノードを採用
    double best_d = 1e9;
    int best_id = -1;
    for (size_t i = 0; i < nodes.size(); ++i) {
      double d = (nodes[i].p - goal.pos).norm();
      if (d < best_d) {
        best_d = d;
        best_id = static_cast<int>(i);
      }
    }
    goal_node = best_id;
  }

  if (goal_node < 0) {
    // フォールバック: 直線2点の経路
    RCLCPP_WARN(node_->get_logger(), "[GraphPlanner] goal_node<0. Fallback to straight line.");
    std::vector<Point> pts{start.pos, goal.pos};
    return buildWaypointsWithVelocities(pts, v0, limits);
  }

  // 最終経路の復元（states 参照に備えサイズ防御）
  if (static_cast<int>(states.size()) <= goal_node) states.resize(goal_node + 1);
  std::vector<Point> path_pts;
  for (int cur = goal_node; cur >= 0;) {
    path_pts.push_back(nodes[cur].p);
    if (static_cast<int>(states.size()) <= cur) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[GraphPlanner] states out-of-range: cur=%d states_size=%zu. Abort backtrack.", cur,
        states.size());
      cur = -1;
    } else {
      cur = states[cur].parent;
    }
  }
  std::reverse(path_pts.begin(), path_pts.end());

  // 可視化（任意）
  // 始点・終点
  viz_->circle()
    .center(start.pos)
    .radius(0.05)
    .stroke("green", 0.8)
    .strokeWidth(6)
    .fill("green", 0.2)
    .build();
  viz_->circle()
    .center(goal.pos)
    .radius(0.05)
    .stroke("green", 0.8)
    .strokeWidth(6)
    .fill("green", 0.2)
    .build();
  // 経路はポリラインで描画
  {
    auto poly = viz_->polyline().stroke("red", 1.0).strokeWidth(40);
    for (const auto & p : path_pts) {
      poly.addPoint(p);
    }
    poly.build();
  }
  // ウェイポイントの速度ベクトル（短い矢印）
  auto wps = buildWaypointsWithVelocities(path_pts, v0, limits);
  for (const auto & wp : wps) {
    Point to = wp.position + wp.target_velocity * 0.15;  // スケール係数
    viz_->line().start(wp.position).end(to).stroke("orange", 0.9).strokeWidth(6).build();
  }
  // フラッシュはフレーム末にRVO2側でまとめて実行

  return buildWaypointsWithVelocities(path_pts, v0, limits);
}

}  // namespace crane
