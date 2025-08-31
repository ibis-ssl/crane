// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/graph_planner.hpp"

#include <algorithm>

namespace crane
{

GraphPlanner::GraphPlanner(
  rclcpp::Node & node, WorldModelWrapper::SharedPtr world_model,
  VisualizerMessageBuilder::SharedPtr visualizer)
: node_(&node), world_(std::move(world_model)), viz_(std::move(visualizer))
{
  reloadParamsFromROS();
}

void GraphPlanner::reloadParamsFromROS()
{
  // Declare-once, get current values each call
  node_->declare_parameter("graph_planner.max_expansion", params_.max_expansion);
  node_->declare_parameter("graph_planner.node_tangent_offset", params_.node_tangent_offset);
  node_->declare_parameter("graph_planner.edge_clearance_eps", params_.edge_clearance_eps);

  node_->declare_parameter("graph_planner.robot_radius", params_.robot_radius);
  node_->declare_parameter("graph_planner.static_margin", params_.static_margin);
  node_->declare_parameter("graph_planner.K_alpha", params_.K_alpha);
  node_->declare_parameter("graph_planner.K_v", params_.K_v);
  node_->declare_parameter("graph_planner.K_t", params_.K_t);
  node_->declare_parameter("graph_planner.far_margin_cap", params_.far_margin_cap);

  bool use_time_cost = (params_.cost_mode == CostMode::Time);
  node_->declare_parameter("graph_planner.use_time_cost", use_time_cost);  // 時間ベースDijkstraの有効化

  params_.max_expansion = node_->get_parameter("graph_planner.max_expansion").as_int();
  params_.node_tangent_offset =
    node_->get_parameter("graph_planner.node_tangent_offset").as_double();
  params_.edge_clearance_eps = node_->get_parameter("graph_planner.edge_clearance_eps").as_double();

  params_.robot_radius = node_->get_parameter("graph_planner.robot_radius").as_double();
  params_.static_margin = node_->get_parameter("graph_planner.static_margin").as_double();
  params_.K_alpha = node_->get_parameter("graph_planner.K_alpha").as_double();
  params_.K_v = node_->get_parameter("graph_planner.K_v").as_double();
  params_.K_t = node_->get_parameter("graph_planner.K_t").as_double();
  params_.far_margin_cap = node_->get_parameter("graph_planner.far_margin_cap").as_double();

  use_time_cost = node_->get_parameter("graph_planner.use_time_cost").as_bool();
  params_.cost_mode = use_time_cost ? CostMode::Time : CostMode::Distance;
}

auto GraphPlanner::boxToPolygon(const Box & b) -> Polygon
{
  Polygon poly;
  auto minc = b.min_corner();
  auto maxc = b.max_corner();
  poly.outer().push_back(Point(minc.x(), minc.y()));
  poly.outer().push_back(Point(maxc.x(), minc.y()));
  poly.outer().push_back(Point(maxc.x(), maxc.y()));
  poly.outer().push_back(Point(minc.x(), maxc.y()));
  poly.outer().push_back(Point(minc.x(), minc.y()));
  return poly;
}

auto GraphPlanner::intersects(const Segment & seg, const CircleObstacle & c) -> bool
{
  // 円と線分の距離が半径以下なら交差
  return bg::distance(seg, c.center) <= c.radius;
}

auto GraphPlanner::intersects(const Segment & seg, const BoxObstacle & b) -> bool
{
  // Segment と Box の交差は intersects で直接判定
  return bg::intersects(seg, b.box);
}

auto GraphPlanner::intersects(const Segment & seg, const Obstacle & ob) -> bool
{
  if (std::holds_alternative<CircleObstacle>(ob)) {
    return intersects(seg, std::get<CircleObstacle>(ob));
  } else {
    return intersects(seg, std::get<BoxObstacle>(ob));
  }
}

auto GraphPlanner::intersectsAny(const Segment & seg, const std::vector<Obstacle> & obs) -> bool
{
  for (const auto & ob : obs) {
    if (intersects(seg, ob)) return true;
  }
  return false;
}

// Eq.(2) helper: compute dynamic margin and displacement horizon (∆t)
static inline double compute_dt(double r, double K_v, double K_t)
{
  return std::min(r / std::max(1e-6, K_v), K_t);
}

auto GraphPlanner::buildObstacles(const Pose2D & start) -> std::vector<Obstacle>
{
  std::vector<Obstacle> obs;

  const auto & wm = world_;
  const Point start_pos = start.pos;

  // Robots (our + their), except the robot at start position (closest id unknown -> skip only if very close)
  auto push_robot_circle = [&](const RobotInfo::SharedPtr & r) {
    Point c = r->pose.pos;
    // Skip if this is likely the moving robot
    if ((c - start_pos).norm() < params_.robot_radius * 0.5) return;

    double r_dist = (c - start_pos).norm();
    double dt = compute_dt(r_dist, params_.K_v, params_.K_t);
    // predicted displacement ld = ∆t * v_r
    Point ld = r->vel.linear * dt;
    Point pred_center = c + ld;
    // margin lm = Ms + 1/2 Kα ∆t^2
    double lm = params_.static_margin + 0.5 * params_.K_alpha * dt * dt;
    lm = std::min(lm, params_.far_margin_cap);

    CircleObstacle co{pred_center, params_.robot_radius + lm};
    obs.emplace_back(co);
  };

  for (const auto & rr : wm->ours().robots) {
    if (rr->available) push_robot_circle(rr);
  }
  for (const auto & rr : wm->theirs().robots) {
    if (rr->available) push_robot_circle(rr);
  }

  // Penalty areas as inflated boxes
  auto inflate_box = [&](const Box & box, double margin) {
    Box bb = box;
    bb.min_corner().x() -= margin;
    bb.min_corner().y() -= margin;
    bb.max_corner().x() += margin;
    bb.max_corner().y() += margin;
    return bb;
  };
  obs.emplace_back(BoxObstacle{inflate_box(world_->getOurPenaltyArea(), params_.static_margin)});
  obs.emplace_back(BoxObstacle{inflate_box(world_->getTheirPenaltyArea(), params_.static_margin)});

  // フィールド境界: 障害物としては追加せず、展開時にフィールド外となる候補を棄却する

  // 可視化: 障害物アウトライン
  if (viz_) {
    auto draw_box = [&](const Box & bb, const std::string & color, double alpha, int width) {
      auto minc = bb.min_corner();
      auto maxc = bb.max_corner();
      Point c1(minc.x(), minc.y());
      Point c2(maxc.x(), minc.y());
      Point c3(maxc.x(), maxc.y());
      Point c4(minc.x(), maxc.y());
      viz_->line().start(c1).end(c2).stroke(color, alpha).strokeWidth(width).build();
      viz_->line().start(c2).end(c3).stroke(color, alpha).strokeWidth(width).build();
      viz_->line().start(c3).end(c4).stroke(color, alpha).strokeWidth(width).build();
      viz_->line().start(c4).end(c1).stroke(color, alpha).strokeWidth(width).build();
    };
    for (const auto & o : obs) {
      if (std::holds_alternative<CircleObstacle>(o)) {
        const auto & co = std::get<CircleObstacle>(o);
        viz_->circle().center(co.center).radius(co.radius).stroke("magenta", 0.3).strokeWidth(4).fill("magenta", 0.05).build();
      } else {
        const auto & bo = std::get<BoxObstacle>(o);
        draw_box(bo.box, "magenta", 0.25, 4);
      }
    }
  }

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
  double d = std::sqrt(d2);
  // 標準式により接点を計算
  // T = C + (r^2/d^2) * u +/- (r * sqrt(d^2 - r^2) / d^2) * perp(u)
  double l = (r * r) / d2;
  double h = r * std::sqrt(std::max(0.0, d2 - r * r)) / d2;
  Point perp;
  perp.x() = -u.y();
  perp.y() = u.x();

  Point t1 = c.center + l * u + h * perp;
  Point t2 = c.center + l * u - h * perp;

  // 数値衝突を避けるため、円外側へ微小オフセット
  Point n1 = (t1 - c.center).normalized();
  Point n2 = (t2 - c.center).normalized();
  t1 += n1 * 1.0 * 1e-3 + n1 * 1.0 * 0.0;  // minimal epsilon
  t2 += n2 * 1.0 * 1e-3 + n2 * 1.0 * 0.0;

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

  // For each blocking obstacle, compute candidate points
  for (const auto & ob : blocking) {
    if (std::holds_alternative<CircleObstacle>(ob)) {
      const auto & co = std::get<CircleObstacle>(ob);
      for (const auto & tp : tangentPointsFromPointToCircle(from, co)) {
        // 辺方向に僅かに進めた点を候補とする（数値安定性）
        Point dir = (tp - from).normalized();
        Point cand = tp + dir * params_.node_tangent_offset;

        // from→cand が障害物と交差しないこと
        Segment e(from, cand);
        if (intersectsAny(e, obstacles)) continue;

        // フィールド内であること
        if (!world_->point_checker.isFieldInside(cand, 0.0)) continue;

        int id = static_cast<int>(nodes.size());
        nodes.push_back(Node{id, cand});
        new_node_ids.push_back(id);
      }
    } else {
      // ボックス障害物: 角から外方向にオフセットした点を候補にする
      const auto & bb = std::get<BoxObstacle>(ob).box;
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
        int id = static_cast<int>(nodes.size());
        nodes.push_back(Node{id, around});
        new_node_ids.push_back(id);
      }
    }
  }
  return new_node_ids;
}

auto GraphPlanner::computeDistanceCostPath(const std::vector<Node> & nodes, int end_node_id) const
  -> double
{
  // 親を辿って経路の長さを評価（未使用: 実コストはmainループで計算）
  (void)nodes;
  (void)end_node_id;
  return 0.0;  // Placeholder; actual distance costs computed inline in main loop where parent is known
}

// Implements Eq.(1) time calculation along a sequence of points
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
  const int N = Np - 1;  // number of edges
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
      Lp[n] = std::clamp(0.0, 0.0, 0.0);  // initialize
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

    // vm per Eq.(1):
    // vm = min( sqrt( 2 αa αb (ℓ + ℓp) + αb v0^2 ) / (αa + αb), vmax )
    double num = 2.0 * limits.alpha_acc * limits.alpha_dec * (L[n] + Lp[n]) +
                 limits.alpha_dec * v0n[n] * v0n[n];
    double den = limits.alpha_acc + limits.alpha_dec;
    vm[n] = std::min(std::sqrt(std::max(0.0, num)) / std::max(1e-6, den), limits.vmax);

    // ve per Eq.(1): ve = min( sqrt(v0^2 + 2 αa ℓ), sqrt(2 αb ℓp) )
    double ve_acc = std::sqrt(std::max(0.0, v0n[n] * v0n[n] + 2.0 * limits.alpha_acc * L[n]));
    double ve_dec = std::sqrt(std::max(0.0, 2.0 * limits.alpha_dec * Lp[n]));
    ve[n] = std::min({vm[n], ve_acc, ve_dec});
  }

  // 各ウェイポイントでの到達速度ベクトル（当該区間の終端速度）を付与
  wps.reserve(Np);
  for (int i = 0; i < Np; ++i) {
    Velocity v = Velocity::Zero();
    if (i == 0) {
      // initial point: use projection of v0 along first edge, capped by ve[0]
      if (N > 0 && L[0] > 1e-6) v = (e[0] / L[0]) * std::min(ve[0], limits.vmax);
    } else if (i - 1 < N && L[i - 1] > 1e-6) {
      v = (e[i - 1] / L[i - 1]) * std::min(ve[i - 1], limits.vmax);
    }
    wps.push_back(Waypoint{path_points[i], v});
  }
  return wps;
}

auto GraphPlanner::computeTimeCostPath(
  const std::vector<Node> & nodes, int end_node_id, const Velocity & v0, const Constraints & limits)
  const -> double
{
  // 親追跡でのポリライン再構築は呼出側で実施（ここでは未使用）
  (void)nodes;
  (void)end_node_id;
  (void)v0;
  (void)limits;
  // Time cost will be computed in-line by constructing the candidate polyline and using Eq.(1)
  return 0.0;
}

auto GraphPlanner::plan(
  const Pose2D & start, const Pose2D & goal, const Velocity & v0, const Constraints & limits)
  -> std::vector<Waypoint>
{
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
    if (params_.cost_mode == CostMode::Distance) {
      double sum = 0.0;
      for (size_t i = 1; i < pts.size(); ++i) sum += (pts[i] - pts[i - 1]).norm();
      return sum;
    } else {
      // 時間コスト: 式(1)に基づき各区間の厳密時間（台形/三角）を合算
      auto wps = buildWaypointsWithVelocities(pts, v0, limits);
      double Tsum = 0.0;
      for (size_t i = 1; i < wps.size(); ++i) {
        const double v_in = wps[i - 1].target_velocity.norm();
        const double v_out = wps[i].target_velocity.norm();
        const double L = (wps[i].position - wps[i - 1].position).norm();

        const double a = std::max(1e-6, limits.alpha_acc);
        const double b = std::max(1e-6, limits.alpha_dec);
        const double num = 2.0 * a * b * L + b * v_in * v_in + a * v_out * v_out;
        const double den = a + b;
        const double v_peak = std::sqrt(std::max(0.0, num / std::max(1e-6, den)));
        const double vm_lim = limits.vmax;

        if (v_peak <= vm_lim + 1e-6) {
          // 三角プロフィール
          const double t_acc = std::max(0.0, (v_peak - v_in) / a);
          const double t_dec = std::max(0.0, (v_peak - v_out) / b);
          Tsum += t_acc + t_dec;
        } else {
          // 台形プロフィール（巡航あり）
          const double s_acc = std::max(0.0, (vm_lim * vm_lim - v_in * v_in) / (2.0 * a));
          const double s_dec = std::max(0.0, (vm_lim * vm_lim - v_out * v_out) / (2.0 * b));
          const double s_cruise = std::max(0.0, L - s_acc - s_dec);
          const double t_acc = std::max(0.0, (vm_lim - v_in) / a);
          const double t_dec = std::max(0.0, (vm_lim - v_out) / b);
          const double t_cruise = (vm_lim > 1e-6) ? (s_cruise / vm_lim) : 0.0;
          Tsum += t_acc + t_cruise + t_dec;
        }
      }
      return Tsum;
    }
  };

  // 簡易な訪問済み抑制（粗いグリッドハッシュ）
  std::unordered_map<long long, int> seen;  // hashed grid -> node id (coarse)
  auto hash_point = [](const Point & p) {
    // coarse grid 2 cm
    long long xi = static_cast<long long>(std::round(p.x() * 50.0));
    long long yi = static_cast<long long>(std::round(p.y() * 50.0));
    return (xi << 32) ^ (yi & 0xffffffff);
  };

  seen[hash_point(start.pos)] = 0;

  while (!pq.empty() && expansions < params_.max_expansion) {
    auto top = pq.top();
    pq.pop();
    int u = top.node_id;
    // Goal reached?
    if ((nodes[u].p - goal.pos).norm() < 1e-2) {
      goal_node = u;
      break;
    }

    // Expand neighbors
    auto neighbors = expandFrom(u, nodes[u].p, goal.pos, obstacles, nodes);
    for (int v : neighbors) {
      // 中点がフィールド外となる辺は棄却
      Point mid = 0.5 * (nodes[u].p + nodes[v].p);
      if (!world_->point_checker.isFieldInside(mid, 0.0)) continue;

      // 近傍重複の抑制
      long long h = hash_point(nodes[v].p);
      if (seen.find(h) != seen.end()) continue;
      seen[h] = v;

      // 親を設定し、終端vまでの経路でコストを評価
      if ((int)states.size() <= v) states.resize(v + 1);
      states[v].parent = u;
      states[v].cost = compute_path_cost(v);
      pq.push(PQItem{v, states[v].cost});

      expansions++;

      // 目標に十分近ければ終端にゴールノードを追加
      if ((nodes[v].p - goal.pos).norm() < 0.05) {
        int gid = static_cast<int>(nodes.size());
        nodes.push_back(Node{gid, goal.pos});
        if ((int)states.size() <= gid) states.resize(gid + 1);
        states[gid].parent = v;
        states[gid].cost = compute_path_cost(gid);
        pq.push(PQItem{gid, states[gid].cost});
      }
    }
  }

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
    std::vector<Point> pts{start.pos, goal.pos};
    return buildWaypointsWithVelocities(pts, v0, limits);
  }

  // 最終経路の復元
  std::vector<Point> path_pts;
  for (int cur = goal_node; cur >= 0;) {
    path_pts.push_back(nodes[cur].p);
    cur = states[cur].parent;
  }
  std::reverse(path_pts.begin(), path_pts.end());

  // 可視化（任意）
  if (viz_) {
    // 始点・終点
    viz_->circle().center(start.pos).radius(0.05).stroke("green", 0.8).strokeWidth(6).fill("green", 0.2).build();
    viz_->circle().center(goal.pos).radius(0.05).stroke("green", 0.8).strokeWidth(6).fill("green", 0.2).build();
    for (size_t i = 1; i < path_pts.size(); ++i) {
      viz_->line().start(path_pts[i - 1]).end(path_pts[i]).stroke("cyan", 0.6).strokeWidth(8).build();
    }
    // ウェイポイントの速度ベクトル（短い矢印）
    auto wps = buildWaypointsWithVelocities(path_pts, v0, limits);
    for (const auto & wp : wps) {
      Point to = wp.position + wp.target_velocity * 0.15;  // スケール係数
      viz_->line().start(wp.position).end(to).stroke("orange", 0.9).strokeWidth(6).build();
    }
  }

  return buildWaypointsWithVelocities(path_pts, v0, limits);
}

}  // namespace crane
