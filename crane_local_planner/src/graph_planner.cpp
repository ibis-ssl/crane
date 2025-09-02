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

// 円弧スムージング: コーナー点が円障害物の境界上にある場合、前後点を円へ射影し円弧で接続
static std::vector<Point> smooth_with_circle_arcs(
  const std::vector<Point> & path_pts, const std::vector<std::variant<Circle, Box>> & obstacles,
  const Point & goal, double arc_step_rad = 0.15)
{
  using CircleObs = Circle;
  std::vector<Point> out;
  if (path_pts.size() < 3) return path_pts;
  auto eq_pt = [](const Point & a, const Point & b, double eps = 1e-6) {
    return (a - b).norm() <= eps;
  };
  auto intersects_segment = [&](const Segment & seg, const std::variant<Circle, Box> & ob) {
    if (std::holds_alternative<CircleObs>(ob)) {
      const auto & c = std::get<CircleObs>(ob);
      return !getIntersections(c, seg).empty();
    } else {
      const auto & b = std::get<Box>(ob);
      return bg::intersects(seg, b);
    }
  };
  auto intersects_any_except = [&](const Segment & seg, const CircleObs & skip) {
    for (const auto & ob : obstacles) {
      if (std::holds_alternative<CircleObs>(ob)) {
        const auto & c = std::get<CircleObs>(ob);
        if (eq_pt(c.center, skip.center, 1e-6) && std::abs(c.radius - skip.radius) < 1e-6) continue;
        if (intersects_segment(seg, ob)) return true;
      } else {
        if (intersects_segment(seg, ob)) return true;
      }
    }
    return false;
  };
  auto find_circle_near = [&](const Point & p) -> std::optional<CircleObs> {
    double best_err = 1e9;
    std::optional<CircleObs> best;
    for (const auto & ob : obstacles) {
      if (!std::holds_alternative<CircleObs>(ob)) continue;
      const auto & c = std::get<CircleObs>(ob);
      double err = std::abs((p - c.center).norm() - c.radius);
      if (err < best_err) {
        best_err = err;
        best = c;
      }
    }
    if (best && best_err < 0.02) return best;  // 2cm以内なら円周上とみなす
    return std::nullopt;
  };

  out.push_back(path_pts.front());
  for (size_t i = 1; i + 1 < path_pts.size(); ++i) {
    const Point & prev = path_pts[i - 1];
    const Point & curr = path_pts[i];
    const Point & next = path_pts[i + 1];
    auto co = find_circle_near(curr);
    if (!co) {
      out.push_back(curr);
      continue;
    }
    const Point center = co->center;
    const double R = co->radius;
    // 射影関数
    auto proj = [&](const Point & x) {
      Point d = x - center;
      Point res;
      if (d.norm() < 1e-9) {
        res = center + Point(R, 0.0);
      } else {
        res = center + (d / d.norm()) * R;
      }
      return res;
    };
    Point A = proj(prev);
    Point B = proj(next);

    // ゴールから当該円への接線接点を求め、障害物非交差なものを採用
    auto tangents_from_point_to_circle = [&](const Point & p, const CircleObs & c) {
      std::vector<Point> list;
      Point u = p - c.center;
      double d2 = u.squaredNorm();
      double r = c.radius;
      if (d2 <= r * r) return list;  // 内側からは接線なし
      double l = (r * r) / d2;
      double h = r * std::sqrt(std::max(0.0, d2 - r * r)) / d2;
      Point perp = getVerticalVec(u);
      Point t1 = c.center + l * u + h * perp;
      Point t2 = c.center + l * u - h * perp;
      list.push_back(t1);
      list.push_back(t2);
      return list;
    };
    auto candidates = tangents_from_point_to_circle(goal, *co);
    std::optional<Point> tangent;
    if (!candidates.empty()) {
      // 非交差な接線接点のみ
      for (const auto & t : candidates) {
        Segment s(t, goal);
        if (!intersects_any_except(s, *co)) {
          tangent = t;
          break;
        }
      }
    }
    auto ang = [&](const Point & x) { return std::atan2((x - center).y(), (x - center).x()); };
    double a0 = ang(A);
    double a1 = tangent ? ang(*tangent) : ang(B);
    double da = a1 - a0;
    while (da > M_PI) da -= 2 * M_PI;
    while (da < -M_PI) da += 2 * M_PI;
    int steps = std::max(1, (int)std::ceil(std::abs(da) / std::max(0.05, arc_step_rad)));
    for (int k = 1; k <= steps; ++k) {
      double a = a0 + da * (double)k / (double)steps;
      Point p;
      p.x() = center.x() + R * std::cos(a);
      p.y() = center.y() + R * std::sin(a);
      out.push_back(p);
    }
    // 接線接点を通して以降は直線でgoal方向へ進むため、curr→nextの「円射影B」ではなく、
    // 接線接点が得られていればそれを優先して採用済み
  }
  out.push_back(path_pts.back());
  return out;
}

auto GraphPlanner::buildObstacles(uint8_t my_robot_id) -> std::vector<Obstacle>
{
  std::vector<Obstacle> obs;
  for (const auto & rr : world_->ours().getAvailableRobots(my_robot_id)) {
    obs.emplace_back(rr->geometry());
  }
  for (const auto & rr : world_->theirs().getAvailableRobots()) {
    obs.emplace_back(rr->geometry());
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

// ---- 疑似コード準拠の補助: 最初の交差障害物を取得（Step(2)前提）----
auto GraphPlanner::firstIntersection(
  const Point & from, const Point & to, const std::vector<Obstacle> & obstacles)
  -> std::optional<IntersectionInfo>
{
  Segment seg(from, to);
  const Point dir = to - from;
  const double len2 = dir.squaredNorm();
  if (len2 < 1e-12) return std::nullopt;

  auto point_t = [&](const Point & p) {
    // seg の始点からみたパラメータ t を概算（直交射影）
    double t = (dir.x() * (p.x() - from.x()) + dir.y() * (p.y() - from.y())) / len2;
    return std::clamp(t, 0.0, 1.0);
  };

  std::optional<IntersectionInfo> best;
  double best_t = 2.0;

  for (size_t i = 0; i < obstacles.size(); ++i) {
    const auto & ob = obstacles[i];
    if (std::holds_alternative<CircleObstacle>(ob)) {
      CircleObstacle c = std::get<CircleObstacle>(ob);
      c.radius = c.radius + 0.2;
      auto pts = getIntersections(c, seg);
      for (const auto & p : pts) {
        double t = point_t(p);
        if (t < best_t) {
          best_t = t;
          best = IntersectionInfo{i, t, p};
        }
      }
    } else {
      const auto & b = std::get<BoxObstacle>(ob);
      if (!bg::intersects(seg, b)) continue;
      // 箱の4辺との交点を調べる（seg vs seg のみを使用）
      auto minc = b.min_corner();
      auto maxc = b.max_corner();
      Point p00(minc.x(), minc.y());
      Point p10(maxc.x(), minc.y());
      Point p11(maxc.x(), maxc.y());
      Point p01(minc.x(), maxc.y());
      std::array<Segment, 4> edges{
        Segment(p00, p10), Segment(p10, p11), Segment(p11, p01), Segment(p01, p00)};
      for (const auto & e : edges) {
        std::vector<Point> ips;
        bg::intersection(seg, e, ips);
        for (const auto & p : ips) {
          double t = point_t(p);
          if (t < best_t) {
            best_t = t;
            best = IntersectionInfo{i, t, p};
          }
        }
      }
    }
  }

  return best;
}

auto GraphPlanner::boxCornersOutward(const BoxObstacle & bb, double offset) -> std::vector<Point>
{
  auto minc = bb.min_corner();
  auto maxc = bb.max_corner();
  std::vector<Point> corners{
    Point(minc.x(), minc.y()), Point(maxc.x(), minc.y()), Point(maxc.x(), maxc.y()),
    Point(minc.x(), maxc.y())};
  Point center = 0.5 * (minc + maxc);
  std::vector<Point> out;
  out.reserve(4);
  for (const auto & c : corners) {
    Point n = (c - center).normalized();
    out.push_back(c + n * offset);
  }
  return out;
}

auto GraphPlanner::getOrCreateNodeAt(const Point & p, std::vector<Node> & nodes) -> int
{
  for (size_t k = 0; k < nodes.size(); ++k) {
    if ((nodes[k].p - p).norm() <= params_.node_merge_epsilon) return static_cast<int>(k);
  }
  int id = static_cast<int>(nodes.size());
  nodes.push_back(Node{id, p});
  return id;
}

void GraphPlanner::expandUntilLineOfSight(
  const Point & from, const Point & goal, const std::vector<Obstacle> & obstacles,
  std::vector<int> & candidate_node_ids, std::vector<Node> & nodes, int depth, int max_depth)
{
  if (depth > max_depth) return;

  auto inter = firstIntersection(from, goal, obstacles);
  if (!inter) {
    // 直通: ゴールを候補に
    int gid = getOrCreateNodeAt(goal, nodes);
    candidate_node_ids.push_back(gid);
    return;
  }

  const auto & ob = obstacles[inter->obs_index];
  if (std::holds_alternative<CircleObstacle>(ob)) {
    const auto & co = std::get<CircleObstacle>(ob);
    // 接線接点（微小外押し）
    for (const auto & tp : tangentPointsFromPointToCircle(from, co)) {
      Point dir = (tp - from).normalized();
      Point cand = tp + dir * params_.node_tangent_offset;
      // from→cand が障害物と重なるなら除外
      if (intersectsAny(Segment(from, cand), obstacles)) continue;
      if (!world_->point_checker.isFieldInside(cand, 0.0)) continue;
      int id = getOrCreateNodeAt(cand, nodes);
      candidate_node_ids.push_back(id);
      // さらに先で直線が通るまで再帰的に候補を追加
      expandUntilLineOfSight(cand, goal, obstacles, candidate_node_ids, nodes, depth + 1, max_depth);
    }
  } else {
    // 矩形は簡易化: 角から外に僅かに出した点を候補に
    const auto & bb = std::get<BoxObstacle>(ob);
    for (const auto & cand : boxCornersOutward(bb, params_.node_tangent_offset)) {
      if (intersectsAny(Segment(from, cand), obstacles)) continue;
      if (!world_->point_checker.isFieldInside(cand, 0.0)) continue;
      int id = getOrCreateNodeAt(cand, nodes);
      candidate_node_ids.push_back(id);
      expandUntilLineOfSight(cand, goal, obstacles, candidate_node_ids, nodes, depth + 1, max_depth);
    }
  }
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
  const Pose2D & start, const Pose2D & goal, const Velocity & v0, const Constraints & limits,
  uint8_t my_robot_id)
  -> std::vector<Waypoint>
{
  // world_ は前提として有効

  RCLCPP_DEBUG(
    node_->get_logger(),
    "[GraphPlanner] plan start=(%.2f,%.2f) goal=(%.2f,%.2f) v0=%.2f vmax=%.2f acc=%.2f dec=%.2f",
    start.pos.x(), start.pos.y(), goal.pos.x(), goal.pos.y(), v0.norm(), limits.vmax, limits.alpha_acc,
    limits.alpha_dec);

  reloadParamsFromROS();

  // 障害物生成（自IDを除外）
  const auto obstacles = buildObstacles(my_robot_id);

  // ノード: 0=始点。ゴールは必要時に追加
  std::vector<Node> nodes;
  nodes.push_back(Node{0, start.pos});

  // Dijkstra（疑似コード準拠）
  std::priority_queue<PQItem> Open;  // 未確定集合
  std::vector<PathState> state;       // コストと親
  state.resize(1);
  state[0].cost = 0.0;
  state[0].parent = -1;

  std::vector<char> in_closed;
  in_closed.resize(1, 0);
  std::vector<int> closed_order;
  closed_order.push_back(0);  // Step(1): start を Closed に置く

  auto ensure_size = [&](int id) {
    if (id >= static_cast<int>(state.size())) state.resize(id + 1);
    if (id >= static_cast<int>(in_closed.size())) in_closed.resize(id + 1, 0);
  };

  int goal_node = -1;
  int expansions = 0;

  // ループ
  for (int iter = 0; iter < params_.max_expansion; ++iter) {
    int u = closed_order.back();  // Step(2)の前提: 直近で確定したノード

    // ---- 候補生成（Step(2)-(3)） ----
    std::vector<int> candidate;
    expandUntilLineOfSight(nodes[u].p, goal.pos, obstacles, candidate, nodes, 0, 8);

    // ---- 候補評価（Step(4)-(5)） ----
    for (int v : candidate) {
      ensure_size(v);
      if (in_closed[v]) continue;  // Closed にあるものは除外
      if (v == u) continue;        // 自己ループ回避
      // u→v の距離
      double w = (nodes[v].p - nodes[u].p).norm();
      if (!std::isfinite(w)) continue;
      double new_cost = state[u].cost + w;
      if (state[v].cost <= new_cost) {
        // 既存の方が良ければスキップ
      } else {
        state[v].cost = new_cost;
        state[v].parent = u;
        Open.push(PQItem{v, new_cost});
      }
      expansions++;
    }

    // ---- 次の確定（Step(6)） ----
    int x = -1;
    while (!Open.empty()) {
      auto top = Open.top();
      Open.pop();
      ensure_size(top.node_id);
      if (in_closed[top.node_id]) continue;  // 古い重複を破棄
      x = top.node_id;
      break;
    }
    if (x < 0) break;  // Open が空: 失敗
    in_closed[x] = 1;
    closed_order.push_back(x);

    // ゴール到達（ノードとして一致した場合）
    if ((nodes[x].p - goal.pos).norm() < 1e-6) {
      goal_node = x;
      break;
    }
  }

  RCLCPP_DEBUG(
    node_->get_logger(), "[GraphPlanner] expansions=%d nodes=%zu goal_node=%d", expansions,
    nodes.size(), goal_node);

  if (goal_node < 0) {
    // ゴールノードが生成されなかった場合、最も近い確定ノードを採用
    double best_d = 1e9;
    int best_id = -1;
    for (int id : closed_order) {
      double d = (nodes[id].p - goal.pos).norm();
      if (d < best_d) {
        best_d = d;
        best_id = id;
      }
    }
    goal_node = best_id;
  }

  if (goal_node < 0) {
    RCLCPP_WARN(node_->get_logger(), "[GraphPlanner] goal_node<0. Fallback to straight line.");
    std::vector<Point> pts{start.pos, goal.pos};
    return buildWaypointsWithVelocities(pts, v0, limits);
  }

  // 最終経路の復元
  std::vector<Point> path_pts;
  for (int cur = goal_node; cur >= 0;) {
    path_pts.push_back(nodes[cur].p);
    if (cur < 0 || cur >= static_cast<int>(state.size())) break;
    cur = state[cur].parent;
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
  // 円弧スムージング（接線と円を滑らかに接続）
  auto smooth_pts = smooth_with_circle_arcs(path_pts, obstacles, goal.pos, 0.15);
  // 経路はポリラインで描画
  {
    auto poly = viz_->polyline().stroke("cyan", 0.8).strokeWidth(20);
    for (const auto & p : smooth_pts) {
      poly.addPoint(p);
    }
    poly.build();
  }
  // ウェイポイントの速度ベクトル（短い矢印）
  auto wps = buildWaypointsWithVelocities(smooth_pts, v0, limits);
  for (const auto & wp : wps) {
    Point to = wp.position + wp.target_velocity * 0.15;  // スケール係数
    viz_->line().start(wp.position).end(to).stroke("orange", 0.9).strokeWidth(6).build();
  }
  // フラッシュはフレーム末にRVO2側でまとめて実行

  return buildWaypointsWithVelocities(smooth_pts, v0, limits);
}

}  // namespace crane
