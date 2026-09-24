// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/visibility_graph.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>

namespace crane::visibility_graph
{
namespace
{
constexpr double EPSILON = 1e-9;

// isEdgeVisible が障害物との食い込みを判定する際の許容量。
// ノードの間引き（plan 内）と同じ値を使うことで、判定が完全に一致する。
constexpr double EDGE_CLEARANCE_EPSILON = 1e-6;

auto closestPointOnSegment(const Segment & segment, const Point & point) -> Point
{
  const Vector2 direction = segment.second - segment.first;
  const double squared_length = direction.squaredNorm();
  if (squared_length < EPSILON) {
    return segment.first;
  }
  const double ratio =
    std::clamp((point - segment.first).dot(direction) / squared_length, 0.0, 1.0);
  return segment.first + ratio * direction;
}

auto distanceToSegment(const Segment & segment, const Point & point) -> double
{
  return (point - closestPointOnSegment(segment, point)).norm();
}

// 以下は isEdgeVisible の内側だけで使う。辺の総数が O(N^2)、障害物ごとに判定するため
// この3関数が計画時間のほぼ全部を占める。boost.geometry の汎用版は同じ計算をするが
// ディスパッチと sqrt が乗るので、ここだけ平方距離の自前実装に置き換えている。
// 判定は「距離 < しきい値」なので、両辺を2乗しても符号は変わらない。

auto squaredDistancePointToSegment(const Point & a, const Point & b, const Point & point) -> double
{
  const Vector2 direction = b - a;
  const double squared_length = direction.squaredNorm();
  if (squared_length < EPSILON) {
    return (point - a).squaredNorm();
  }
  const double ratio = std::clamp((point - a).dot(direction) / squared_length, 0.0, 1.0);
  return (point - (a + ratio * direction)).squaredNorm();
}

// 2線分間の最近接距離の2乗。Ericson, Real-Time Collision Detection 5.1.9 と同じ手順。
auto squaredDistanceSegmentToSegment(
  const Point & p1, const Point & q1, const Point & p2, const Point & q2) -> double
{
  const Vector2 d1 = q1 - p1;
  const Vector2 d2 = q2 - p2;
  const Vector2 r = p1 - p2;
  const double a = d1.squaredNorm();
  const double e = d2.squaredNorm();
  const double f = d2.dot(r);

  double s = 0.0;
  double t = 0.0;
  if (a < EPSILON && e < EPSILON) {
    return r.squaredNorm();
  }
  if (a < EPSILON) {
    t = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = d1.dot(r);
    if (e < EPSILON) {
      s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = d1.dot(d2);
      const double denom = a * e - b * b;
      s = denom > EPSILON ? std::clamp((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
      } else if (t > 1.0) {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }
  return ((p1 + d1 * s) - (p2 + d2 * t)).squaredNorm();
}

// 線分と軸平行矩形の交差判定（Liang-Barsky のスラブ法）。接触も交差として扱う。
auto segmentIntersectsBox(const Point & a, const Point & b, const Box & box) -> bool
{
  const Vector2 direction = b - a;
  double t_enter = 0.0;
  double t_exit = 1.0;
  for (int axis = 0; axis < 2; ++axis) {
    const double lower = box.min_corner()[axis];
    const double upper = box.max_corner()[axis];
    if (std::abs(direction[axis]) < EPSILON) {
      if (a[axis] < lower || a[axis] > upper) {
        return false;
      }
      continue;
    }
    const double inverse = 1.0 / direction[axis];
    double near_hit = (lower - a[axis]) * inverse;
    double far_hit = (upper - a[axis]) * inverse;
    if (near_hit > far_hit) {
      std::swap(near_hit, far_hit);
    }
    t_enter = std::max(t_enter, near_hit);
    t_exit = std::min(t_exit, far_hit);
    if (t_enter > t_exit) {
      return false;
    }
  }
  return true;
}
}  // namespace

auto Obstacle::makeCircle(const Point & center, double radius) -> Obstacle
{
  Obstacle obstacle;
  obstacle.type = Type::CIRCLE;
  obstacle.circle = Circle{center, radius};
  return obstacle;
}

auto Obstacle::makeBox(const Box & box) -> Obstacle
{
  Obstacle obstacle;
  obstacle.type = Type::BOX;
  obstacle.box = box;
  return obstacle;
}

auto Obstacle::makeCapsule(const Capsule & capsule, bool dynamic_robot) -> Obstacle
{
  Obstacle obstacle;
  obstacle.type = Type::CAPSULE;
  obstacle.capsule = capsule;
  obstacle.escape_capsule = capsule;
  obstacle.is_dynamic_robot = dynamic_robot;
  return obstacle;
}

auto Obstacle::signedDistance(const Point & point) const -> double
{
  switch (type) {
    case Type::CIRCLE:
      return (point - circle.center).norm() - circle.radius;
    case Type::CAPSULE:
      return distanceToSegment(capsule.segment, point) - capsule.radius;
    case Type::BOX: {
      const double xmin = box.min_corner().x();
      const double xmax = box.max_corner().x();
      const double ymin = box.min_corner().y();
      const double ymax = box.max_corner().y();
      const double dx = std::max({xmin - point.x(), 0.0, point.x() - xmax});
      const double dy = std::max({ymin - point.y(), 0.0, point.y() - ymax});
      if (dx > 0.0 || dy > 0.0) {
        return std::hypot(dx, dy);
      }
      return -std::min({point.x() - xmin, xmax - point.x(), point.y() - ymin, ymax - point.y()});
    }
  }
  return 0.0;
}

auto Obstacle::projectOutside(const Point & point, double clearance) const -> Point
{
  switch (type) {
    case Type::CIRCLE: {
      Vector2 direction = point - circle.center;
      if (direction.norm() < EPSILON) {
        direction = Vector2::UnitX();
      }
      return circle.center + direction.normalized() * (circle.radius + clearance);
    }
    case Type::CAPSULE: {
      const Point closest = closestPointOnSegment(capsule.segment, point);
      Vector2 direction = point - closest;
      if (direction.norm() < EPSILON) {
        const Vector2 axis = capsule.segment.second - capsule.segment.first;
        direction = axis.norm() < EPSILON ? Vector2::UnitX() : Vector2(-axis.y(), axis.x());
      }
      return closest + direction.normalized() * (capsule.radius + clearance);
    }
    case Type::BOX: {
      const double xmin = box.min_corner().x();
      const double xmax = box.max_corner().x();
      const double ymin = box.min_corner().y();
      const double ymax = box.max_corner().y();
      if (signedDistance(point) >= 0.0) {
        return point;
      }
      const std::array<std::pair<double, Point>, 4> candidates = {{
        {point.x() - xmin, Point(xmin - clearance, point.y())},
        {xmax - point.x(), Point(xmax + clearance, point.y())},
        {point.y() - ymin, Point(point.x(), ymin - clearance)},
        {ymax - point.y(), Point(point.x(), ymax + clearance)},
      }};
      return std::min_element(
               candidates.begin(), candidates.end(),
               [](const auto & lhs, const auto & rhs) { return lhs.first < rhs.first; })
        ->second;
    }
  }
  return point;
}

auto VisibilityGraph::computeObstacleBounds(const std::vector<Obstacle> & obstacles)
  -> std::vector<ObstacleBounds>
{
  std::vector<ObstacleBounds> bounds;
  bounds.reserve(obstacles.size());
  for (const auto & obstacle : obstacles) {
    ObstacleBounds box;
    switch (obstacle.type) {
      case Obstacle::Type::CIRCLE:
        box.min_x = obstacle.circle.center.x() - obstacle.circle.radius;
        box.max_x = obstacle.circle.center.x() + obstacle.circle.radius;
        box.min_y = obstacle.circle.center.y() - obstacle.circle.radius;
        box.max_y = obstacle.circle.center.y() + obstacle.circle.radius;
        break;
      case Obstacle::Type::CAPSULE: {
        const auto & first = obstacle.capsule.segment.first;
        const auto & second = obstacle.capsule.segment.second;
        box.min_x = std::min(first.x(), second.x()) - obstacle.capsule.radius;
        box.max_x = std::max(first.x(), second.x()) + obstacle.capsule.radius;
        box.min_y = std::min(first.y(), second.y()) - obstacle.capsule.radius;
        box.max_y = std::max(first.y(), second.y()) + obstacle.capsule.radius;
        break;
      }
      case Obstacle::Type::BOX:
        box.min_x = obstacle.box.min_corner().x();
        box.max_x = obstacle.box.max_corner().x();
        box.min_y = obstacle.box.min_corner().y();
        box.max_y = obstacle.box.max_corner().y();
        break;
    }
    bounds.push_back(box);
  }
  return bounds;
}

auto VisibilityGraph::isEdgeVisible(
  const Point & from, const Point & to, const std::vector<Obstacle> & obstacles,
  const std::vector<ObstacleBounds> & bounds) const -> bool
{
  // 十分に近いなら干渉していないとみなす
  if ((to - from).norm() < EPSILON) {
    return true;
  }

  const double edge_min_x = std::min(from.x(), to.x());
  const double edge_max_x = std::max(from.x(), to.x());
  const double edge_min_y = std::min(from.y(), to.y());
  const double edge_max_y = std::max(from.y(), to.y());
  for (size_t index = 0; index < obstacles.size(); ++index) {
    const auto & box = bounds[index];

    // 作成した矩形境界と線分が重なっていなければスキップ。
    if (
      box.max_x < edge_min_x || box.min_x > edge_max_x || box.max_y < edge_min_y ||
      box.min_y > edge_max_y) {
      continue;
    }

    // 厳密チェック
    const auto & obstacle = obstacles[index];
    switch (obstacle.type) {
      case Obstacle::Type::CIRCLE: {
        const double threshold = obstacle.circle.radius - EDGE_CLEARANCE_EPSILON;
        if (
          threshold > 0.0 &&
          squaredDistancePointToSegment(from, to, obstacle.circle.center) < threshold * threshold) {
          return false;
        }
        break;
      }
      case Obstacle::Type::CAPSULE: {
        const double threshold = obstacle.capsule.radius - EDGE_CLEARANCE_EPSILON;
        if (
          threshold > 0.0 && squaredDistanceSegmentToSegment(
                               from, to, obstacle.capsule.segment.first,
                               obstacle.capsule.segment.second) < threshold * threshold) {
          return false;
        }
        break;
      }
      case Obstacle::Type::BOX:
        if (segmentIntersectsBox(from, to, obstacle.box)) {
          return false;
        }
        break;
    }
  }
  return true;
}

auto VisibilityGraph::generateNodes(const std::vector<Obstacle> & obstacles) const
  -> std::vector<Point>
{
  std::vector<Point> nodes;
  for (const auto & obstacle : obstacles) {
    switch (obstacle.type) {
      case Obstacle::Type::CIRCLE: {
        const int samples = std::max(config_.circle_samples, 6);
        const double outer_radius =
          obstacle.circle.radius / std::cos(M_PI / samples) + config_.node_clearance;
        for (int i = 0; i < samples; ++i) {
          const double angle = 2.0 * M_PI * static_cast<double>(i) / samples;
          nodes.emplace_back(
            obstacle.circle.center + outer_radius * Vector2(std::cos(angle), std::sin(angle)));
        }
        break;
      }
      case Obstacle::Type::CAPSULE: {
        const Vector2 axis = obstacle.capsule.segment.second - obstacle.capsule.segment.first;
        if (axis.norm() < EPSILON) {
          const auto circle =
            Obstacle::makeCircle(obstacle.capsule.segment.first, obstacle.capsule.radius);
          auto circle_nodes = generateNodes({circle});
          nodes.insert(nodes.end(), circle_nodes.begin(), circle_nodes.end());
          break;
        }
        const double axis_angle = std::atan2(axis.y(), axis.x());
        const int samples = std::max(config_.capsule_end_samples, 3);
        const double step = M_PI / samples;
        const double outer_radius =
          obstacle.capsule.radius / std::cos(step / 2.0) + config_.node_clearance;
        for (int i = 0; i <= samples; ++i) {
          const double front_angle = axis_angle - M_PI_2 + step * i;
          const double back_angle = axis_angle + M_PI_2 + step * i;
          nodes.emplace_back(
            obstacle.capsule.segment.second +
            outer_radius * Vector2(std::cos(front_angle), std::sin(front_angle)));
          nodes.emplace_back(
            obstacle.capsule.segment.first +
            outer_radius * Vector2(std::cos(back_angle), std::sin(back_angle)));
        }
        break;
      }
      case Obstacle::Type::BOX: {
        const Point center = (obstacle.box.min_corner() + obstacle.box.max_corner()) * 0.5;
        const std::array<Point, 4> corners = {{
          obstacle.box.min_corner(),
          Point(obstacle.box.max_corner().x(), obstacle.box.min_corner().y()),
          obstacle.box.max_corner(),
          Point(obstacle.box.min_corner().x(), obstacle.box.max_corner().y()),
        }};
        for (const auto & corner : corners) {
          nodes.push_back(corner + (corner - center).normalized() * config_.node_clearance);
        }
        break;
      }
    }
  }
  return nodes;
}

auto VisibilityGraph::plan(
  const Point & start, const Point & goal, const std::vector<Obstacle> & obstacles) const
  -> std::optional<std::vector<Point>>
{
  Point effective_start = start;
  Point effective_goal = goal;
  for (int iteration = 0; iteration < 16; ++iteration) {
    bool projected = false;
    for (const auto & obstacle : obstacles) {
      if (obstacle.signedDistance(effective_start) < 0.0) {
        effective_start = obstacle.projectOutside(effective_start, config_.node_clearance);
        projected = true;
      }
      if (obstacle.signedDistance(effective_goal) < 0.0) {
        effective_goal = obstacle.projectOutside(effective_goal, config_.node_clearance);
        projected = true;
      }
    }
    if (!projected) {
      break;
    }
  }

  const auto is_outside_all = [&obstacles](const Point & point) {
    return std::all_of(obstacles.begin(), obstacles.end(), [&point](const auto & obstacle) {
      return obstacle.signedDistance(point) >= -EPSILON;
    });
  };
  if (!is_outside_all(effective_start) || !is_outside_all(effective_goal)) {
    return std::nullopt;
  }

  // 障害物に遮られていない通常ケースでは、グラフ全辺の生成を避ける。
  const auto bounds = computeObstacleBounds(obstacles);
  if (isEdgeVisible(effective_start, effective_goal, obstacles, bounds)) {
    std::vector<Point> path{effective_start, effective_goal};
    if ((effective_start - start).norm() > 1e-6) {
      path.insert(path.begin(), start);
    } else {
      path.front() = start;
    }
    return path;
  }

  std::vector<Point> nodes{effective_start, effective_goal};
  const auto obstacle_nodes = generateNodes(obstacles);
  // 障害物の内側に入り込んだノードは捨てる。isEdgeVisible はノードの所属を問わず
  // 「辺が障害物に食い込んでいれば不可視」と判定するので、内側のノードへ向かう辺は
  // すべて不可視になり経路に採用されない。辺の総数は O(N^2) なので、ここで落とす
  // ほうが圧倒的に安い。混雑した盤面では隣接ロボットのカプセルに埋まるノードが多い。
  nodes.reserve(nodes.size() + obstacle_nodes.size());
  for (const auto & node : obstacle_nodes) {
    const bool buried =
      std::any_of(obstacles.begin(), obstacles.end(), [&node](const Obstacle & obstacle) {
        return obstacle.signedDistance(node) < -EDGE_CLEARANCE_EPSILON;
      });
    if (!buried) {
      nodes.push_back(node);
    }
  }

  // 可視グラフの全辺を先に作ると、辺の本数 O(N^2) × 障害物数の判定が必要になり、
  // 計画時間のほぼ全部を占める。そこで辺は A* が実際に展開したノードから
  // だけ遅延生成する。ヒューリスティックは目標までの直線距離で、可容かつ整合的
  // （辺コストがユークリッド距離なので三角不等式が成り立つ）なので、得られる経路長は
  // Dijkstra と同一である。
  //
  // さらに効くのが next_cost の枝刈り。直線距離だけで既知の距離を改善できないと
  // 分かった辺は、可視判定そのものを省ける。可視判定は障害物数に比例するため、
  // ここで落とせる分がそのまま効く。
  struct QueueEntry
  {
    double priority;
    size_t node;
    auto operator>(const QueueEntry & other) const -> bool { return priority > other.priority; }
  };
  const double infinity = std::numeric_limits<double>::infinity();
  const Point & goal_node = nodes[1];
  const auto heuristic = [&goal_node, &nodes](size_t index) {
    return (nodes[index] - goal_node).norm();
  };
  std::vector<double> distances(nodes.size(), infinity);
  std::vector<size_t> predecessors(nodes.size(), nodes.size());
  std::vector<bool> closed(nodes.size(), false);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
  distances[0] = 0.0;
  queue.push({heuristic(0), 0});
  while (!queue.empty()) {
    const auto [priority, node] = queue.top();
    queue.pop();
    if (closed[node]) {
      continue;
    }
    closed[node] = true;
    if (node == 1) {
      break;
    }
    const double node_distance = distances[node];
    for (size_t next = 0; next < nodes.size(); ++next) {
      if (next == node || closed[next]) {
        continue;
      }
      const double next_cost = node_distance + (nodes[next] - nodes[node]).norm();
      if (next_cost >= distances[next]) {
        continue;
      }
      if (!isEdgeVisible(nodes[node], nodes[next], obstacles, bounds)) {
        continue;
      }
      distances[next] = next_cost;
      predecessors[next] = node;
      queue.push({next_cost + heuristic(next), next});
    }
  }
  if (!std::isfinite(distances[1])) {
    return std::nullopt;
  }

  std::vector<Point> path;
  for (size_t node = 1; node < nodes.size(); node = predecessors[node]) {
    path.push_back(nodes[node]);
    if (node == 0) {
      break;
    }
  }
  std::reverse(path.begin(), path.end());
  if (path.empty() || (path.front() - effective_start).norm() > 1e-6) {
    return std::nullopt;
  }
  if ((effective_start - start).norm() > 1e-6) {
    path.insert(path.begin(), start);
  } else {
    path.front() = start;
  }
  return path;
}

auto VisibilityGraph::isPathVisible(
  const std::vector<Point> & path, const std::vector<Obstacle> & obstacles) const -> bool
{
  const auto bounds = computeObstacleBounds(obstacles);
  for (size_t i = 1; i < path.size(); ++i) {
    if (!isEdgeVisible(path[i - 1], path[i], obstacles, bounds)) {
      return false;
    }
  }
  return path.size() >= 2;
}

auto VisibilityGraph::nearestDynamicEscape(
  const Point & point, const std::vector<Obstacle> & obstacles, double inside_margin,
  double escape_clearance) const -> std::optional<Point>
{
  // 退避判定は escape_capsule（相手の実位置 + 相手自身の予測移動）で行う。
  // 経路計画用の capsule は自機速度ぶん自機側へ伸びているので、それで判定すると
  // 静止ロボットへ近づくだけで退避が発火し、減速→カプセル縮小→再加速→再発火を繰り返す
  // （2026-09-20 bag, robot 3 の OUR_DIRECT_FREE）。
  std::optional<Obstacle> nearest;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (const auto & obstacle : obstacles) {
    if (!obstacle.is_dynamic_robot) {
      continue;
    }
    const Obstacle shape = obstacle.type == Obstacle::Type::CAPSULE
                             ? Obstacle::makeCapsule(obstacle.escape_capsule, true)
                             : obstacle;
    const double signed_distance = shape.signedDistance(point);
    if (signed_distance >= inside_margin) {
      continue;
    }
    if (signed_distance < nearest_distance) {
      nearest = shape;
      nearest_distance = signed_distance;
    }
  }
  if (!nearest.has_value()) {
    return std::nullopt;
  }

  // 一番近い障害物の回避。解除余裕 inside_margin より外側へ出す。
  // 退避先が解除線とほぼ同じ距離（node_clearance は 1mm）だと、位置制御器の到達許容誤差
  // （CM4 は 10mm）の内側で止まった機体が解除線を越えられず、退避が固着する
  return nearest->projectOutside(
    point, config_.node_clearance + std::max({0.0, inside_margin, escape_clearance}));
}

auto pathLength(const std::vector<Point> & path) -> double
{
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    length += (path[i] - path[i - 1]).norm();
  }
  return length;
}

auto firstWaypointBeyond(const std::vector<Point> & path, double min_distance) -> WaypointChoice
{
  WaypointChoice choice;
  if (path.size() < 2) {
    choice.index = 0;
    return choice;
  }
  double arc = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    arc += (path[i] - path[i - 1]).norm();
    if (arc >= min_distance || i + 1 == path.size()) {
      choice.index = i;
      choice.arc_length = arc;
      return choice;
    }
  }
  choice.index = path.size() - 1;
  choice.arc_length = arc;
  return choice;
}

auto trimPathFromCurrent(const Point & current, const std::vector<Point> & path)
  -> std::vector<Point>
{
  if (path.size() < 2) {
    return {};
  }
  size_t best_segment = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  Point best_projection = current;
  for (size_t i = 1; i < path.size(); ++i) {
    const Point projection = closestPointOnSegment(Segment(path[i - 1], path[i]), current);
    const double distance = (current - projection).norm();
    if (distance < best_distance) {
      best_distance = distance;
      best_segment = i;
      best_projection = projection;
    }
  }
  std::vector<Point> result{current};
  // 射影点は現在位置から 1mm 程度しか離れていないことがあるが、この点が保持経路の可視性
  // （current -> path[k] が障害物をかすらないこと）を成立させている。目標としての退化は
  // firstWaypointBeyond 側で吸収するので、ここで閾値を上げてはいけない。
  if ((best_projection - current).norm() > 1e-4) {
    result.push_back(best_projection);
  }
  result.insert(result.end(), path.begin() + static_cast<std::ptrdiff_t>(best_segment), path.end());
  return result;
}

auto pointAtDistance(const std::vector<Point> & path, double distance) -> Point
{
  if (path.empty()) {
    return Point::Zero();
  }
  double remaining = std::max(0.0, distance);
  for (size_t i = 1; i < path.size(); ++i) {
    const Vector2 segment = path[i] - path[i - 1];
    const double length = segment.norm();
    if (length > 1e-9 && remaining <= length) {
      return path[i - 1] + segment * (remaining / length);
    }
    remaining -= length;
  }
  return path.back();
}

auto toString(SubgoalMode mode) -> const char *
{
  switch (mode) {
    case SubgoalMode::LOOKAHEAD:
      return "LOOKAHEAD";
    case SubgoalMode::WAYPOINT:
      return "WAYPOINT";
    case SubgoalMode::WAYPOINT_SKIP:
      return "WAYPOINT_SKIP";
    case SubgoalMode::PATH_POINT:
      return "PATH_POINT";
  }
  return "UNKNOWN";
}

auto selectSubgoal(
  const VisibilityGraph & graph, const Point & current, const std::vector<Point> & path,
  const std::vector<Obstacle> & obstacles, double lookahead_distance, double min_subgoal_distance)
  -> SubgoalChoice
{
  SubgoalChoice choice;
  const double remaining_distance = pathLength(path);
  // pointAtDistance は経路長を超える距離を渡すと終点を返すので、弧長は min(lookahead, 全長)
  choice.point = pointAtDistance(path, lookahead_distance);
  choice.arc_length = std::min(lookahead_distance, remaining_distance);
  choice.mode = SubgoalMode::LOOKAHEAD;
  if (path.size() < 2 || graph.isPathVisible({current, choice.point}, obstacles)) {
    return choice;
  }

  // path[1] は trimPathFromCurrent の射影点や plan() の押し出し始点で、現在位置から
  // 1mm 程度のことがある。そのまま目標にすると「今いる場所へ行け、ただし終端速度は
  // 1.4 m/s で」となり、終端速度ベクトルの向きだけがフレームごとに反転する
  // （2026-09-20 bag, robot 3）。現在位置から十分離れた最初の中継点を選ぶ。
  const auto waypoint = firstWaypointBeyond(path, min_subgoal_distance);
  choice.point = path[waypoint.index];
  choice.arc_length = waypoint.arc_length;
  choice.mode = waypoint.index == 1 ? SubgoalMode::WAYPOINT : SubgoalMode::WAYPOINT_SKIP;
  if (waypoint.index > 1 && !graph.isPathVisible({current, choice.point}, obstacles)) {
    // 中継点を読み飛ばすと current→path[k] の弦が path[1]→path[k] から横にずれ、
    // 障害物の角をかすめることがある。退化した path[1] へ戻すのではなく、
    // 経路（可視性が保証されている）に沿って min_subgoal_distance だけ進んだ点を目標にする
    choice.point = pointAtDistance(path, min_subgoal_distance);
    choice.arc_length = std::min(min_subgoal_distance, remaining_distance);
    choice.mode = SubgoalMode::PATH_POINT;
  }
  return choice;
}

auto decideReplanAction(bool has_safe_retained_path, bool direct_path_visible, bool full_replan_due)
  -> ReplanAction
{
  if (direct_path_visible) {
    return ReplanAction::USE_DIRECT_PATH;
  }
  if (has_safe_retained_path && !full_replan_due) {
    return ReplanAction::REUSE_RETAINED_PATH;
  }
  return ReplanAction::RUN_FULL_REPLAN;
}

auto makePredictedRobotObstacle(
  const Vector2 & ego_velocity, double ego_radius, const Point & other_position,
  const Vector2 & other_velocity, double other_radius, double prediction_horizon,
  double safety_margin) -> Obstacle
{
  const double horizon = std::max(0.0, prediction_horizon);
  const Vector2 relative_velocity = other_velocity - ego_velocity;
  const Point predicted = other_position + relative_velocity * horizon;
  const double radius =
    std::max(0.0, ego_radius) + std::max(0.0, other_radius) + std::max(0.0, safety_margin);
  auto obstacle = Obstacle::makeCapsule(Capsule{Segment(other_position, predicted), radius}, true);
  // 退避判定用の形状は相手自身の移動予測だけで作る。相対速度を使うと静止ロボットの
  // カプセルが自機速度ぶん自機側へ伸び、近づくだけで「食い込み」になる。
  const Point predicted_other_only = other_position + other_velocity * horizon;
  obstacle.escape_capsule = Capsule{Segment(other_position, predicted_other_only), radius};
  return obstacle;
}

}  // namespace crane::visibility_graph
