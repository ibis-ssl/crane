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

auto VisibilityGraph::isEdgeVisible(
  const Point & from, const Point & to, const std::vector<Obstacle> & obstacles) const -> bool
{
  if ((to - from).norm() < EPSILON) {
    return true;
  }
  const Segment edge(from, to);
  for (const auto & obstacle : obstacles) {
    switch (obstacle.type) {
      case Obstacle::Type::CIRCLE:
        if (distanceToSegment(edge, obstacle.circle.center) < obstacle.circle.radius - 1e-6) {
          return false;
        }
        break;
      case Obstacle::Type::CAPSULE:
        if (bg::distance(edge, obstacle.capsule.segment) < obstacle.capsule.radius - 1e-6) {
          return false;
        }
        break;
      case Obstacle::Type::BOX:
        if (bg::intersects(edge, obstacle.box)) {
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
  if (isEdgeVisible(effective_start, effective_goal, obstacles)) {
    std::vector<Point> path{effective_start, effective_goal};
    if ((effective_start - start).norm() > 1e-6) {
      path.insert(path.begin(), start);
    } else {
      path.front() = start;
    }
    return path;
  }

  std::vector<Point> nodes{effective_start, effective_goal};
  auto obstacle_nodes = generateNodes(obstacles);
  nodes.insert(nodes.end(), obstacle_nodes.begin(), obstacle_nodes.end());

  std::vector<std::vector<std::pair<size_t, double>>> edges(nodes.size());
  for (size_t from = 0; from < nodes.size(); ++from) {
    for (size_t to = from + 1; to < nodes.size(); ++to) {
      if (isEdgeVisible(nodes[from], nodes[to], obstacles)) {
        const double cost = (nodes[to] - nodes[from]).norm();
        edges[from].emplace_back(to, cost);
        edges[to].emplace_back(from, cost);
      }
    }
  }

  struct QueueEntry
  {
    double cost;
    size_t node;
    auto operator>(const QueueEntry & other) const -> bool { return cost > other.cost; }
  };
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> distances(nodes.size(), infinity);
  std::vector<size_t> predecessors(nodes.size(), nodes.size());
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
  distances[0] = 0.0;
  queue.push({0.0, 0});
  while (!queue.empty()) {
    const auto [cost, node] = queue.top();
    queue.pop();
    if (cost > distances[node]) {
      continue;
    }
    if (node == 1) {
      break;
    }
    for (const auto & [next, edge_cost] : edges[node]) {
      const double next_cost = cost + edge_cost;
      if (next_cost < distances[next]) {
        distances[next] = next_cost;
        predecessors[next] = node;
        queue.push({next_cost, next});
      }
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
  for (size_t i = 1; i < path.size(); ++i) {
    if (!isEdgeVisible(path[i - 1], path[i], obstacles)) {
      return false;
    }
  }
  return path.size() >= 2;
}

auto VisibilityGraph::nearestDynamicEscape(
  const Point & point, const std::vector<Obstacle> & obstacles) const -> std::optional<Point>
{
  const Obstacle * nearest = nullptr;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (const auto & obstacle : obstacles) {
    if (!obstacle.is_dynamic_robot || obstacle.signedDistance(point) >= 0.0) {
      continue;
    }
    const double distance = std::abs(obstacle.signedDistance(point));
    if (distance < nearest_distance) {
      nearest = &obstacle;
      nearest_distance = distance;
    }
  }
  if (nearest == nullptr) {
    return std::nullopt;
  }
  return nearest->projectOutside(point, config_.node_clearance);
}

auto pathLength(const std::vector<Point> & path) -> double
{
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    length += (path[i] - path[i - 1]).norm();
  }
  return length;
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
  const Vector2 relative_velocity = other_velocity - ego_velocity;
  const Point predicted = other_position + relative_velocity * std::max(0.0, prediction_horizon);
  const double radius =
    std::max(0.0, ego_radius) + std::max(0.0, other_radius) + std::max(0.0, safety_margin);
  return Obstacle::makeCapsule(Capsule{Segment(other_position, predicted), radius}, true);
}

}  // namespace crane::visibility_graph
