// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__VISIBILITY_GRAPH_HPP_
#define CRANE_LOCAL_PLANNER__VISIBILITY_GRAPH_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <optional>
#include <vector>

namespace crane::visibility_graph
{

struct Obstacle
{
  enum class Type { CIRCLE, BOX, CAPSULE };

  Type type = Type::CIRCLE;
  Circle circle{Point::Zero(), 0.0};
  Box box{};
  Capsule capsule{};
  bool is_dynamic_robot = false;

  static auto makeCircle(const Point & center, double radius) -> Obstacle;
  static auto makeBox(const Box & box) -> Obstacle;
  static auto makeCapsule(const Capsule & capsule, bool is_dynamic_robot = false) -> Obstacle;

  [[nodiscard]] auto signedDistance(const Point & point) const -> double;
  [[nodiscard]] auto projectOutside(const Point & point, double clearance = 1e-3) const -> Point;
};

class VisibilityGraph
{
public:
  struct Config
  {
    int circle_samples = 12;
    int capsule_end_samples = 6;
    double node_clearance = 1e-3;
  };

  void configure(const Config & config) { config_ = config; }

  [[nodiscard]] auto plan(
    const Point & start, const Point & goal, const std::vector<Obstacle> & obstacles) const
    -> std::optional<std::vector<Point>>;

  [[nodiscard]] auto isPathVisible(
    const std::vector<Point> & path, const std::vector<Obstacle> & obstacles) const -> bool;

  [[nodiscard]] auto nearestDynamicEscape(
    const Point & point, const std::vector<Obstacle> & obstacles) const -> std::optional<Point>;

private:
  [[nodiscard]] auto isEdgeVisible(
    const Point & from, const Point & to, const std::vector<Obstacle> & obstacles) const -> bool;

  [[nodiscard]] auto generateNodes(const std::vector<Obstacle> & obstacles) const
    -> std::vector<Point>;

  Config config_;
};

enum class ReplanAction { USE_DIRECT_PATH, REUSE_RETAINED_PATH, RUN_FULL_REPLAN };

[[nodiscard]] auto decideReplanAction(
  bool has_safe_retained_path, bool direct_path_visible, bool full_replan_due) -> ReplanAction;

[[nodiscard]] auto pathLength(const std::vector<Point> & path) -> double;

[[nodiscard]] auto makePredictedRobotObstacle(
  const Vector2 & ego_velocity, double ego_radius, const Point & other_position,
  const Vector2 & other_velocity, double other_radius, double prediction_horizon,
  double safety_margin) -> Obstacle;

}  // namespace crane::visibility_graph

#endif  // CRANE_LOCAL_PLANNER__VISIBILITY_GRAPH_HPP_
