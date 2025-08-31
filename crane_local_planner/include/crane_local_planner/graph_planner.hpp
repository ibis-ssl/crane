// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__GRAPH_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__GRAPH_PLANNER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <variant>

namespace crane
{

// 経由点（ウェイポイント）: 位置と、その点に到達すべき目標速度
struct Waypoint
{
  Point position;
  Velocity target_velocity;  // 当該点で実現したい到達速度ベクトル
};

class GraphPlanner
{
public:
  using SharedPtr = std::shared_ptr<GraphPlanner>;

  // ロボットの運動制約（最大速度・加減速度）
  struct Constraints
  {
    double vmax{4.0};
    double alpha_acc{4.0};  // 加速度 (α_a)
    double alpha_dec{4.0};  // 減速度 (α_b)
  };

  // パラメータ（ROSパラメータから変更可能）
  struct Params
  {
    // 一般
    int max_expansion{300};
    double node_tangent_offset{0.03};  // 接線接地点からの微小オフセット[m]

    // 障害物モデル（式(2)）
    double robot_radius{0.09};
    double static_margin{0.10};  // Ms（静的障害物マージン）
    double K_alpha{1.0};         // Kα（加速を考慮する係数）
    double K_v{2.0};             // Kv（速度スケール）
    double K_t{0.6};             // Kt [s]（Δtの上限）
    double far_margin_cap{2.0};  // 遠方障害物のマージン上限[m]
  };

  explicit GraphPlanner(rclcpp::Node & node, WorldModelWrapper::SharedPtr world_model);

  // 動的障害物を考慮したグラフ探索により、始点→目標の経路を計画
  // v0: 現在のロボット速度（2D）、limits: ロボットの運動制約
  auto plan(
    const Pose2D & start, const Pose2D & goal, const Velocity & v0, const Constraints & limits)
    -> std::vector<Waypoint>;

  // パラメータをROSから再読込
  void reloadParamsFromROS();

private:
  using CircleObstacle = Circle;  // center, radius を持つ
  using BoxObstacle = Box;        // min/max corner を持つ
  using Obstacle = std::variant<CircleObstacle, BoxObstacle>;

  // 探索グラフのノード
  struct Node
  {
    int id;
    Point p;
  };

  struct PQItem
  {
    int node_id;
    double cost;  // 距離or時間コスト（経路依存）
    bool operator<(const PQItem & other) const { return cost > other.cost; }
  };

  struct PathState
  {
    double cost{std::numeric_limits<double>::infinity()};
    int parent{-1};
  };

  // コア補助関数群
  auto buildObstacles(const Pose2D & start) -> std::vector<Obstacle>;
  static auto intersectsAny(const Segment & seg, const std::vector<Obstacle> & obs) -> bool;
  static auto intersects(const Segment & seg, const Obstacle & ob) -> bool;
  static auto intersects(const Segment & seg, const CircleObstacle & c) -> bool;
  static auto intersects(const Segment & seg, const BoxObstacle & b) -> bool;

  // 接線生成とオンデマンド拡張
  static auto tangentPointsFromPointToCircle(const Point & p, const CircleObstacle & c)
    -> std::vector<Point>;
  auto expandFrom(
    int from_id, const Point & from, const Point & goal, const std::vector<Obstacle> & obstacles,
    std::vector<Node> & nodes) -> std::vector<int>;

  // ウェイポイントに達すべき速度を付与（式(1)）
  auto buildWaypointsWithVelocities(
    const std::vector<Point> & path_points, const Velocity & v0, const Constraints & limits) const
    -> std::vector<Waypoint>;

  // ユーティリティ
  static auto angleCosBetween(const Point & a, const Point & b) -> double;

private:
  rclcpp::Node * node_;
  WorldModelWrapper::SharedPtr world_;
  VisualizerMessageBuilder::SharedPtr viz_;
  Params params_;
};

}  // namespace crane

#endif  // CRANE_LOCAL_PLANNER__GRAPH_PLANNER_HPP_
