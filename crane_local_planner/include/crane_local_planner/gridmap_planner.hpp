// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__GRIDMAP_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__GRIDMAP_PLANNER_HPP_

#include <algorithm>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace crane
{
// Eigen::Arrayのためのカスタム等価性判定関数
struct EigenArrayEqual
{
  bool operator()(const Eigen::Array<int, 2, 1> & a, const Eigen::Array<int, 2, 1> & b) const
  {
    // 全要素が等しいかどうかを判断
    return (a == b).all();
  }
};

// Eigen::Arrayのためのカスタムハッシュ関数
struct EigenArrayHash
{
  std::size_t operator()(const Eigen::Array<int, 2, 1> & array) const
  {
    std::size_t seed = 0;
    for (int i = 0; i < array.size(); ++i) {
      // 各要素に基づいてハッシュ値を計算
      seed ^= std::hash<int>()(array[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};
struct AStarNode
{
  enum class NodeStatus : uint8_t { None, Open, Closed } status = NodeStatus::None;
  grid_map::Index index;
  double cost;
  double heuristic;
  AStarNode * parent;

  [[nodiscard]] double calcHeuristic(const grid_map::Index & goal_index) const
  {
    return std::abs(index.x() - goal_index.x()) + std::abs(index.y() - goal_index.y());
    //    return std::hypot(index.x() - goal_index.x(), index.y() - goal_index.y());
  }

  float totalCost() const { return cost + heuristic; }

  bool operator<(const AStarNode & other) const
  {
    return totalCost() >
           other.totalCost();  // Priority queue uses max heap, so we invert comparison
  }
};

class GridMapPlanner
{
public:
  explicit GridMapPlanner(rclcpp::Node & node);

  std::vector<AStarNode> findPathAStar(
    const Point & start_point, const Point & goal_point, const std::string & layer) const;

  crane_msgs::msg::RobotCommands calculateRobotCommand(
    const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model);

private:
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_publisher;

  grid_map::GridMap map;

  double MAP_RESOLUTION = 0.05;
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__GRIDMAP_PLANNER_HPP_
