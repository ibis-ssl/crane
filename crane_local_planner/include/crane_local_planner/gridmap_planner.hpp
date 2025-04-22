// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__GRIDMAP_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__GRIDMAP_PLANNER_HPP_

#include <algorithm>
#include <crane_basics/pid_controller.hpp>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <functional>
#include <grid_map_ros/grid_map_ros.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "planner_base.hpp"

namespace crane
{
// Eigen::Arrayのためのカスタム等価性判定関数
struct EigenArrayEqual
{
  auto operator()(const Eigen::Array<int, 2, 1> & a, const Eigen::Array<int, 2, 1> & b) const
    -> bool
  {
    // 全要素が等しいかどうかを判断
    return (a == b).all();
  }
};

// Eigen::Arrayのためのカスタムハッシュ関数
struct EigenArrayHash
{
  auto operator()(const Eigen::Array<int, 2, 1> & array) const -> std::size_t
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
  grid_map::Index index;

  double g;

  double h;

  std::optional<grid_map::Index> parent_index = std::nullopt;

  [[nodiscard]] auto calcHeuristic(const grid_map::Index & goal_index) const -> double
  {
    return std::hypot(index.x() - goal_index.x(), index.y() - goal_index.y());
  }

  auto getScore() const -> float { return g + h; }

  auto operator<(const AStarNode & other) const -> bool { return getScore() < other.getScore(); }
};

class GridMapPlanner : public LocalPlannerBase
{
public:
  explicit GridMapPlanner(rclcpp::Node & node);

  auto findPathAStar(
    const Point & start_point, const Point & goal_point, const std::string & layer,
    const uint8_t robot_id) const -> std::vector<grid_map::Index>;

  auto calculateRobotCommand(const crane_msgs::msg::RobotCommands & msg, double theta_offset)
    -> crane_msgs::msg::RobotCommands override;

private:
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_publisher;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;

  grid_map::GridMap map;

  double MAP_RESOLUTION = 0.05;

  double MAX_VEL = 4.0;
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__GRIDMAP_PLANNER_HPP_
