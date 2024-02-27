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
  explicit GridMapPlanner(rclcpp::Node & node)
  : map({"penalty", "ball_placement", "theirs", "ours", "ball"})
  {
    node.declare_parameter("map_resolution", MAP_RESOLUTION);
    MAP_RESOLUTION = node.get_parameter("map_resolution").as_double();

    gridmap_publisher =
      node.create_publisher<grid_map_msgs::msg::GridMap>("local_planner/grid_map", 1);
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(1.2, 2.0), MAP_RESOLUTION, grid_map::Position(0.0, 0.0));
  }

  std::vector<AStarNode> findPathAStar(
    const Point & start_point, const Point & goal_point, const std::string & layer) const
  {
    std::priority_queue<AStarNode> openSet;
    std::unordered_map<grid_map::Index, double, EigenArrayHash, EigenArrayEqual> costSoFar;
    std::vector<AStarNode> path;

    AStarNode start;
    map.getIndex(start_point, start.index);

    AStarNode goal;
    map.getIndex(goal_point, goal.index);

    start.heuristic = start.calcHeuristic(goal.index);
    openSet.push(start);
    costSoFar[start.index] = 0.;

    while (!openSet.empty()) {
      AStarNode current = openSet.top();
      openSet.pop();

      if (current.index.x() == goal.index.x() && current.index.y() == goal.index.y()) {
        while (current.parent != nullptr) {
          path.push_back(current);
          current = *current.parent;
        }
        break;
      }

      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          if (dx == 0 && dy == 0) continue;  // Skip the current node
          AStarNode next;
          next.index = current.index + grid_map::Index(dx, dy);

          if (!map.isValid(next.index) || map.at(layer, next.index) < 0.f)
            continue;  // Check for obstacles or out of bounds

          next.heuristic = next.calcHeuristic(goal.index) + map.at(layer, next.index);
          next.cost = costSoFar[current.index] + 1;

          if (!costSoFar.count(next.index) || next.cost < costSoFar[next.index]) {
            costSoFar[next.index] = next.cost;
            next.parent = &current;
            openSet.push(next);
          }
        }
      }
    }

    std::reverse(path.begin(), path.end());
    return path;
  }

  crane_msgs::msg::RobotCommands calculateRobotCommand(
    const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model)
  {
    // update map size

    if (
      map.getLength().x() != world_model->field_size.x() ||
      map.getLength().y() != world_model->field_size.y()) {
      map.clearAll();
      map.setGeometry(
        grid_map::Length(world_model->field_size.x(), world_model->field_size.y()), MAP_RESOLUTION);
    }

    // DefenseSize更新時にdefense_areaを更新する
    static Vector2 defense_area_size;
    if (
      defense_area_size.x() != world_model->defense_area_size.x() ||
      defense_area_size.y() != world_model->defense_area_size.y()) {
      defense_area_size = world_model->defense_area_size;
      if (not map.exists("defense_area")) {
        map.add("defense_area");
      }

      for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
        grid_map::Position position;
        map.getPosition(*iterator, position);
        map.at("defense_area", *iterator) = world_model->isDefenseArea(position);
      }
    }

    // ボールプレイスメントMap
    if (not map.exists("ball_placement")) {
      map.add("ball_placement");
    }
    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
      grid_map::Position position;
      map.getPosition(*iterator, position);
      map.at("ball_placement", *iterator) = world_model->isBallPlacementArea(position);
    }

    // 味方ロボットMap
    if (not map.exists("friend_robot")) {
      map.add("friend_robot");
    }
    map["friend_robot"].setConstant(0.0);
    for (const auto & robot : world_model->ours.getAvailableRobots()) {
      for (grid_map::CircleIterator iterator(map, robot->pose.pos, 0.1); !iterator.isPastEnd();
           ++iterator) {
        map.at("friend_robot", *iterator) = 1.0;
      }
    }

    // 敵ロボットMap
    if (not map.exists("enemy_robot")) {
      map.add("enemy_robot");
    }
    map["enemy_robot"].setConstant(0.0);
    for (const auto & robot : world_model->theirs.getAvailableRobots()) {
      for (grid_map::CircleIterator iterator(map, robot->pose.pos, 0.1); !iterator.isPastEnd();
           ++iterator) {
        map.at("enemy_robot", *iterator) = 1.0;
      }
    }

    // ボールMap
    if (not map.exists("ball")) {
      map.add("ball");
    }
    map["ball"].setConstant(0.0);
    for (grid_map::CircleIterator iterator(map, world_model->ball.pos, 0.1); !iterator.isPastEnd();
         ++iterator) {
      map.at("ball", *iterator) = 1.0;
    }

    // ボールMap (時間)
    if (not map.exists("ball_time")) {
      map.add("ball_time");
    }
    Vector2 ball_vel_unit = world_model->ball.vel.normalized() * MAP_RESOLUTION;
    Point ball_pos = world_model->ball.pos;
    float time = 0.f;
    const double TIME_STEP = MAP_RESOLUTION / world_model->ball.vel.norm();
    map["ball_time"].setConstant(100.0);
    for (int i = 0; i < 100; ++i) {
      for (grid_map::CircleIterator iterator(map, ball_pos, 0.05); !iterator.isPastEnd();
           ++iterator) {
        map.at("ball_time", *iterator) = std::min(map.at("ball_time", *iterator), time);
      }
      ball_pos += ball_vel_unit;
      time += TIME_STEP;
    }
    //      map.setTimestamp(now().nanoseconds());
    std::unique_ptr<grid_map_msgs::msg::GridMap> message;
    message = grid_map::GridMapRosConverter::toMessage(map);

    gridmap_publisher->publish(std::move(message));

    // TODO(HansRobo): implement
    crane_msgs::msg::RobotCommands commands = msg;
    for (auto & command : commands.robot_commands) {
      if ((not command.target_x.empty()) && (not command.target_y.empty())) {
        auto robot = world_model->getOurRobot(command.robot_id);
        Point target;
        target << command.target_x.front(), command.target_y.front();
        if (map.exists("cost")) {
          map.add("cost", 0.f);
        } else {
          map.get("cost").setZero();
        }

        if (not command.local_planner_config.disable_collision_avoidance) {
          map.get("cost") += map.get("friend_robot");
          // delete current robot position
          for (grid_map::CircleIterator iterator(map, robot->pose.pos, 0.15); !iterator.isPastEnd();
               ++iterator) {
            map.at("cost", *iterator) = 0.;
          }

          map.get("cost") += map.get("enemy_robot");
        }

        if (not command.local_planner_config.disable_ball_avoidance) {
          map.get("cost") += map.get("ball");
        }

        if (not command.local_planner_config.disable_goal_area_avoidance) {
          map.get("cost") += map.get("defense_area");
        }

        if (not command.local_planner_config.disable_placement_avoidance) {
          map.get("cost") += map.get("ball_placement");
        }

        auto route = findPathAStar(robot->pose.pos, target, "cost");

        // velocity planning
        // 最終位置・速度から速度計画
        // 現在位置・速度から速度計画
        // 次の位置・速度でコマンドを上書き
      }
    }
    return commands;
  }

private:
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_publisher;

  grid_map::GridMap map;

  double MAP_RESOLUTION = 0.05;
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__GRIDMAP_PLANNER_HPP_
