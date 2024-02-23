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
#include <utility>

namespace crane
{
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

  crane_msgs::msg::RobotCommands calculateRobotCommand(
    const crane_msgs::msg::RobotCommands &, WorldModelWrapper::SharedPtr world_model)
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
    return crane_msgs::msg::RobotCommands();
  }

private:
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_publisher;

  grid_map::GridMap map;

  double MAP_RESOLUTION = 0.05;
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__GRIDMAP_PLANNER_HPP_
