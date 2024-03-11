// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/gridmap_planner.hpp"

#include <nav_msgs/msg/path.hpp>

constexpr static int debug_id = 1;

namespace crane
{
GridMapPlanner::GridMapPlanner(rclcpp::Node & node)
: map({"penalty", "ball_placement", "theirs", "ours", "ball", "path/0"})
{
  node.declare_parameter("map_resolution", MAP_RESOLUTION);
  MAP_RESOLUTION = node.get_parameter("map_resolution").as_double();

  gridmap_publisher =
    node.create_publisher<grid_map_msgs::msg::GridMap>("local_planner/grid_map", 1);
  map.setFrameId("map");
  map.setGeometry(grid_map::Length(1.2, 2.0), MAP_RESOLUTION, grid_map::Position(0.0, 0.0));

  path_publisher = node.create_publisher<nav_msgs::msg::Path>("local_planner/path", 1);
}

std::vector<grid_map::Index> GridMapPlanner::findPathAStar(
  const Point & start_point, const Point & goal_point, const std::string & layer,
  const uint8_t robot_id)
{
  auto isMapInside = [&](const grid_map::Index & index) -> bool {
    grid_map::Position p;
    return map.getPosition(index, p);
  };

  auto isObstacle = [&](const grid_map::Index & index) -> bool {
    return map.at(layer, index) >= 0.5f;
  };

  // 注意：コストでソートするためにAstarNodeをKeyにしている
  std::multimap<AStarNode, grid_map::Index> openSet;
  std::unordered_map<grid_map::Index, AStarNode, EigenArrayHash, EigenArrayEqual> closedSet;

  AStarNode start;
  map.getIndex(start_point, start.index);
  if (not isMapInside(start.index)) {
    if (robot_id == debug_id) {
      std::cout << "start is not in the map" << std::endl;
    }
    return {};
  }

  // 脱出モード：障害物の中にいる場合障害物の外に出るまで、障害物関係なく経路を探索する
  bool escape_mode = isObstacle(start.index);

  AStarNode goal;
  map.getIndex(goal_point, goal.index);

  // ゴールが障害物内にある場合、最寄りの障害物外の点を探索してゴールとする
  auto find_alternative_goal = [&](double search_radius) -> grid_map::Index {
    for (grid_map::SpiralIterator goal_candidate(map, goal_point, search_radius);
         !goal_candidate.isPastEnd(); ++goal_candidate) {
      if (isMapInside(*goal_candidate) and not isObstacle(*goal_candidate)) {
        return *goal_candidate;
      }
    }
    return goal.index;
  };

  if (not isMapInside(goal.index)) {
    if (robot_id == debug_id) {
      std::cout << "goal is not in the map. replace goal" << std::endl;
    }
    auto alternative_goal = find_alternative_goal(1.0);
    if (alternative_goal.x() != goal.index.x() or alternative_goal.y() != goal.index.y()) {
      goal.index = alternative_goal;
    } else {
      if (robot_id == debug_id) {
        std::cout << "failed to find alternative goal" << std::endl;
      }
      return {};
    }
  } else if (isObstacle(goal.index)) {
    if (robot_id == debug_id) {
      std::cout << "goal is in obstacle" << std::endl;
    }
    auto alternative_goal = find_alternative_goal(1.0);
    if (alternative_goal.x() != goal.index.x() or alternative_goal.y() != goal.index.y()) {
      goal.index = alternative_goal;
    } else {
      if (robot_id == debug_id) {
        std::cout << "failed to find alternative goal" << std::endl;
      }
      return {};
    }
  }

  start.h = start.calcHeuristic(goal.index);
  start.g = 0.;
  openSet.emplace(start, start.index);

  if (not map.exists("closed")) {
    map.add("closed");
  }
  map["closed"].setZero();

  while (!openSet.empty()) {
    // openリストの先頭の要素を取得＆pop
    auto current = openSet.begin()->first;
    openSet.erase(openSet.begin());

    closedSet[current.index] = current;

    // ゴール判定
    if (current.index.x() == goal.index.x() && current.index.y() == goal.index.y()) {
      if (robot_id == debug_id) {
        //        std::cout << "スタートとゴールが同じマス内にあります" << std::endl;
      }
      // ゴールからスタートまでの経路を取得
      std::vector<grid_map::Index> path;
      path.emplace_back(current.index);
      while (current.parent_index) {
        path.push_back(current.parent_index.value());
        current = closedSet[current.parent_index.value()];
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    // 8方のマスをOpen
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;  // 自分はスキップ
        AStarNode next;
        next.index = current.index + grid_map::Index(dx, dy);
        next.parent_index = current.index;
        next.g = current.g + std::hypot(dx, dy);
        next.h = next.calcHeuristic(goal.index);

        // マップ外ならスキップ
        if (not isMapInside(next.index)) continue;

        // 障害物以外なら進む
        if (not escape_mode && isObstacle(next.index)) continue;

        // 脱出モードを更新
        if (escape_mode) {
          escape_mode = isObstacle(next.index);
        }

        if (robot_id == debug_id) {
          // std::cout << "next index: " << next.index.x() << ", " << next.index.y() << std::endl;
        }

        // closedSetとopenSetに含まれていない場合のみ追加
        if (
          closedSet.count(next.index) == 0 &&
          std::find_if(openSet.begin(), openSet.end(), [index = next.index](const auto & elem) {
            return elem.second.x() == index.x() && elem.second.y() == index.y();
          }) == openSet.end()) {
          openSet.emplace(next, next.index);
        }
      }
    }
  }
  std::cout << "openSet is empty" << std::endl;
  return {};
}
crane_msgs::msg::RobotCommands GridMapPlanner::calculateRobotCommand(
  const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model)
{
  // update map size

  static Vector2 defense_area_size;

  if (
    map.getLength().x() != world_model->field_size.x() ||
    map.getLength().y() != world_model->field_size.y()) {
    map.clearAll();
    map.setGeometry(
      grid_map::Length(world_model->field_size.x(), world_model->field_size.y()), MAP_RESOLUTION);
    defense_area_size << 0, 0;
  }

  // DefenseSize更新時にdefense_areaを更新する

  if (
    defense_area_size.x() != world_model->defense_area_size.x() ||
    defense_area_size.y() != world_model->defense_area_size.y()) {
    std::cout << "update defense_area" << std::endl;
    defense_area_size = world_model->defense_area_size;
    if (not map.exists("defense_area")) {
      map.add("defense_area");
    }
    map["defense_area"].setZero();

    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
      grid_map::Position position;
      map.getPosition(*iterator, position);
      map.at("defense_area", *iterator) = world_model->isDefenseArea(position) ? 1.f : 0.f;
    }
  }

  // ボールプレイスメントMap
  if (not map.exists("ball_placement")) {
    map.add("ball_placement");
  }
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    map.getPosition(*iterator, position);
    map.at("ball_placement", *iterator) = world_model->isBallPlacementArea(position, 0.2);
  }

  // 味方ロボットMap
  if (not map.exists("friend_robot")) {
    map.add("friend_robot");
  }
  map["friend_robot"].setZero();
  for (const auto & robot : world_model->ours.getAvailableRobots()) {
    for (grid_map::CircleIterator iterator(map, robot->pose.pos, 0.3); !iterator.isPastEnd();
         ++iterator) {
      map.at("friend_robot", *iterator) = 1.0;
    }
  }

  // 敵ロボットMap
  if (not map.exists("enemy_robot")) {
    map.add("enemy_robot");
  }
  map["enemy_robot"].setZero();
  for (const auto & robot : world_model->theirs.getAvailableRobots()) {
    for (grid_map::CircleIterator iterator(map, robot->pose.pos, 0.3); !iterator.isPastEnd();
         ++iterator) {
      map.at("enemy_robot", *iterator) = 1.0;
    }
  }

  // ボールMap
  if (not map.exists("ball")) {
    map.add("ball");
  }
  map["ball"].setZero();
  for (grid_map::CircleIterator iterator(map, world_model->ball.pos, 0.2); !iterator.isPastEnd();
       ++iterator) {
    map.at("ball", *iterator) = 1.0;
  }

  // ボールMap (時間)
  if (not map.exists("ball_time")) {
    map.add("ball_time");
  }
  map["ball_time"].setZero();
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

  crane_msgs::msg::RobotCommands commands = msg;
  for (auto & command : commands.robot_commands) {
    if ((not command.target_x.empty()) && (not command.target_y.empty())) {
      auto robot = world_model->getOurRobot(command.robot_id);
      Point target;
      target << command.target_x.front(), command.target_y.front();
      std::string map_name = "cost/" + std::to_string(command.robot_id);
      if (not map.exists(map_name)) {
        map.add(map_name);
      }
      map[map_name].setZero();

      if (not command.local_planner_config.disable_collision_avoidance) {
        map[map_name] += map.get("friend_robot");
        // delete current robot position
        for (grid_map::CircleIterator iterator(map, robot->pose.pos, 0.2); !iterator.isPastEnd();
             ++iterator) {
          map.at(map_name, *iterator) = 0.;
        }

        map[map_name] += map["enemy_robot"];
      }

      if (not command.local_planner_config.disable_ball_avoidance) {
        map[map_name] += map["ball"];
      }

      if (not command.local_planner_config.disable_goal_area_avoidance) {
        map[map_name] += map["defense_area"];
      }

      if (not command.local_planner_config.disable_placement_avoidance) {
        map[map_name] += map["ball_placement"];
      }

      auto route = findPathAStar(robot->pose.pos, target, map_name, command.robot_id);

      std::vector<Point> path;
      for (const auto & node : route) {
        Point p;
        map.getPosition(node, p);
        path.push_back(p);
      }

      if (path.size() < 2) {
        path.push_back(robot->pose.pos);
        path.push_back(target);
      }

      if (command.robot_id == debug_id) {
        if (not map.exists("path/0")) {
          map.add("path/0", 0.f);
        }
        map.get("path/0").setZero();
        float cost = 0.5f;
        for (const auto & p : path) {
          grid_map::Index index;
          map.getIndex(p, index);
          map.at("path/0", index) = cost;
          cost += 0.1f;
        }
      }

      const double a = 0.5;
      const double b = 0.8;

      auto smooth_path = path;

      for (int l = 0; l < 3; l++) {
        for (int i = 0; i < static_cast<int>(smooth_path.size()); i++) {
          smooth_path[i] = smooth_path[i] - a * (smooth_path[i] - path[i]);
        }
        for (int i = 1; i < static_cast<int>(smooth_path.size()) - 1; i++) {
          smooth_path[i] =
            smooth_path[i] - b * (2 * smooth_path[i] - smooth_path[i - 1] - smooth_path[i + 1]);
        }
      }

      if (command.robot_id == debug_id) {
        nav_msgs::msg::Path path_msg;
        for (const auto & p : smooth_path) {
          geometry_msgs::msg::PoseStamped pose;
          pose.pose.position.x = p.x();
          pose.pose.position.y = p.y();
          path_msg.poses.push_back(pose);
        }
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = rclcpp::Time(0);
        path_publisher->publish(path_msg);
      }

      std::vector<double> velocity(smooth_path.size(), 0.0);
      velocity[0] = robot->vel.linear.norm();
      velocity.back() = command.local_planner_config.terminal_velocity;

      // 最終速度を考慮した速度
      for (int i = static_cast<int>(smooth_path.size()) - 2; i > 0; i--) {
        double distance = (smooth_path[i + 1] - smooth_path[i]).norm();
        velocity[i] = std::min(
          std::sqrt(
            velocity[i + 1] * velocity[i + 1] +
            2 * command.local_planner_config.max_acceleration * distance),
          static_cast<double>(command.local_planner_config.max_velocity));
      }
      //      std::cout << std::endl;
      //      std::cout << "2. ";
      //      for (const auto & v : velocity) {
      //        std::cout << v << ", ";
      //      }
      //      std::cout << std::endl;

      // 現在速度を考慮した速度
      for (int i = 1; i < static_cast<int>(smooth_path.size()); i++) {
        double distance = (smooth_path[i] - smooth_path[i - 1]).norm();
        velocity[i] = std::min(
          velocity[i], std::sqrt(
                         velocity[i - 1] * velocity[i - 1] +
                         2 * command.local_planner_config.max_acceleration * distance));
      }
      //      std::cout << "3. ";
      //      for (const auto & v : velocity) {
      //        std::cout << v << ", ";
      //      }
      //      std::cout << std::endl;

      command.target_x.clear();
      command.target_y.clear();
      command.target_x.push_back(smooth_path[1].x());
      command.target_y.push_back(smooth_path[1].y());
      //      std::cout << "ID: " << static_cast<int>(command.robot_id) << " target: "
      //                << smooth_path[1].x() << ", " << smooth_path[1].y() << ", velocity: "
      //                << velocity[1] << std::endl;
      Velocity global_vel = (smooth_path[1] - robot->pose.pos).normalized() * velocity[1];

      command.target_velocity.x = global_vel.x();
      command.target_velocity.y = global_vel.y();
    }
  }
  return commands;
}
}  // namespace crane
