// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/gridmap_planner.hpp"

#include <nav_msgs/msg/path.hpp>

constexpr static int debug_id = -1;

namespace crane
{
GridMapPlanner::GridMapPlanner(rclcpp::Node & node)
{
  node.declare_parameter("map_resolution", MAP_RESOLUTION);
  MAP_RESOLUTION = node.get_parameter("map_resolution").as_double();

  node.declare_parameter("max_vel", MAX_VEL);
  MAX_VEL = node.get_parameter("max_vel").as_double();

  node.declare_parameter("p_gain", P_GAIN);
  P_GAIN = node.get_parameter("p_gain").as_double();
  node.declare_parameter("i_gain", I_GAIN);
  I_GAIN = node.get_parameter("i_gain").as_double();
  node.declare_parameter("i_saturation", I_SATURATION);
  I_SATURATION = node.get_parameter("i_saturation").as_double();
  node.declare_parameter("d_gain", D_GAIN);
  D_GAIN = node.get_parameter("d_gain").as_double();

  for (auto & controller : vx_controllers) {
    controller.setGain(P_GAIN, I_GAIN, D_GAIN);
  }

  for (auto & controller : vy_controllers) {
    controller.setGain(P_GAIN, I_GAIN, D_GAIN, I_SATURATION);
  }

  visualizer = std::make_shared<ConsaiVisualizerWrapper>(node, "gridmap_local_planner");

  gridmap_publisher =
    node.create_publisher<grid_map_msgs::msg::GridMap>("local_planner/grid_map", 1);
  map.setFrameId("map");
  map.setGeometry(grid_map::Length(1.2, 2.0), MAP_RESOLUTION, grid_map::Position(0.0, 0.0));

  path_publisher = node.create_publisher<nav_msgs::msg::Path>("local_planner/path", 1);
}

std::vector<grid_map::Index> GridMapPlanner::findPathAStar(
  const Point & start_point, const Point & goal_point, const std::string & layer,
  const uint8_t robot_id) const
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
    auto alternative_goal = find_alternative_goal(3.0);
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

  while (!openSet.empty()) {
    // openリストの先頭の要素を取得＆pop
    auto current = openSet.begin()->first;
    openSet.erase(openSet.begin());

    closedSet[current.index] = current;

    // ゴール判定
    if (current.index.x() == goal.index.x() && current.index.y() == goal.index.y()) {
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
  return {};
}

crane_msgs::msg::RobotCommands GridMapPlanner::calculateRobotCommand(
  const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model)
{
  // update map size

  static Vector2 defense_area_size;

  if (
    map.getLength().x() != world_model->field_size.x() + world_model->getFieldMargin() * 2. ||
    map.getLength().y() != world_model->field_size.y() + world_model->getFieldMargin() * 2.) {
    map.clearAll();
    map.setGeometry(
      grid_map::Length(
        world_model->field_size.x() + world_model->getFieldMargin() * 2.,
        world_model->field_size.y() + world_model->getFieldMargin() * 2),
      MAP_RESOLUTION);
    defense_area_size << 0, 0;
  }

  // DefenseSize更新時にdefense_areaを更新する
  if (
    defense_area_size.x() != world_model->defense_area_size.x() ||
    defense_area_size.y() != world_model->defense_area_size.y()) {
    defense_area_size = world_model->defense_area_size;
    if (not map.exists("defense_area")) {
      map.add("defense_area");
    }
    map["defense_area"].setZero();

    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
      grid_map::Position position;
      map.getPosition(*iterator, position);
      map.at("defense_area", *iterator) = world_model->isDefenseArea(position) ? 1.f : 0.f;
      // ゴール後ろのすり抜けの防止
      if (
        std::abs(position.x()) > world_model->field_size.x() / 2. &&
        std::abs(position.y()) <= std::abs(world_model->ours.defense_area.max_corner().y())) {
        map.at("defense_area", *iterator) = 1.f;
      }
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

  // ルールMap
  if (not map.exists("rule")) {
    map.add("rule");
  }
  map["rule"].setZero();
  switch (world_model->play_situation.getSituationCommandID()) {
    case crane_msgs::msg::PlaySituation::STOP: {
      // 5.1.1 ボールと0.5m以上離れる必要がある
      for (grid_map::CircleIterator iterator(map, world_model->ball.pos, 0.6);
           !iterator.isPastEnd(); ++iterator) {
        map.at("rule", *iterator) = 1.0;
      }
    } break;
    case crane_msgs::msg::PlaySituation::OUR_BALL_PLACEMENT:
    case crane_msgs::msg::PlaySituation::THEIR_BALL_PLACEMENT: {
      grid_map::Position position;
      for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
        map.getPosition(*iterator, position);
        map.at("rule", *iterator) = world_model->isBallPlacementArea(position, 0.2);
      }
    } break;
      //    case crane_msgs::msg::PlaySituation::THEIR_KICKOFF_PREPARATION:
      //    formationに戻るので使わない
    case crane_msgs::msg::PlaySituation::THEIR_KICKOFF_START: {
      if (world_model->onPositiveHalf()) {
        grid_map::Position position;
        for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
          map.getPosition(*iterator, position);
          map.at("rule", *iterator) = position.x() < 0 ? 1.f : 0.f;
        }
      } else {
        grid_map::Position position;
        for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
          map.getPosition(*iterator, position);
          map.at("rule", *iterator) = position.x() > 0 ? 1.f : 0.f;
        }
      }
    } break;
    case crane_msgs::msg::PlaySituation::THEIR_PENALTY_PREPARATION:
    case crane_msgs::msg::PlaySituation::THEIR_PENALTY_START: {
      // 敵PKなので、全員敵陣側に行く
      if (world_model->onPositiveHalf()) {
        // 自陣が正なので敵陣は負
        double x_threshold = world_model->ball.pos.x() - 1.0;
        grid_map::Position position;
        for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
          map.getPosition(*iterator, position);
          map.at("rule", *iterator) = x_threshold > position.x() ? 0.f : 1.f;
        }
      } else {
        // 自陣が負なので敵陣は正
        double x_threshold = world_model->ball.pos.x() + 1.0;
        grid_map::Position position;
        for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
          map.getPosition(*iterator, position);
          map.at("rule", *iterator) = x_threshold < position.x() ? 0.f : 1.f;
        }
      }
    } break;
    case crane_msgs::msg::PlaySituation::OUR_PENALTY_PREPARATION:
    case crane_msgs::msg::PlaySituation::OUR_PENALTY_START: {
      // 味方PKなので、全員味方陣側に行く
      if (world_model->onPositiveHalf()) {
        // みんな正
        double x_threshold = world_model->ball.pos.x() + 1.0;
        grid_map::Position position;
        for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
          map.getPosition(*iterator, position);
          map.at("rule", *iterator) = x_threshold < position.x() ? 0.f : 1.f;
        }
      } else {
        // みんな負
        double x_threshold = world_model->ball.pos.x() - 1.0;
        grid_map::Position position;
        for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
          map.getPosition(*iterator, position);
          map.at("rule", *iterator) = x_threshold > position.x() ? 0.f : 1.f;
        }
      }
    } break;
  }

  //  // ボールMap (時間)
  //  if (not map.exists("ball_time")) {
  //    map.add("ball_time");
  //  }

  //  map["ball_time"].setConstant(100.0);
  //  Vector2 ball_vel_unit = world_model->ball.vel.normalized() * MAP_RESOLUTION;
  //  Point ball_pos = world_model->ball.pos;
  //  float time = 0.f;
  //  const double TIME_STEP = MAP_RESOLUTION / world_model->ball.vel.norm();
  //  for (int i = 0; i < 100; ++i) {
  //    for (grid_map::CircleIterator iterator(map, ball_pos, 0.05); !iterator.isPastEnd();
  //         ++iterator) {
  //      map.at("ball_time", *iterator) = std::min(map.at("ball_time", *iterator), time);
  //    }
  //    ball_pos += ball_vel_unit;
  //    time += TIME_STEP;
  //  }
//  std::unique_ptr<grid_map_msgs::msg::GridMap> message;
//  message = grid_map::GridMapRosConverter::toMessage(map);
//  message->header.stamp = rclcpp::Clock().now();
//
//  gridmap_publisher->publish(std::move(message));

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
        // 使うロボットの位置が障害物にならないようにする
        for (grid_map::CircleIterator iterator(map, robot->pose.pos, 0.31); !iterator.isPastEnd();
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

      if (not command.local_planner_config.disable_rule_area_avoidance) {
        map[map_name] += map["rule"];
      }

      if (command.robot_id == debug_id) {
        for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
          Point position;
          map.getPosition(*iterator, position);
          visualizer->addPoint(
            position.x(), position.y(), static_cast<int>(map.at(map_name, *iterator)), "red", 1.);
        }
      }

      auto route = findPathAStar(robot->pose.pos, target, map_name, command.robot_id);

      // Index -> Position変換
      std::vector<Point> path;
      std::transform(route.begin(), route.end(), std::back_inserter(path), [&](auto & index) {
        Point p;
        map.getPosition(index, p);
        return p;
      });

      if (path.size() < 2) {
        path.clear();
        path.push_back(robot->pose.pos);
        path.push_back(target);
      }

      // ゴール地点の量子化誤差を除去
      if ((path.back() - target).norm() < 0.05) {
        path.back() = target;
      }

      // 始点と終点以外の経由点を近い順に削除できるものは取り除く
      int max_safe_index = [&]() {
        for (int i = 1; i < static_cast<int>(path.size()); ++i) {
          if (not map.isInside(path[i])) {
            return i - 1;
          }
          auto diff = path[i] - path[0];
          for (int j = 0; j < i; ++j) {
            auto intermediate_point = path[0] + diff * ((j + 1.) / (i + 1.));
            grid_map::Index index;
            if (not map.getIndex(intermediate_point, index) or map.at(map_name, index) >= 1.0) {
              // i番目の経由点は障害物にぶつかるのでi-1番目まではOK
              return i - 1;
            }
          }
        }
        return static_cast<int>(path.size()) - 1;
      }();

      // max_safe_indexは残す経由点のインデックス
      if (max_safe_index > 1) {
        // [最初の経由点, max_safe_index)の経由点を消す
        path.erase(path.begin() + 1, path.begin() + max_safe_index);
      }

      std::vector<double> velocity(path.size(), 0.0);
      velocity[0] = robot->vel.linear.norm();
      velocity.back() = command.local_planner_config.terminal_velocity;

      for (int k = 1; k < static_cast<int>(path.size()); k++) {
        visualizer->addLine(path[k - 1], path[k], 1);
      }

      if (path.size() > 2) {
        // 最終速度を考慮した速度
        for (int i = static_cast<int>(path.size()) - 2; i > 0; i--) {
          double distance = (path[i + 1] - path[i]).norm();
          velocity[i] = std::min(
            std::sqrt(
              velocity[i + 1] * velocity[i + 1] +
              2 * command.local_planner_config.max_acceleration * distance),
            static_cast<double>(command.local_planner_config.max_velocity));
        }

        // 現在速度を考慮した速度
        for (int i = 1; i < static_cast<int>(path.size()); i++) {
          double distance = (path[i] - path[i - 1]).norm();
          velocity[i] = std::min(
            velocity[i], std::sqrt(
                           velocity[i - 1] * velocity[i - 1] +
                           2 * command.local_planner_config.max_acceleration * distance));
        }
        command.local_planner_config.terminal_velocity = velocity[1];
      } else {
        // 経由点なしの場合
        auto distance = (path[0] - path[1]).norm();
        command.local_planner_config.terminal_velocity = 0.;
        auto two_a_x = 2 * command.local_planner_config.max_acceleration * distance;
        velocity[1] = std::min(
          std::sqrt(velocity[0] * velocity[0] + two_a_x),
          static_cast<double>(command.local_planner_config.max_velocity));
        velocity[1] = std::min(
          velocity[1], std::sqrt(
                         command.local_planner_config.terminal_velocity *
                           command.local_planner_config.terminal_velocity +
                         two_a_x));
      }

      Velocity vel;
      vel << vx_controllers[command.robot_id].update(
        path[1].x() - command.current_pose.x, 1.f / 30.f),
        vy_controllers[command.robot_id].update(path[1].y() - command.current_pose.y, 1.f / 30.f);
      vel += vel.normalized() * command.local_planner_config.terminal_velocity;

      double max_velocity = [&]() {
        // プランナーやスキルで設定された最大速度
        double max_vel = command.local_planner_config.max_velocity;
        // LocalPlannerで設定された最大速度
        max_vel = std::min(max_vel, MAX_VEL);
        // STOPの場合はルールで設定された最大速度
        if (
          world_model->play_situation.getSituationCommandID() ==
          crane_msgs::msg::PlaySituation::STOP) {
          max_vel = std::min(max_vel, 1.5);
        }

        // ロボットに衝突しそうなときに速度を抑える
        {
          auto [nearest_robot, nearest_robot_distance] =
            world_model->getNearestRobotsWithDistanceFromPoint(
              robot->pose.pos, world_model->theirs.getAvailableRobots());

          if (nearest_robot) {
            Velocity relative_velocity = (robot->vel.linear - nearest_robot->vel.linear);
            // 2m以内のロボットに対してx,y ともに近づいていて、速度が1.0m以上の場合、速度を1.0にする
            if (
              nearest_robot_distance < 2.0 && relative_velocity.x() > 0.0 &&
              relative_velocity.y() > 0.0 && relative_velocity.norm() > 1.0) {
              max_vel = std::max(1.0, max_vel * 0.5);
            }
          }
        }

        // 3点以上の経由点がある場合、2点目と3点目の角度を考慮して速度を抑える
        if (path.size() > 2) {
          double path_angle =
            M_PI - std::abs(getAngleDiff(getAngle(path[2] - path[1]), getAngle(path[0] - path[1])));
          max_vel = std::min(
            max_vel,
            std::sqrt(
              2. * command.local_planner_config.max_acceleration * (path[1] - path[0]).norm()) +
              cos(path_angle) * command.local_planner_config.max_velocity);
        }
        return max_vel;
      }();

      // 低すぎると動かない
      if (max_velocity > 0.0001) {
        max_velocity = std::max(max_velocity, 0.1);
      }

      // 最大速度を超えないようにする
      if (vel.norm() > max_velocity) {
        vel = vel.normalized() * max_velocity;
      }

      command.target_velocity.x = vel.x();
      command.target_velocity.y = vel.y();

      command.target_x.clear();
      command.target_y.clear();
      command.target_x.push_back(path[1].x());
      command.target_y.push_back(path[1].y());

      {
        // 角度を小出しにしていく
        double target_theta = command.target_theta.empty() ? 0.0 : command.target_theta.front();
        command.target_theta.clear();
        double angle_diff = getAngleDiff(target_theta, command.current_pose.theta);
        double max_diff = 2.0 / (vel.norm() + 0.2);
        angle_diff = std::clamp(angle_diff, -max_diff, max_diff);
        command.target_theta.push_back(normalizeAngle(command.current_pose.theta + angle_diff));
      }
    }
  }
  visualizer->publish();
  return commands;
}
}  // namespace crane
