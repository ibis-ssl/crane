// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "crane_geometry/boost_geometry.hpp"
#include "crane_geometry/geometry_operations.hpp"
#include "crane_msgs/msg/world_model.hpp"

namespace crane
{
struct RobotInfo
{
  uint8_t id;

  Pose2D pose;

  Velocity2D vel;

  bool available = false;

  using SharedPtr = std::shared_ptr<RobotInfo>;
};

struct TeamInfo
{
  Rect defense_area;

  std::vector<std::shared_ptr<RobotInfo>> robots;
};

struct Ball
{
  Point pos;

  Point vel;

  bool is_curve;
};

struct RobotIdentifier
{
  bool is_ours;

  uint8_t robot_id;
};

struct WorldModelWrapper
{
  typedef std::shared_ptr<WorldModelWrapper> SharedPtr;

  typedef std::unique_ptr<WorldModelWrapper> UniquePtr;

  WorldModelWrapper(rclcpp::Node & node)
  {
    // メモリ確保
    // ヒトサッカーの台数は超えないはず
    constexpr uint8_t MAX_ROBOT_NUM = 20;
    for (int i = 0; i < MAX_ROBOT_NUM; i++) {
      ours.robots.emplace_back(std::make_shared<RobotInfo>());
      theirs.robots.emplace_back(std::make_shared<RobotInfo>());
    }

    subscriber = node.create_subscription<crane_msgs::msg::WorldModel>(
      "/world_model", 10, [this](const crane_msgs::msg::WorldModel::SharedPtr msg) -> void {
        latest_msg = *msg;
        this->update(*msg);
        has_updated = true;
        for (auto & callback : callbacks) {
          callback();
        }
      });
  }

  void update(const crane_msgs::msg::WorldModel & world_model)
  {
    for (auto & robot : world_model.robot_info_ours) {
      auto & info = ours.robots.at(robot.id);
      info->available = !robot.disappeared;
      if (info->available) {
        info->id = robot.id;
        info->pose.pos << robot.pose.x, robot.pose.y;
        info->pose.theta = robot.pose.theta;
        info->vel.linear << robot.velocity.x, robot.velocity.y;
        // todo : omega
      }
    }

    for (auto robot : world_model.robot_info_theirs) {
      auto & info = theirs.robots.at(robot.id);
      info->available = !robot.disappeared;
      if (info->available) {
        info->id = robot.id;
        info->pose.pos << robot.pose.x, robot.pose.y;
        info->pose.theta = robot.pose.theta;
        info->vel.linear << robot.velocity.x, robot.velocity.y;
        // todo : omega
      }
    }
    //
    //    ours.defense_area.max << world_model.our_defense.max.x, world_model.our_defense.max.y;
    //    ours.defense_area.min << world_model.our_defense.min.x, world_model.our_defense.min.y;
    //
    //    theirs.defense_area.max << world_model.our_defense.max.x, world_model.our_defense.max.y;
    //    theirs.defense_area.min << world_model.their_defense.min.x, world_model.their_defense.min.y;

    ball.pos << world_model.ball_info.pose.x, world_model.ball_info.pose.y;
    ball.vel << world_model.ball_info.velocity.x, world_model.ball_info.velocity.y;
    //    ball.is_curve = world_model.ball_info.curved;

    field_size << world_model.field_info.x, world_model.field_info.y;
    defense_area << world_model.defense_area.x, world_model.defense_area.y;
    goal << world_model.goal.x, world_model.goal.y;
  }

  const crane_msgs::msg::WorldModel & getMsg() const { return latest_msg; }

  bool hasUpdated() const { return has_updated; }

  void addCallback(std::function<void(void)> && callback_func)
  {
    callbacks.emplace_back(callback_func);
  }

  auto getRobot(RobotIdentifier id)
  {
    if (id.is_ours) {
      return ours.robots.at(id.robot_id);
    } else {
      return theirs.robots.at(id.robot_id);
    }
  }

  auto getDistanceFromRobotToBall(RobotIdentifier id) -> double
  {
    return getDistanceFromRobot(id, ball.pos);
  }

  auto getSquareDistanceFromRobotToBall(RobotIdentifier id) -> double
  {
    return getSquareDistanceFromRobot(id, ball.pos);
  }

  auto generateFieldPoints(float grid_size) const
  {
    std::vector<Point> points;
    for (float x = 0.f; x <= field_size.x() / 2.f; x += grid_size) {
      for (float y = 0.f; y <= field_size.y() / 2.f; y += grid_size) {
        points.emplace_back(Point(x, y));
      }
    }
    return points;
  }

  auto getDistanceFromRobot(RobotIdentifier id, Point point) -> double
  {
    return (getRobot(id)->pose.pos - point).norm();
  }

  auto getSquareDistanceFromRobot(RobotIdentifier id, Point point) -> double
  {
    return (getRobot(id)->pose.pos - point).squaredNorm();
  }

  bool isEnemyDefenseArea(const Point & p) const { return isInRect(ours.defense_area, p); }

  bool isFriendDefenseArea(const Point & p) const { return isInRect(theirs.defense_area, p); }

  bool isDefenseArea(Point p) const { return isFriendDefenseArea(p) || isEnemyDefenseArea(p); }

  double getDefenseWidth() const { return ours.defense_area.max.y() - ours.defense_area.min.y(); }

  double getDefenseHeight() const { return ours.defense_area.max.x() - ours.defense_area.min.x(); }

  TeamInfo ours;

  TeamInfo theirs;

  Point field_size, defense_area, goal;

  Ball ball;

private:
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr subscriber;

  std::vector<std::function<void(void)>> callbacks;

  crane_msgs::msg::WorldModel latest_msg;

  bool has_updated = false;
};
}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_
