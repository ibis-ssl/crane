// Copyright (c) 2022 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_

#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "crane_geometry/boost_geometry.hpp"
#include "crane_msgs/msg/world_model.hpp"
#include "eigen3/Eigen/Core"

struct Pose2D
{
  Point pos;
  float theta;
};

struct Velocity2D
{
  Point linear;
  float omega;
};

struct Rect
{
  Point min;
  Point max;
};

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
  WorldModelWrapper(rclcpp::Node & node)
  {
    // メモリ確保
    // ヒトサッカーの台数は超えないはず
    constexpr uint8_t MAX_ROBOT_NUM = 20;
    for (int i = 0; i < MAX_ROBOT_NUM; i++) {
      ours.robots.emplace_back(std::make_shared<RobotInfo>());
      theirs.robots.emplace_back(std::make_shared<RobotInfo>());
    }

    subscriber_ = node.create_subscription<crane_msgs::msg::WorldModel>(
      "/world_model", 10,
      [this](const crane_msgs::msg::WorldModel::SharedPtr msg) -> void {
        latest_msg_ = *msg;
        this->update(*msg);
        has_updated_ = true;
        for (auto & callback : callbacks_) {
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

  const crane_msgs::msg::WorldModel & getMsg() const { return latest_msg_; }
  bool hasUpdated() const { return has_updated_; }

  void addCallback(std::function<void(void)> && callback_func)
  {
    callbacks_.emplace_back(callback_func);
  }

  auto getRobot(RobotIdentifier id)
  {
    if (id.is_ours) {
      return ours.robots.at(id.robot_id);
    } else {
      return theirs.robots.at(id.robot_id);
    }
  }

  auto generateFieldPoints(float grid_size)
  {
    std::vector<Point> points;
    for (float x = 0.f; x <= field_size.x() / 2.f; x += grid_size) {
      for (float y = 0.f; y <= field_size.y() / 2.f; y += grid_size) {
        points.emplace_back(Point(x, y));
      }
    }
    return points;
  }

  bool isEnemyGoalArea(Point p) { return isInRect(ours.defense_area, p); }
  bool isFriendGoalArea(Point p) { return isInRect(theirs.defense_area, p); }

  bool isGoalArea(Point p) { return isFriendGoalArea(p) || isEnemyGoalArea(p); }

  bool isInRect(Rect rect, Point p)
  {
    if (
      p.x() >= rect.min.x() && p.x() <= rect.max.x() && p.y() >= rect.min.y() &&
      p.y() <= rect.max.y()) {
      return true;
    }
    return false;
  }

  TeamInfo ours;
  TeamInfo theirs;
  Point field_size, defense_area, goal;
  Ball ball;
  // std_msgs::Time
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr subscriber_;
  std::vector<std::function<void(void)>> callbacks_;
  crane_msgs::msg::WorldModel latest_msg_;
  bool has_updated_ = false;
};

#endif  // CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_
