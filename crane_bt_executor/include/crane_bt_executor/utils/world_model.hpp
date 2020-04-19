// Copyright (c) 2019 ibis-ssl
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

#ifndef CRANE_BT_EXECUTOR__UTILS__WORLD_MODEL_HPP_
#define CRANE_BT_EXECUTOR__UTILS__WORLD_MODEL_HPP_

#include <crane_msgs/msg/world_model.hpp>
#include <crane_bt_executor/utils/boost_geometry.hpp>
#include <eigen3/Eigen/Core>
#include <vector>
#include <memory>

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
};

struct TeamInfo
{
  Rect defense_area;
  std::vector<RobotInfo> robots;
};

struct Ball
{
  Point pos;
  Point vel;
  bool is_curve;
};

struct WorldModel
{
  typedef std::shared_ptr<WorldModel> SharedPtr;
  WorldModel()
  {
    // メモリ確保
    // ヒトサッカーの台数は超えないはず
    constexpr uint8_t MAX_ROBOT_NUM = 11;
    ours.robots.reserve(MAX_ROBOT_NUM);
    theirs.robots.reserve(MAX_ROBOT_NUM);
  }
  void update(const crane_msgs::msg::WorldModel world_model)
  {
    ours.robots.clear();
    theirs.robots.clear();
    for (auto robot : world_model.robot_info_ours) {
      if (!robot.disappeared) {
        RobotInfo info;
        info.id = robot.robot_id;
        info.pose.pos << robot.pose.x, robot.pose.y;
        // todo : theta
        info.vel.linear << robot.velocity.x, robot.velocity.y;
        // todo : omega
        ours.robots.emplace_back(info);
      }
    }

    for (auto robot : world_model.robot_info_theirs) {
      if (!robot.disappeared) {
        RobotInfo info;
        info.id = robot.robot_id;
        info.pose.pos << robot.pose.x, robot.pose.y;
        // todo : theta
        info.vel.linear << robot.velocity.x, robot.velocity.y;
        // todo : omega
        theirs.robots.emplace_back(info);
      }
    }

    ours.defense_area.max << world_model.our_defense.max.x,
      world_model.our_defense.max.y;
    ours.defense_area.min << world_model.our_defense.min.x,
      world_model.our_defense.min.y;

    theirs.defense_area.max << world_model.our_defense.max.x,
      world_model.our_defense.max.y;
    theirs.defense_area.min << world_model.their_defense.min.x,
      world_model.their_defense.min.y;

    ball.pos << world_model.ball_info.pose.x,
      world_model.ball_info.pose.y;
    ball.vel << world_model.ball_info.velocity.x,
      world_model.ball_info.velocity.y;
//    ball.is_curve = world_model->ball_info.curved;

    field_size << world_model.field_info.x, world_model.field_info.y;
  }

  TeamInfo ours;
  TeamInfo theirs;
  Point field_size;
  Ball ball;
};

#endif  // CRANE_BT_EXECUTOR__UTILS__WORLD_MODEL_HPP_
