// Copyright (c) 2020 ibis-ssl
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

#ifndef CRANE_BT_EXECUTOR__UTILS__ROBOT_COMMAND_BUILDER_HPP_
#define CRANE_BT_EXECUTOR__UTILS__ROBOT_COMMAND_BUILDER_HPP_

#include <memory>
#include <vector>
#include <algorithm>

#include "crane_bt_executor/utils/eigen_adapter.hpp"
#include "crane_bt_executor/utils/boost_geometry.hpp"
#include "crane_msgs/msg/robot_command.hpp"
#include "crane_bt_executor/utils/tool.hpp"
//  #include "utils/pid_controller.hpp"

//  pre declaration
class WorldModel;

class AvoidancePathGenerator
{
public:
  AvoidancePathGenerator(
    Point target_pos,
    const std::shared_ptr<WorldModel> world_model, std::shared_ptr<RobotInfo> info)
  : target(target_pos), world_model(world_model), info(info)
  {}
  void calcAvoidancePath(bool ball_avoidance = true)
  {
    const Point original_target = target;
    Segment seg;
    seg.first = info->pose.pos;
    seg.second = target;
    struct Obstacle
    {
      Point pos;
      float r;
    };
    std::vector<Obstacle> obs;
    // friend
    for (auto robot : world_model->ours.robots) {
      if (robot.id != info->id) {
        obs.push_back({robot.pose.pos, 0.2f});
      }
    }
    // enemy
    for (auto robot : world_model->theirs.robots) {
      obs.push_back({robot.pose.pos, 0.3f});
    }
    // ball
    if (ball_avoidance) {
      obs.push_back({world_model->ball.pos, 0.1f});
    }
    // detect collision
    std::vector<Obstacle> collided_obs;
    Vector2 segment = target - info->pose.pos;

    for (auto ob : obs) {
      // robotより後ろに居たら無視
      if (segment.dot(ob.pos - info->pose.pos) < 0) {
        continue;
      }
      // targetより向こうに居たら無視
      if (segment.dot(ob.pos - target) > 0) {
        continue;
      }
      if (bg::distance(ob.pos, seg) < ob.r) {
        collided_obs.push_back(ob);
      }
    }

    if (!collided_obs.empty()) {
      auto closest_obs = std::min_element(collided_obs.begin(), collided_obs.end(),
          [this](Obstacle a, Obstacle b) -> bool {
            return bg::comparable_distance(a.pos, info->pose.pos) <
            bg::comparable_distance(b.pos, info->pose.pos);
          });
      // generate avoiding point
      constexpr float OFFSET = 0.5f;
      Vector2 diff = (closest_obs->pos - info->pose.pos).normalized();
      Vector2 offset = tool::getVerticalVec(diff) * OFFSET;

      Point avoid_point_1 = closest_obs->pos + offset;
      Point avoid_point_2 = closest_obs->pos - offset;

      // 近い方の回避点を採用
      target = std::min(avoid_point_1, avoid_point_2, [this](auto a, auto b) -> bool {
            return bg::comparable_distance(target, a) < bg::comparable_distance(target, b);
          });
    }
    if (target != original_target) {
      calcAvoidancePath(ball_avoidance);
    }
  }

  Point getTarget()
  {
    return target;
  }

protected:
//  static constexpr float DECELARATION_THRESHOLD = 0.5f;

protected:
  Point target;
  const std::shared_ptr<WorldModel> world_model;
  std::shared_ptr<RobotInfo> info;
};

class RobotCommandBuilder
{
public:
  struct Output
  {
    Velocity velocity;
    float theta;
  };

  explicit RobotCommandBuilder(std::shared_ptr<RobotInfo> info)
  : info(info)
  {}

  crane_msgs::msg::RobotCommand getCmd() {return cmd;}

  void resetCmd()
  {
    cmd.chip_enable = false;
    cmd.kick_power = 0.f;
    cmd.dribble_power = 0.f;
  }
  RobotCommandBuilder & addDribble(float power)
  {
    cmd.dribble_power = power;
    return *this;
  }

  RobotCommandBuilder & addChipKick(float power)
  {
    cmd.chip_enable = true;
    cmd.kick_power = power;
    return *this;
  }

  RobotCommandBuilder & addStraightKick(float power)
  {
    cmd.chip_enable = false;
    cmd.kick_power = power;
    return *this;
  }

  RobotCommandBuilder & setTargetPos(Point pos)
  {
    auto generator = AvoidancePathGenerator(pos, world_model, info);
    generator.calcAvoidancePath();
    Point target_pos = generator.getTarget();

    float vel_size = 0;  // TODO(HansRobo) : calc velocity
    Velocity vel = tool::getDirectonNorm(info->pose.pos, target_pos) * vel_size;
    cmd.target.x = vel.x();
    cmd.target.y = vel.y();
    return *this;
  }

  RobotCommandBuilder & setTargetTheta(float theta)
  {
    cmd.target.theta = theta;
    return *this;
  }


  void update(const std::shared_ptr<WorldModel> world_model)
  {}

protected:
//  const uint8_t ROBOT_ID;
  crane_msgs::msg::RobotCommand cmd;
  std::shared_ptr<WorldModel> world_model;
  std::shared_ptr<RobotInfo> info;
};

#endif  // CRANE_BT_EXECUTOR__UTILS__ROBOT_COMMAND_BUILDER_HPP_
