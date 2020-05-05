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

#include "eigen3/Eigen/Geometry"
#include "crane_bt_executor/utils/eigen_adapter.hpp"
#include "crane_bt_executor/utils/boost_geometry.hpp"
#include "crane_msgs/msg/robot_command.hpp"
#include "crane_bt_executor/utils/tool.hpp"
#include "crane_bt_executor/utils/velocity_planner.hpp"
//  #include "utils/pid_controller.hpp"

//  pre declaration
class WorldModel;

class AvoidancePathGenerator
{
public:
  AvoidancePathGenerator(
    Point target_pos,
    const std::shared_ptr<WorldModel> world_model, std::shared_ptr<RobotInfo> info)
  : target_(target_pos), world_model_(world_model), info_(info)
  {}
  void calcAvoidancePath(bool ball_avoidance = true)
  {
    const Point original_target = target_;
    Segment seg;
    seg.first = info_->pose.pos;
    seg.second = target_;
    struct Obstacle
    {
      Point pos;
      float r;
    };
    std::vector<Obstacle> obs;
    // friend
    for (auto robot : world_model_->ours.robots) {
      if (!robot->available) {
        continue;
      }
      if (robot->id != info_->id) {
        obs.push_back({robot->pose.pos, 0.5f});
      }
    }
    // enemy
    for (auto robot : world_model_->theirs.robots) {
      if (!robot->available) {
        continue;
      }
      obs.push_back({robot->pose.pos, 0.5f});
    }
    // ball
    if (ball_avoidance) {
      obs.push_back({world_model_->ball.pos, 0.1f});
    }
    // detect collision
    std::vector<Obstacle> collided_obs;
    Vector2 segment = target_ - info_->pose.pos;

    for (auto ob : obs) {
      // robotより後ろに居たら無視
      if (segment.dot(ob.pos - info_->pose.pos) < 0) {
        continue;
      }
      // targetより向こうに居たら無視
      if (segment.dot(ob.pos - target_) > 0) {
        continue;
      }
      if (bg::distance(ob.pos, seg) < ob.r) {
        collided_obs.push_back(ob);
      }
    }

    if (!collided_obs.empty()) {
      auto closest_obs = std::min_element(collided_obs.begin(), collided_obs.end(),
          [this](Obstacle a, Obstacle b) -> bool {
            return bg::comparable_distance(a.pos, info_->pose.pos) <
            bg::comparable_distance(b.pos, info_->pose.pos);
          });
      // generate avoiding point
      constexpr float OFFSET = 0.5f;
      Vector2 diff = (closest_obs->pos - info_->pose.pos).normalized();
      Vector2 offset = tool::getVerticalVec(diff) * OFFSET;

      Point avoid_point_1 = closest_obs->pos + offset;
      Point avoid_point_2 = closest_obs->pos - offset;

      // 近い方の回避点を採用
      target_ = std::min(avoid_point_1, avoid_point_2, [this](auto a, auto b) -> bool {
            return bg::comparable_distance(target_, a) < bg::comparable_distance(target_, b);
          });
    }
    if (target_ != original_target) {
      calcAvoidancePath(ball_avoidance);
    }
  }

  Point getTarget()
  {
    return target_;
  }

protected:
//  static constexpr float DECELARATION_THRESHOLD = 0.5f;

protected:
  Point target_;
  const std::shared_ptr<WorldModel> world_model_;
  std::shared_ptr<RobotInfo> info_;
};

class RobotCommandBuilder
{
public:
  struct Output
  {
    Velocity velocity;
    float theta;
  };

  explicit RobotCommandBuilder(
    std::shared_ptr<WorldModel> world_model,
    std::shared_ptr<RobotInfo> info)
  : world_model_(world_model)
  {
    info_ = info;
    velocity_planner_.initilize(60.f, 2.0f, 2.0f);
    cmd_.robot_id = info->id;
  }

  crane_msgs::msg::RobotCommand getCmd() {return cmd_;}

  void resetCmd()
  {
    cmd_.chip_enable = false;
    cmd_.kick_power = 0.f;
    cmd_.dribble_power = 0.f;
  }
  RobotCommandBuilder & addDribble(float power)
  {
    cmd_.dribble_power = power;
    return *this;
  }

  RobotCommandBuilder & addChipKick(float power)
  {
    cmd_.chip_enable = true;
    cmd_.kick_power = power;
    return *this;
  }

  RobotCommandBuilder & addStraightKick(float power)
  {
    cmd_.chip_enable = false;
    cmd_.kick_power = power;
    return *this;
  }

  RobotCommandBuilder & setTargetPos(Point pos)
  {
    auto generator = AvoidancePathGenerator(pos, world_model_, info_);
    generator.calcAvoidancePath();
    Point target_pos = generator.getTarget();
    if ((target_pos - pos).squaredNorm() < 0.1f) {
      velocity_planner_.update(info_->pose.pos, target_pos, 0.f);
    } else {
      velocity_planner_.update(info_->pose.pos, target_pos, 1.f);
    }

    Velocity vel =
      tool::getDirectonNorm(info_->pose.pos, target_pos) * velocity_planner_.getVelocity();

    setVelocity(vel);
    return *this;
  }

  RobotCommandBuilder & setTargetTheta(float theta)
  {
    cmd_.target.theta = theta;
    return *this;
  }

  RobotCommandBuilder & setVelocity(float x, float y)
  {
    Eigen::Rotation2D<float> rot;
    rot.angle() = -info_->pose.theta;
    Velocity vel, transformed_vel;
    vel << x, y;
    transformed_vel = rot * vel;
    cmd_.target.x = transformed_vel.x();
    cmd_.target.y = transformed_vel.y();

    return *this;
  }

  RobotCommandBuilder & setVelocity(Velocity vel)
  {
    return setVelocity(vel.x(), vel.y());
  }

protected:
//  const uint8_t ROBOT_ID;
  crane_msgs::msg::RobotCommand cmd_;
  std::shared_ptr<WorldModel> world_model_;
  std::shared_ptr<RobotInfo> info_;
  VelocityPlanner velocity_planner_;
};

#endif  // CRANE_BT_EXECUTOR__UTILS__ROBOT_COMMAND_BUILDER_HPP_
