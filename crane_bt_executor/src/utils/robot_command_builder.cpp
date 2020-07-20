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

#include <memory>
#include <vector>
#include <algorithm>

#include "crane_bt_executor/utils/robot_command_builder.hpp"
#include "crane_world_observer/world_model.hpp"

void AvoidancePathGenerator::calcAvoidancePath(bool ball_avoidance = true)
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
crane_msgs::msg::RobotCommand RobotCommandBuilder::getCmd()
{
  cmd_.current_theta = info_->pose.theta;
  return cmd_;
}
RobotCommandBuilder & RobotCommandBuilder::setTargetPos(
  Point pos,
  bool ball_avoidance)
{
  auto generator = AvoidancePathGenerator(pos, world_model_, info_);
  generator.calcAvoidancePath(ball_avoidance);
  Point target_pos = generator.getTarget();

  float dist = bg::distance(info_->pose.pos, target_pos);
  if ((target_pos - pos).squaredNorm() > 0.5f * 0.5f) {
    // if target_pos is intermediate point, robot should not be stop
    dist += 0.5f;
  }

  Vector2 dir =
    tool::getDirectonNorm(info_->pose.pos, target_pos) * velocity_planner_.getVelocity();

  setVelocity(dir, dist);
  return *this;
}
RobotCommandBuilder & RobotCommandBuilder::setVelocity(
  Vector2 direction,
  float distance)
{
  direction.normalize();
  Eigen::Rotation2D<float> rot;
  rot.angle() = -info_->pose.theta;
  Velocity transformed_vel;
  velocity_planner_.update(distance);
  transformed_vel = (rot * direction) * velocity_planner_.getVelocity();
  cmd_.target.x = transformed_vel.x();
  cmd_.target.y = transformed_vel.y();

  return *this;
}
RobotCommandBuilder::RobotCommandBuilder(
  std::shared_ptr<WorldModel> world_model, std::shared_ptr<RobotInfo> info)
: world_model_(world_model)
{
  info_ = info;
  velocity_planner_.initilize(60.f, 6.0f, 2.0f);
  cmd_.robot_id = info->id;
}
