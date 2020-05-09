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

//  pre declaration
class WorldModel;
class RobotInfo;

class AvoidancePathGenerator
{
public:
  AvoidancePathGenerator(
    Point target_pos,
    const std::shared_ptr<WorldModel> world_model, std::shared_ptr<RobotInfo> info)
  : target_(target_pos), world_model_(world_model), info_(info)
  {}
  void calcAvoidancePath(bool ball_avoidance);

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
    std::shared_ptr<RobotInfo> info);

  crane_msgs::msg::RobotCommand getCmd();

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

  RobotCommandBuilder & setTargetPos(Point pos, bool ball_avoidance = true);

  RobotCommandBuilder & setTargetTheta(float theta)
  {
    cmd_.target.theta = theta;
    return *this;
  }

  RobotCommandBuilder & setVelocity(Vector2 direction, float distance);

  RobotCommandBuilder & stop()
  {
    cmd_.target.x = 0.0f;
    cmd_.target.y = 0.0f;
    cmd_.target.theta = 0.0f;
    return *this;
  }

//  RobotCommandBuilder & setVelocity(Velocity vel)
//  {
//    return setVelocity(vel.x(), vel.y());
//  }

protected:
//  const uint8_t ROBOT_ID;
  crane_msgs::msg::RobotCommand cmd_;
  std::shared_ptr<WorldModel> world_model_;
  std::shared_ptr<RobotInfo> info_;
  VelocityPlanner velocity_planner_;
};

#endif  // CRANE_BT_EXECUTOR__UTILS__ROBOT_COMMAND_BUILDER_HPP_
