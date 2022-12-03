// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__UTILS__ROBOT_COMMAND_BUILDER_HPP_
#define CRANE_BT_EXECUTOR__UTILS__ROBOT_COMMAND_BUILDER_HPP_

#include <memory>
#include <vector>
#include <algorithm>

#include "eigen3/Eigen/Geometry"
#include "crane_geometry/eigen_adapter.hpp"
#include "crane_geometry/boost_geometry.hpp"
#include "crane_msgs/msg/robot_command.hpp"
#include "crane_bt_executor/utils/tool.hpp"
#include "crane_bt_executor/utils/velocity_planner.hpp"

//  pre declaration
class WorldModelWrapper;
class RobotInfo;

class AvoidancePathGenerator
{
public:
  AvoidancePathGenerator(Point target_pos, const WorldModelWrapper::SharedPtr world_model,
                         std::shared_ptr<RobotInfo> info)
    : target_(target_pos), world_model_(world_model), info_(info)
  {
  }
  void calcAvoidancePath(bool ball_avoidance);

  Point getTarget()
  {
    return target_;
  }

protected:
  //  static constexpr float DECELARATION_THRESHOLD = 0.5f;

protected:
  Point target_;
  const WorldModelWrapper::SharedPtr world_model_;
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

  explicit RobotCommandBuilder(WorldModelWrapper::SharedPtr world_model, std::shared_ptr<RobotInfo> info);

  crane_msgs::msg::RobotCommand getCmd();

  void resetCmd()
  {
    cmd_.chip_enable = false;
    cmd_.kick_power = 0.f;
    cmd_.dribble_power = 0.f;
  }
  RobotCommandBuilder& addDribble(float power)
  {
    cmd_.dribble_power = power;
    return *this;
  }

  RobotCommandBuilder& addChipKick(float power)
  {
    cmd_.chip_enable = true;
    cmd_.kick_power = power;
    return *this;
  }

  RobotCommandBuilder& addStraightKick(float power)
  {
    cmd_.chip_enable = false;
    cmd_.kick_power = power;
    return *this;
  }

  RobotCommandBuilder& setTargetPos(Point pos, bool ball_avoidance = true);

  RobotCommandBuilder& setTargetTheta(float theta)
  {
    cmd_.target.theta = theta;
    return *this;
  }

  RobotCommandBuilder& setVelocity(Vector2 direction, float distance);

  RobotCommandBuilder& stop()
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
  WorldModelWrapper::SharedPtr world_model_;
  std::shared_ptr<RobotInfo> info_;
  VelocityPlanner velocity_planner_;
};

#endif  // CRANE_BT_EXECUTOR__UTILS__ROBOT_COMMAND_BUILDER_HPP_
