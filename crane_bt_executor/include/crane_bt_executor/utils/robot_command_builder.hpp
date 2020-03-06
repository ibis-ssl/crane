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

#include "crane_bt_executor/utils/eigen_adapter.hpp"
#include <crane_msgs/msg/robot_command.hpp>
#include <memory>
//  #include "utils/pid_controller.hpp"

//  pre declaration
class WorldModel;



class AvoidancePathGenerator
{
public:
  AvoidancePathGenerator(
    Eigen::Vector2f target_pos,
    const std::shared_ptr<WorldModel> world_model)
  : target(target_pos), world_model(world_model)
  {}
  void calcAvoidancePath()
  {}
  void calcVelocity() {}
  Eigen::Vector2f getNextPoint()
  {}
  Eigen::Vector2f getVelocity() {}

protected:
  static constexpr float DECELARATION_THRESHOLD = 0.5f;

protected:
  Eigen::Vector2f target;
  const std::shared_ptr<WorldModel> world_model;
};

class RobotCommandBuilder
{
public:
  struct Output
  {
    Eigen::Vector2f velocity;
    float theta;
  };

  explicit RobotCommandBuilder(const uint8_t robot_id)
  : ROBOT_ID(robot_id)
  {}

  crane_msgs::msg::RobotCommand getCmd() {return cmd;}

  void resetCmd()
  {
    cmd.chip_kick_enable = false;
    cmd.straight_kick_enable = false;
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
    cmd.chip_kick_enable = true;
    cmd.straight_kick_enable = false;
    cmd.kick_power = power;

    return *this;
  }

  RobotCommandBuilder & addStraightKick(float power)
  {
    cmd.chip_kick_enable = false;
    cmd.straight_kick_enable = true;
    cmd.kick_power = power;

    return *this;
  }

  RobotCommandBuilder & setTargetPose(Eigen::Vector2f pos, float theta)
  {
    auto generator = AvoidancePathGenerator(pos, world_model);
    generator.calcAvoidancePath();
    generator.calcVelocity();
    Eigen::Vector2f vel = generator.getVelocity();

    cmd.target.x = vel.x();
    cmd.target.y = vel.y();
    cmd.target.theta = theta;

    return *this;
  }

  void update(const std::shared_ptr<WorldModel> world_model)
  {}

protected:
  const uint8_t ROBOT_ID;
  crane_msgs::msg::RobotCommand cmd;
  std::shared_ptr<WorldModel> world_model;
};


#endif  // CRANE_BT_EXECUTOR__UTILS__ROBOT_COMMAND_BUILDER_HPP_
