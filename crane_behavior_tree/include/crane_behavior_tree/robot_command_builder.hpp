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

#ifndef CRANE_BEHAVIOR_TREE__ROBOT_COMMAND_BUILDER_HPP_
#define CRANE_BEHAVIOR_TREE__ROBOT_COMMAND_BUILDER_HPP_

#include <eigen3/Eigen/Core>
#include <crane_msgs/msg/robot_command.hpp>
#include "utils/pid_controller.hpp"


enum class InputType
{
  TRANSFER,
  MOTION,
};

struct TransferInput
{
  Eigen::Vector2f pos;
  float theta;
};

struct MotionInput
{
  Eigen::Vector2f velocity;
  float theta;
};


class RobotCommandBuilder
{
public:
  struct Output
  {
    Eigen::Vector2f velocity;
    float theta;
  };

  explicit RobotCommandBuilder(const uint8_t robot_id) : ROBOT_ID(robot_id)
  {

  }

  virtual crane_msgs::msg::RobotCommand getCmd(){ return cmd;}

  void resetCmd(){
    cmd.chip_kick_enable = false;
    cmd.straight_kick_enable = false;
    cmd.kick_power = 0.f;
    cmd.dribble_power = 0.f;
  }
  RobotCommandBuilder & addDribble(float power)
  {
    cmd.dribble_power = power;
  }

  RobotCommandBuilder & addChipKick(float power)
  {
    cmd.chip_kick_enable = true;
    cmd.straight_kick_enable = false;
    cmd.kick_power = power;
  }

  RobotCommandBuilder & addStraightKick(float power)
  {
    cmd.chip_kick_enable = false;
    cmd.straight_kick_enable = true;
    cmd.kick_power = power;
  }

protected:
  const uint8_t ROBOT_ID;
  crane_msgs::msg::RobotCommand cmd;

};

class TransferRobotCommandBuilder : public RobotCommandBuilder
{
public:
  explicit TransferRobotCommandBuilder(const uint8_t robot_id)
  : RobotCommandBuilder(robot_id)
  {

  }

  void calcObstacleAvoidance()
  {

  }

  void calcVelocity()
  {

  }
  void publishTarget()
  {

  }

  void update()
  {
    calcObstacleAvoidance();
    calcVelocity();
    publishTarget();
  }

};
class MotionRobotCommandBuilder : public RobotCommandBuilder
{
public:
  explicit MotionRobotCommandBuilder(const uint8_t robot_id)
  : RobotCommandBuilder(robot_id)
  {

  }

  void publishTarget()
  {

  }

  void update()
  {
    publishTarget();
  }
};
#endif  // CRANE_BEHAVIOR_TREE__ROBOT_COMMAND_BUILDER_HPP_
