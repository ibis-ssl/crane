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

#ifndef CRANE_BT_EXECUTOR__BEHAVIOR_TREE__MULTI_ROBOT_SEQUENCE_HPP_
#define CRANE_BT_EXECUTOR__BEHAVIOR_TREE__MULTI_ROBOT_SEQUENCE_HPP_

#include <memory>
#include <vector>
#include <iostream>

#include "crane_bt_executor/composite/composite.hpp"
#include "crane_bt_executor/behavior_tree/single_robot_sequence.hpp"
#include "crane_bt_executor/robot_io.hpp"

/**
 * 逐次実行．（先頭のものが成功するまで次にいかない）
 * 全て成功すれば成功判定
 * 途中で失敗が出れば失敗判定
 */
class MultiRobotBehavior : public Composite
{
public:
  MultiRobotBehavior()
  {
    name_ = "MultiRobotBehavior";
  }

  Status run(std::shared_ptr<WorldModelWrapper> world_model, RobotIO robot) override
  {
    return Status::FAILURE;
  }
  void update(std::shared_ptr<WorldModelWrapper> world_model)
  {
    for (auto& robot : robots_)
    {
      robot->update(world_model);
    }
  }
  void getCommands(std::vector<crane_msgs::msg::RobotCommand>& cmds)
  {
    for (auto& robot : robots_)
    {
      cmds.push_back(robot->getCommand());
    }
  }

  void registerRobot(std::shared_ptr<SingleRobotSequence> robot)
  {
    robots_.push_back(robot);
  }

protected:
  std::vector<std::shared_ptr<SingleRobotSequence>> robots_;
};
#endif  // CRANE_BT_EXECUTOR__BEHAVIOR_TREE__MULTI_ROBOT_SEQUENCE_HPP_
