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

#ifndef CRANE_BT_EXECUTOR__BEHAVIOR_TREE__SINGLE_ROBOT_SEQUENCE_HPP_
#define CRANE_BT_EXECUTOR__BEHAVIOR_TREE__SINGLE_ROBOT_SEQUENCE_HPP_

#include "crane_bt_executor/composite/composite.hpp"
//#include "crane_bt_executor/behavior_tree/behavior_base.hpp"
#include <crane_bt_executor/robot_io.hpp>

/**
 * 逐次実行．（先頭のものが成功するまで次にいかない）
 * 全て成功すれば成功判定
 * 途中で失敗が出れば失敗判定
 */
class SingleRobotSequence : public Composite
{
public:
  SingleRobotSequence()
  {
    name = "SingleRobotSequence";
  }
  void assignID(uint8_t id)
  {
    robot_id = id;
  }

  void update(WorldModel & world_model)
  {
    RobotIO io;
    io.resetBuilder(robot_id);
    io.extractRobotInfo(world_model, robot_id);

    run(world_model, io);
  }
  Status run(WorldModel & world_model, RobotIO robot) override
  {
    for (auto & c : children) {
      if (c->status == Status::SUCCESS) {
        continue;
      }

      c->status = c->run(world_model, robot);

      if (c->status != Status::SUCCESS) {
        if (c->status == Status::FAILURE) {
          return Status::FAILURE;
        }
        return c->status;
      }
    }
    return Status::SUCCESS;
  }
  int robot_id = -1;
};
#endif  // CRANE_BT_EXECUTOR__BEHAVIOR_TREE__SINGLE_ROBOT_SEQUENCE_HPP_