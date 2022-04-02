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

#include <memory>
#include "crane_bt_executor/composite/composite.hpp"
#include "crane_bt_executor/robot_io.hpp"

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
    name_ = "SingleRobotSequence";
  }
  void assignID(uint8_t id)
  {
    robot_id_ = id;
  }

  void update(std::shared_ptr<WorldModelWrapper> world_model)
  {
    io_.extractRobotInfo(world_model, robot_id_);
    if (!initilized_) {
      io_.setupBuilder(world_model);
      initilized_ = true;
    }

    run(world_model, io_);
  }
  Status run(std::shared_ptr<WorldModelWrapper> world_model, RobotIO robot) override
  {
    for (auto & c : children_) {
      if (c->status_ == Status::SUCCESS) {
        continue;
      }

      c->status_ = c->run(world_model, robot);

      if (c->status_ != Status::SUCCESS) {
        if (c->status_ == Status::FAILURE) {
          return Status::FAILURE;
        }
        return c->status_;
      }
    }
    return Status::SUCCESS;
  }

  crane_msgs::msg::RobotCommand getCommand()
  {
    return io_.builder->getCmd();
  }

  int robot_id_ = -1;
  RobotIO io_;
  bool initilized_ = false;
};
#endif  // CRANE_BT_EXECUTOR__BEHAVIOR_TREE__SINGLE_ROBOT_SEQUENCE_HPP_
