// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

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
  SingleRobotSequence() { name_ = "SingleRobotSequence"; }
  void assignID(uint8_t id) { robot_id_ = id; }

  void update(WorldModelWrapper::SharedPtr world_model)
  {
    io_.extractRobotInfo(world_model, robot_id_);
    if (!initilized_) {
      io_.setupBuilder(world_model);
      initilized_ = true;
    }

    run(world_model, io_);
  }
  Status run(WorldModelWrapper::SharedPtr world_model, RobotIO robot) override
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

  crane_msgs::msg::RobotCommand getCommand() { return io_.builder->getCmd(); }

  int robot_id_ = -1;
  RobotIO io_;
  bool initilized_ = false;
};
#endif  // CRANE_BT_EXECUTOR__BEHAVIOR_TREE__SINGLE_ROBOT_SEQUENCE_HPP_
