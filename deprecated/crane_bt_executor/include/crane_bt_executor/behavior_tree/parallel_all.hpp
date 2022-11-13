// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__BEHAVIOR_TREE__PARALLEL_ALL_HPP_
#define CRANE_BT_EXECUTOR__BEHAVIOR_TREE__PARALLEL_ALL_HPP_

#include "crane_bt_executor/composite/composite.hpp"
#include "crane_bt_executor/robot_io.hpp"

/**
 * 全てのタスクを並列実行（既に成功したものは実行しない）
 * 全て成功したら成功判定
 * 一つでも失敗すれば失敗判定
 */
class ParallelAll : public Composite
{
public:
  ParallelAll()
  {
    name_ = "ParallelAll";
  }

  Status run(const WorldModelWrapper::SharedPtr world_model, RobotIO robot) override
  {
    uint8_t num_success = 0;
    for (auto& c : children_)
    {
      if (c->status_ == Status::SUCCESS)
      {
        num_success++;
        continue;
      }
      c->status_ = c->run(world_model, robot);
      if (c->status_ == Status::FAILURE)
      {
        return Status::FAILURE;
      }
      else if (c->status_ == Status::SUCCESS)
      {
        num_success++;
      }
    }
    if (num_success == children_.size())
    {
      return Status::SUCCESS;
    }
    return Status::RUNNING;
  }
};
#endif  // CRANE_BT_EXECUTOR__BEHAVIOR_TREE__PARALLEL_ALL_HPP_
