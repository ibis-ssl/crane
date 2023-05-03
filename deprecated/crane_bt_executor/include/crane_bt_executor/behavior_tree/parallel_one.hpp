// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__BEHAVIOR_TREE__PARALLEL_ONE_HPP_
#define CRANE_BT_EXECUTOR__BEHAVIOR_TREE__PARALLEL_ONE_HPP_

#include <memory>

#include "crane_bt_executor/composite/composite.hpp"
#include "crane_bt_executor/robot_io.hpp"

/**
 * 全てのタスクを並列実行(失敗したものは実行しない)
 * 一つでも成功判定が出たら成功
 * 全て失敗したら失敗判定
 */
class ParallelOne : public Composite
{
public:
  ParallelOne() { name_ = "ParallelOne"; }

  Status run(WorldModelWrapper::SharedPtr world_model, RobotIO robot) override
  {
    uint8_t num_failure = 0;
    for (auto & c : children_) {
      if (c->status_ == Status::FAILURE) {
        num_failure++;
        continue;
      }
      c->status_ = c->run(world_model, robot);

      if (c->status_ == Status::SUCCESS) {
        return Status::SUCCESS;
      } else if (c->status_ == Status::FAILURE) {
        num_failure++;
      }
    }

    if (num_failure == children_.size()) {
      return Status::FAILURE;
    }
    return Status::RUNNING;
  }
};
#endif  // CRANE_BT_EXECUTOR__BEHAVIOR_TREE__PARALLEL_ONE_HPP_
