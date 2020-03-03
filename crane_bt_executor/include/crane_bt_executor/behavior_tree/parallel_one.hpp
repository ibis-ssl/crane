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

#ifndef CRANE_BT_EXECUTOR__BEHAVIOR_TREE__PARALLEL_ONE_HPP_
#define CRANE_BT_EXECUTOR__BEHAVIOR_TREE__PARALLEL_ONE_HPP_

#include "crane_bt_executor/composite/composite.hpp"


/**
 * 全てのタスクを並列実行(失敗したものは実行しない)
 * 一つでも成功判定が出たら成功
 * 全て失敗したら失敗判定
 */
class ParallelOne : public Composite
{
public:
  ParallelOne()
  {
    name = "ParallelOne";
  }

  Status run(WorldModel & world_model, uint8_t my_id) override
  {
    uint8_t num_failure = 0;
    for (auto & c : children) {
      if (c->status == Status::FAILURE) {
        num_failure++;
        continue;
      }
      c->status = c->run(world_model, my_id);

      if (c->status == Status::SUCCESS) {
        return Status::SUCCESS;
      } else if (c->status == Status::FAILURE) {
        num_failure++;
      }
    }

    if (num_failure == children.size()) {
      return Status::FAILURE;
    }
    return Status::RUNNING;
  }
};
#endif  // CRANE_BT_EXECUTOR__BEHAVIOR_TREE__PARALLEL_ONE_HPP_
