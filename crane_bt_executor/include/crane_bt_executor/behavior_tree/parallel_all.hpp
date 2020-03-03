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

#ifndef CRANE_BT_EXECUTOR__BEHAVIOR_TREE__PARALLEL_ALL_HPP_
#define CRANE_BT_EXECUTOR__BEHAVIOR_TREE__PARALLEL_ALL_HPP_

#include "crane_bt_executor/composite/composite.hpp"


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
    name = "ParallelAll";
  }

  Status run(WorldModel & world_model, uint8_t my_id) override
  {
    uint8_t num_success = 0;
    for (auto & c : children) {
      if (c->status == Status::SUCCESS) {
        num_success++;
        continue;
      }
      c->status = c->run(world_model, my_id);
      if (c->status == Status::FAILURE) {
        return Status::FAILURE;
      } else if (c->status == Status::SUCCESS) {
        num_success++;
      }
    }
    if (num_success == children.size()) {
      return Status::SUCCESS;
    }
    return Status::RUNNING;
  }
};
#endif  // CRANE_BT_EXECUTOR__BEHAVIOR_TREE__PARALLEL_ALL_HPP_
