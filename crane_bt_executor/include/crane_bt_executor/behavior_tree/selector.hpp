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

#ifndef CRANE_BT_EXECUTOR__BEHAVIOR_TREE__SELECTOR_HPP_
#define CRANE_BT_EXECUTOR__BEHAVIOR_TREE__SELECTOR_HPP_

#include "crane_bt_executor/composite/composite.hpp"
#include <crane_bt_executor/robot_io.hpp>

/**
 * 失敗しないものが出るまで先頭から順番に実行し続ける
 * 失敗ではない場合，その判定が返される
 * 全て失敗すれば失敗判定
 */
class Selector : public Composite
{
public:
  Selector()
  {
    name = "Selector";
  }

  Status run(WorldModel & world_model, RobotIO robot) override
  {
    for (auto & c : children) {
      c->status = c->run(world_model, robot);
      if (c->status != Status::FAILURE) {
        return c->status;
      }
    }
    return Status::FAILURE;
  }
};
#endif  // CRANE_BT_EXECUTOR__BEHAVIOR_TREE__SELECTOR_HPP_