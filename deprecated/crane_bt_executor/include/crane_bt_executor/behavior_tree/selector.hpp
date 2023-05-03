// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__BEHAVIOR_TREE__SELECTOR_HPP_
#define CRANE_BT_EXECUTOR__BEHAVIOR_TREE__SELECTOR_HPP_

#include <memory>

#include "crane_bt_executor/composite/composite.hpp"
#include "crane_bt_executor/robot_io.hpp"

/**
 * 失敗しないものが出るまで先頭から順番に実行し続ける
 * 失敗ではない場合，その判定が返される
 * 全て失敗すれば失敗判定
 */
class Selector : public Composite
{
public:
  Selector() { name_ = "Selector"; }

  Status run(WorldModelWrapper::SharedPtr world_model, RobotIO robot) override
  {
    for (auto & c : children_) {
      c->status_ = c->run(world_model, robot);
      if (c->status_ != Status::FAILURE) {
        return c->status_;
      }
    }
    return Status::FAILURE;
  }
};
#endif  // CRANE_BT_EXECUTOR__BEHAVIOR_TREE__SELECTOR_HPP_
