// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__BEHAVIOR_TREE__MULTI_ROBOT_SEQUENCE_HPP_
#define CRANE_BT_EXECUTOR__BEHAVIOR_TREE__MULTI_ROBOT_SEQUENCE_HPP_

#include <iostream>
#include <memory>
#include <vector>

#include "crane_bt_executor/behavior_tree/single_robot_sequence.hpp"
#include "crane_bt_executor/composite/composite.hpp"
#include "crane_bt_executor/robot_io.hpp"

/**
 * 逐次実行．（先頭のものが成功するまで次にいかない）
 * 全て成功すれば成功判定
 * 途中で失敗が出れば失敗判定
 */
class MultiRobotBehavior : public Composite
{
public:
  MultiRobotBehavior() { name_ = "MultiRobotBehavior"; }

  Status run(WorldModelWrapper::SharedPtr world_model, RobotIO robot) override
  {
    return Status::FAILURE;
  }
  void update(WorldModelWrapper::SharedPtr world_model)
  {
    for (auto & robot : robots_) {
      robot->update(world_model);
    }
  }
  void getCommands(std::vector<crane_msgs::msg::RobotCommand> & cmds)
  {
    for (auto & robot : robots_) {
      cmds.push_back(robot->getCommand());
    }
  }

  void registerRobot(std::shared_ptr<SingleRobotSequence> robot) { robots_.push_back(robot); }

protected:
  std::vector<std::shared_ptr<SingleRobotSequence>> robots_;
};
#endif  // CRANE_BT_EXECUTOR__BEHAVIOR_TREE__MULTI_ROBOT_SEQUENCE_HPP_
