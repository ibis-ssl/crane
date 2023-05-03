// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__BEHAVIOR_TREE__STATUS_CONVERTER__ALWAYS_HPP_
#define CRANE_BT_EXECUTOR__BEHAVIOR_TREE__STATUS_CONVERTER__ALWAYS_HPP_

#include <memory>

#include "crane_bt_executor/behavior_tree/status_converter/status_converter.hpp"

/**
 * 判定は常にRUNNING
 */
class AlwaysRunning : public StatusConverter
{
public:
  explicit AlwaysRunning(std::shared_ptr<Component> base) : StatusConverter("AlwaysRunning", base)
  {
  }

  Status run(WorldModelWrapper::SharedPtr world_model, RobotIO robot) override
  {
    base_->status_ = base_->run(world_model, robot);
    return Status::RUNNING;
  }

  static auto build(std::shared_ptr<Component> base)
  {
    return std::make_shared<AlwaysRunning>(base);
  }
};

/**
 * 判定は常にFAILURE
 */
class AlwaysFailure : public StatusConverter
{
public:
  explicit AlwaysFailure(std::shared_ptr<Component> base) : StatusConverter("AlwaysFailure", base)
  {
  }

  Status run(WorldModelWrapper::SharedPtr world_model, RobotIO robot) override
  {
    base_->status_ = base_->run(world_model, robot);
    return Status::FAILURE;
  }

  static auto build(std::shared_ptr<Component> base)
  {
    return std::make_shared<AlwaysFailure>(base);
  }
};

/**
 * 判定は常にSUCCESS
 */
class AlwaysSuccess : public StatusConverter
{
public:
  explicit AlwaysSuccess(std::shared_ptr<Component> base) : StatusConverter("AlwaysSuccess", base)
  {
  }

  Status run(WorldModelWrapper::SharedPtr world_model, RobotIO robot) override
  {
    base_->status_ = base_->run(world_model, robot);
    return Status::SUCCESS;
  }

  static auto build(std::shared_ptr<Component> base)
  {
    return std::make_shared<AlwaysSuccess>(base);
  }
};
#endif  // CRANE_BT_EXECUTOR__BEHAVIOR_TREE__STATUS_CONVERTER__ALWAYS_HPP_
