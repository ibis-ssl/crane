// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__BEHAVIOR_TREE__STATUS_CONVERTER__INVERT_HPP_
#define CRANE_BT_EXECUTOR__BEHAVIOR_TREE__STATUS_CONVERTER__INVERT_HPP_

#include <memory>
#include "crane_bt_executor/behavior_tree/status_converter/status_converter.hpp"

class Inverter : StatusConverter
{
public:
  explicit Inverter(std::shared_ptr<Component> base) : StatusConverter("Inverter", base)
  {
  }

  Status run(WorldModelWrapper::SharedPtr world_model, RobotIO robot) override
  {
    base->status = base->run(world_model, my_id);
    if (base->status == Status::SUCCESS)
    {
      this->status = Status::FAILURE;
    }
    else if (base->status == Status::FAILURE)
    {
      this->status = Status::SUCCESS;
    }
    else
    {
      this->status = Status::RUNNING;
    }
    return status;
  }
};
#endif  // CRANE_BT_EXECUTOR__BEHAVIOR_TREE__STATUS_CONVERTER__INVERT_HPP_
