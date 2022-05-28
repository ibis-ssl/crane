// Copyright (c) 2022 ibis-ssl
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
