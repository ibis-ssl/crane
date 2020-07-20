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

#ifndef CRANE_BT_EXECUTOR__ROBOT_IO_HPP_
#define CRANE_BT_EXECUTOR__ROBOT_IO_HPP_

#include "crane_bt_executor/utils/robot_command_builder.hpp"
#include "crane_world_observer/world_model.hpp"

#include <memory>

struct RobotIO
{
  std::shared_ptr<RobotInfo> info;
  std::shared_ptr<RobotCommandBuilder> builder;
  bool extractRobotInfo(std::shared_ptr<WorldModel> world_model, uint8_t id)
  {
    if (id < 0 || id >= world_model->ours.robots.size()) {
      return false;
    }
    info = world_model->ours.robots.at(id);
    return true;
  }
  bool setupBuilder(std::shared_ptr<WorldModel> world_model)
  {
    builder = std::make_shared<RobotCommandBuilder>(world_model, info);

    return true;
  }
};
#endif  // CRANE_BT_EXECUTOR__ROBOT_IO_HPP_
