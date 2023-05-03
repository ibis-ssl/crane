// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__ROBOT_IO_HPP_
#define CRANE_BT_EXECUTOR__ROBOT_IO_HPP_

#include <memory>

#include "crane_bt_executor/utils/robot_command_builder.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"

struct RobotIO
{
  std::shared_ptr<RobotInfo> info;
  std::shared_ptr<RobotCommandBuilder> builder;
  bool extractRobotInfo(WorldModelWrapper::SharedPtr world_model, uint8_t id)
  {
    if (id < 0 || id >= world_model->ours.robots.size()) {
      return false;
    }
    info = world_model->ours.robots.at(id);
    return true;
  }
  bool setupBuilder(WorldModelWrapper::SharedPtr world_model)
  {
    builder = std::make_shared<RobotCommandBuilder>(world_model, info);

    return true;
  }
};
#endif  // CRANE_BT_EXECUTOR__ROBOT_IO_HPP_
