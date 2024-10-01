// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__PLANNER_BASE_HPP_
#define CRANE_LOCAL_PLANNER__PLANNER_BASE_HPP_

#include <crane_msg_wrappers/consai_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>

namespace crane
{
class LocalPlannerBase
{
public:
  virtual crane_msgs::msg::RobotCommands calculateRobotCommand(
    const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model) = 0;

protected:
  ConsaiVisualizerWrapper::SharedPtr visualizer;
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__PLANNER_BASE_HPP_
