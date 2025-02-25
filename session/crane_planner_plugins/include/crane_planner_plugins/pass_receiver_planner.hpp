// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__PASS_RECEIVER_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__PASS_RECEIVER_PLANNER_HPP_

#include <algorithm>
#include <crane_basics/boost_geometry.hpp>
#include <crane_basics/interval.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_plugins/planner_base.hpp>
#include <crane_robot_skills/receive.hpp>
#include <crane_robot_skills/robot_command_as_skill.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class PassReceiverPlanner : public PlannerBase
{
public:
  std::shared_ptr<skills::Receive> receive_skill = nullptr;

  int pass_receiver_id = 0;

  Point pass_target;

  COMPOSITION_PUBLIC explicit PassReceiverPlanner(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  : PlannerBase("PassReceiver", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots, PlannerContext & context) override
  {
    if (receive_skill) {
      auto command = receive_skill->getRobotCommand();
      return {PlannerBase::Status::RUNNING, {command}};
    } else {
      return {PlannerBase::Status::RUNNING, {}};
    }
  }

  auto getSelectedRobots(
    [[maybe_unused]] uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
    -> std::vector<uint8_t> override
  {
    pass_receiver_id = -1;
    if (auto planner_context = context.find("AttackerSkill"); planner_context != context.end()) {
      if (auto pass_receiver = planner_context->second.find("pass_receiver");
          pass_receiver != planner_context->second.end()) {
        pass_receiver_id = static_cast<int>(pass_receiver->second);
      }
    }

    if (std::ranges::count(selectable_robots, pass_receiver_id) == 0) {
      receive_skill = nullptr;
      return {};
    } else {
      auto base =
        std::make_shared<RobotCommandWrapperBase>("pass_receiver", pass_receiver_id, world_model);
      receive_skill = std::make_shared<skills::Receive>(base);
      return {pass_receiver_id};
    }
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__PASS_RECEIVER_PLANNER_HPP_
