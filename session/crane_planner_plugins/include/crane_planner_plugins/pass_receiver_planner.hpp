// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__PASS_RECEIVER_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__PASS_RECEIVER_PLANNER_HPP_

#include <algorithm>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/ddps.hpp>
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
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : PlannerBase("PassReceiver", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots, PlannerContext &) override
  {
    if (!receive_skill) {
      return {PlannerBase::Status::RUNNING, {}};
    }

    // If a kick is ongoing by our team or ball is moving sufficiently, actively receive
    const bool our_kick_ongoing = [&]() {
      const auto & ks = world_model->getMsg().game_analysis.ongoing_kick;
      return !ks.empty() && ks.front().is_kicker_friend;
    }();

    if (world_model->ball().isMoving(1.0) || our_kick_ongoing) {
      // Configure receive behavior
      receive_skill->setParameter("policy", std::string("closest"));
      // Mark reserved receiver clearly
      auto pos = receive_skill->commander()->getRobot()->pose.pos;
      visualizer->circle().center(pos).radius(0.25).stroke("cyan").strokeWidth(18).build();
      visualizer->text()
        .position(pos.x(), pos.y() + 0.32)
        .text("RECEIVER RESERVED")
        .fontSize(90)
        .fill("cyan")
        .textAnchor("middle")
        .build();
      auto status = receive_skill->run();
      return {static_cast<PlannerBase::Status>(status), {receive_skill->getRobotCommand()}};
    }

    // Pre-pass: no alignment needed; just stop and face the ball
    receive_skill->commander()->stopHere().lookAtBall();
    return {PlannerBase::Status::RUNNING, {receive_skill->getRobotCommand()}};
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
      receive_skill =
        std::make_shared<skills::Receive>("pass_receiver", pass_receiver_id, world_model);
      receive_skill->setParameter("policy", std::string("closest"));
      return {pass_receiver_id};
    }
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__PASS_RECEIVER_PLANNER_HPP_
