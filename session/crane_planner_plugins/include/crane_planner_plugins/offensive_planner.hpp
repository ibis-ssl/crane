// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__OFFENSIVE_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__OFFENSIVE_PLANNER_HPP_

#include <algorithm>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_plugins/planner_base.hpp>
#include <crane_robot_skills/attacker.hpp>
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
class OffensivePlanner : public PlannerBase
{
public:
  std::shared_ptr<skills::Attacker> attacker = nullptr;

  COMPOSITION_PUBLIC explicit OffensivePlanner(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : PlannerBase("Offensive", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::string state_name(magic_enum::enum_name(attacker->getCurrentState()));
    {
      visualizer->circle()
        .center(attacker->commander()->getRobot()->pose.pos)
        .radius(0.3)
        .stroke("red")
        .strokeWidth(20)
        .build();
    }
    {
      visualizer->line()
        .start(world_model->ball().pos)
        .end(
          world_model->ball().pos +
          world_model->ball().vel.normalized() * world_model->getMsg().game_analysis.ball_horizon)
        .stroke("red")
        .strokeWidth(30)
        .build();
    }
    auto status = attacker->run();
    return {static_cast<PlannerBase::Status>(status), {attacker->getRobotCommand()}};
  }

  auto getSelectedRobots(
    [[maybe_unused]] uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override
  {
    // attackerを選択
    if (auto our_frontier = world_model->getOurFrontier(); our_frontier) {
      if (attacker == nullptr || attacker->commander()->getRobot()->id != our_frontier->robot->id) {
        attacker =
          std::make_shared<skills::Attacker>("attacker", our_frontier->robot->id, world_model);
      }
    } else {
      return {};
    }

    // PassReceiverを選択
    // PassReceiverが追加になると、Planner自体が作り直されてしまって内部ステートが受け継がれない...
    //
    return {attacker->commander()->getRobot()->id};
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__OFFENSIVE_PLANNER_HPP_
