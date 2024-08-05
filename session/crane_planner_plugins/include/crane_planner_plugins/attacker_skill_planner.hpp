// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__ATTACKER_SKILL_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__ATTACKER_SKILL_PLANNER_HPP_

#include <algorithm>
#include <crane_basics/boost_geometry.hpp>
#include <crane_basics/interval.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
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
class AttackerSkillPlanner : public PlannerBase
{
public:
  std::shared_ptr<skills::Attacker> skill = nullptr;

  COMPOSITION_PUBLIC explicit AttackerSkillPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("AttackerSkill", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    if (not skill) {
      return {PlannerBase::Status::RUNNING, {}};
    } else {
      std::string state_name(magic_enum::enum_name(skill->getCurrentState()));
      visualizer->addCircle(
        skill->commander().getRobot()->pose.pos, 0.3, 2, "red", "", 1.0, state_name);
      auto status = skill->run(visualizer);
      return {static_cast<PlannerBase::Status>(status), {skill->getRobotCommand()}};
    }
  }

  auto getSelectedRobots(
    [[maybe_unused]] uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override
  {
    if (auto our_frontier = world_model->getOurFrontier(); our_frontier) {
      auto base =
        std::make_shared<RobotCommandWrapperBase>("attacker", our_frontier->robot->id, world_model);
      skill = std::make_shared<skills::Attacker>(base);
      return {our_frontier->robot->id};
    } else {
      // nearest robot to ball
      auto selected_robots = this->getSelectedRobotsByScore(
        1, selectable_robots,
        [this](const std::shared_ptr<RobotInfo> & robot) {
          return 100.0 / std::max(world_model->getSquareDistanceFromRobotToBall(robot->id), 0.01);
        },
        prev_roles);
      if (selected_robots.empty()) {
        return {};
      } else {
        auto base = std::make_shared<RobotCommandWrapperBase>(
          "attacker", selected_robots.front(), world_model);
        skill = std::make_shared<skills::Attacker>(base);
        return selected_robots;
      }
    }
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__ATTACKER_SKILL_PLANNER_HPP_
