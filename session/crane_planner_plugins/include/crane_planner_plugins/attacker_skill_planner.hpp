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
class AttackerSkillPlanner : public PlannerBase
{
public:
  std::shared_ptr<skills::Attacker> skill = nullptr;

  COMPOSITION_PUBLIC explicit AttackerSkillPlanner(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  : PlannerBase("AttackerSkill", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots, PlannerContext & context) override
  {
    if (not skill) {
      return {PlannerBase::Status::RUNNING, {}};
    } else {
      std::string state_name(magic_enum::enum_name(skill->getCurrentState()));
      {
        SvgCircleBuilder circle_builder;
        circle_builder.center(skill->commander().getRobot()->pose.pos)
          .radius(0.3)
          .stroke("red")
          .strokeWidth(20);
        visualizer->add(circle_builder.getSvgString());
      }
      if (world_model->ball.isMoving()) {
        {
          SvgPolyLineBuilder polyline_builder;
          for (auto [point, distance] : world_model->getBallSequence(2.0, 0.1)) {
            polyline_builder.addPoint(point);
          }
          polyline_builder.stroke("orange", 0.3).strokeWidth(100);
          visualizer->add(polyline_builder.getSvgString());
        }
      }
      auto status = skill->run();
      if (skill->getID() != robots.front().id) {
        std::stringstream ss;
        ss << "スキルのIDは" << static_cast<int>(skill->getID())
           << "ですが、選択されたロボットのIDは" << static_cast<int>(robots.front().id) << "です。";
        ss << "スキルのStateは" << magic_enum::enum_name(skill->getCurrentState()) << "です。";
        std::cout << ss.str() << std::endl;
      }
      return {static_cast<PlannerBase::Status>(status), {skill->getRobotCommand()}};
    }
  }

  auto getSelectedRobots(
    [[maybe_unused]] uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
    -> std::vector<uint8_t> override
  {
    if (auto our_frontier = world_model->getOurFrontier();
        our_frontier && ranges::contains(selectable_robots, our_frontier->robot->id)) {
      auto base =
        std::make_shared<RobotCommandWrapperBase>("attacker", our_frontier->robot->id, world_model);
      skill = std::make_shared<skills::Attacker>(base);
      return {our_frontier->robot->id};
    } else {
      // ボールに一番近いロボットを選択
      auto selected_robots = this->getSelectedRobotsByScore(
        1, selectable_robots,
        [this](const std::shared_ptr<RobotInfo> & robot) {
          // ボールに近いほどスコアが高い
          return 100.0 / std::max(world_model->getSquareDistanceFromRobotToBall(robot->id), 0.01);
        },
        prev_roles, context);
      if (not selected_robots.empty()) {
        auto base = std::make_shared<RobotCommandWrapperBase>(
          "attacker", selected_robots.front(), world_model);
        skill = std::make_shared<skills::Attacker>(base);
      }
      return {selected_robots.front()};
    }
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__ATTACKER_SKILL_PLANNER_HPP_
