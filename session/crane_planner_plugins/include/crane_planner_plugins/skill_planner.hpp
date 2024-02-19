// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__SKILL_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__SKILL_PLANNER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <crane_robot_skills/skills.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

namespace crane
{
#define DEFINE_SKILL_PLANNER(CLASS_NAME)                                                          \
  class CLASS_NAME##SkillPlanner : public PlannerBase                                             \
  {                                                                                               \
  public:                                                                                         \
    std::shared_ptr<skills::CLASS_NAME> skill = nullptr;                                          \
    std::shared_ptr<RobotCommandWrapper> robot_command_wrapper = nullptr;                         \
    COMPOSITION_PUBLIC explicit CLASS_NAME##SkillPlanner(                                         \
      WorldModelWrapper::SharedPtr & world_model, ConsaiVisualizerWrapper::SharedPtr visualizer)  \
    : PlannerBase(#CLASS_NAME, world_model, visualizer)                                           \
    {                                                                                             \
    }                                                                                             \
    std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(          \
      const std::vector<RobotIdentifier> & robots) override                                       \
    {                                                                                             \
      if (not skill or not robot_command_wrapper) {                                               \
        return {PlannerBase::Status::RUNNING, {}};                                                \
      } else {                                                                                    \
        std::vector<crane_msgs::msg::RobotCommand> robot_commands;                                \
        auto status = skill->run(*robot_command_wrapper, visualizer);                             \
        return {static_cast<PlannerBase::Status>(status), {robot_command_wrapper->getMsg()}};     \
      }                                                                                           \
    }                                                                                             \
    auto getSelectedRobots(                                                                       \
      uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots)              \
      -> std::vector<uint8_t> override                                                            \
    {                                                                                             \
      auto robots = this->getSelectedRobotsByScore(                                               \
        selectable_robots_num, selectable_robots,                                                 \
        [this](const std::shared_ptr<RobotInfo> & robot) {                                        \
          return 15. - static_cast<double>(-robot->id);                                           \
        });                                                                                       \
      skill = std::make_shared<skills::CLASS_NAME>(robots.front(), world_model);                  \
      robot_command_wrapper = std::make_shared<RobotCommandWrapper>(robots.front(), world_model); \
      return {robots.front()};                                                                    \
    }                                                                                             \
  }

// DEFINE_SKILL_PLANNER(Goalie);

class GoalieSkillPlanner : public PlannerBase
{
public:
  std ::shared_ptr<skills::Goalie> skill = nullptr;

  std ::shared_ptr<RobotCommandWrapper> robot_command_wrapper = nullptr;

  COMPOSITION_PUBLIC explicit GoalieSkillPlanner(
    WorldModelWrapper ::SharedPtr & world_model, ConsaiVisualizerWrapper ::SharedPtr visualizer)
  : PlannerBase("Goalie", world_model, visualizer)
  {
  }

  std ::pair<Status, std ::vector<crane_msgs ::msg ::RobotCommand>> calculateRobotCommand(
    const std ::vector<RobotIdentifier> & robots) override
  {
    if (not skill or not robot_command_wrapper) {
      return {PlannerBase ::Status ::RUNNING, {}};
    } else {
      std ::vector<crane_msgs ::msg ::RobotCommand> robot_commands;
      auto status = skill->run(*robot_command_wrapper, visualizer);
      return {static_cast<PlannerBase ::Status>(status), {robot_command_wrapper->getMsg()}};
    }
  }
  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std ::vector<uint8_t> & selectable_robots)
    -> std ::vector<uint8_t> override
  {
    return {world_model->getOurGoalieId()};
  }
};

class BallPlacementSkillPlanner : public PlannerBase
{
public:
  std ::shared_ptr<skills::SingleBallPlacement> skill = nullptr;

  std ::shared_ptr<RobotCommandWrapper> robot_command_wrapper = nullptr;

  COMPOSITION_PUBLIC explicit BallPlacementSkillPlanner(
    WorldModelWrapper ::SharedPtr & world_model, ConsaiVisualizerWrapper ::SharedPtr visualizer)
  : PlannerBase("BallPlacement", world_model, visualizer)
  {
  }

  std ::pair<Status, std ::vector<crane_msgs ::msg ::RobotCommand>> calculateRobotCommand(
    const std ::vector<RobotIdentifier> & robots) override
  {
    if (not skill or not robot_command_wrapper) {
      return {PlannerBase ::Status ::RUNNING, {}};
    } else {
      if (auto target = world_model->getBallPlacementTarget(); target.has_value()) {
        skill->setParameter("placement_x", target->x());
        skill->setParameter("placement_y", target->y());
      }
      std ::vector<crane_msgs ::msg ::RobotCommand> robot_commands;
      auto status = skill->run(*robot_command_wrapper, visualizer);
      return {static_cast<PlannerBase ::Status>(status), {robot_command_wrapper->getMsg()}};
    }
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std ::vector<uint8_t> & selectable_robots)
    -> std ::vector<uint8_t> override
  {
    // ボールに近いロボットを1台選択
    auto selected_robots = this->getSelectedRobotsByScore(
      1, selectable_robots, [this](const std::shared_ptr<RobotInfo> & robot) {
        // ボールに近いほどスコアが高い
        return 100.0 / std::max(world_model->getSquareDistanceFromRobotToBall(robot->id), 0.01);
      });
    skill = std ::make_shared<skills ::SingleBallPlacement>(selected_robots.front(), world_model);

    if (auto target = world_model->getBallPlacementTarget(); target.has_value()) {
      skill->setParameter("placement_x", target->x());
      skill->setParameter("placement_y", target->y());
    }

    robot_command_wrapper =
      std ::make_shared<RobotCommandWrapper>(selected_robots.front(), world_model);
    return {selected_robots.front()};
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__SKILL_PLANNER_HPP_
