// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__SKILL_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__SKILL_PLANNER_HPP_

#include <algorithm>
#include <crane_basics/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <crane_robot_skills/skills.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
#define DEFINE_SKILL_PLANNER(CLASS_NAME)                                                          \
  class CLASS_NAME##SkillPlanner : public PlannerBase                                             \
  {                                                                                               \
  public:                                                                                         \
    std::shared_ptr<skills::CLASS_NAME> skill = nullptr;                                          \
    COMPOSITION_PUBLIC explicit CLASS_NAME##SkillPlanner(                                         \
      WorldModelWrapper::SharedPtr & world_model,                                                 \
      const ConsaiVisualizerWrapper::SharedPtr & visualizer)                                      \
    : PlannerBase(#CLASS_NAME, world_model, visualizer)                                           \
    {                                                                                             \
    }                                                                                             \
    std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(          \
      const std::vector<RobotIdentifier> & robots) override                                       \
    {                                                                                             \
      if (not skill) {                                                                            \
        return {PlannerBase::Status::RUNNING, {}};                                                \
      } else {                                                                                    \
        std::vector<crane_msgs::msg::RobotCommand> robot_commands;                                \
        auto status = skill->run(visualizer);                                                     \
        return {static_cast<PlannerBase::Status>(status), {skill->getRobotCommand()}};            \
      }                                                                                           \
    }                                                                                             \
    auto getSelectedRobots(                                                                       \
      uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,              \
      const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override \
    {                                                                                             \
      auto robots = getSelectedRobotsByScore(                                                     \
        selectable_robots_num, selectable_robots,                                                 \
        [this](const std::shared_ptr<RobotInfo> & robot) {                                        \
          return 15. - static_cast<double>(-robot->id);                                           \
        },                                                                                        \
        prev_roles);                                                                              \
      skill = std::make_shared<skills::CLASS_NAME>(robots.front(), world_model);                  \
      return {robots.front()};                                                                    \
    }                                                                                             \
  }

class GoalieSkillPlanner : public PlannerBase
{
public:
  std::shared_ptr<skills::Goalie> skill = nullptr;

  COMPOSITION_PUBLIC explicit GoalieSkillPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("Goalie", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getSelectedRobots(
    [[maybe_unused]] uint8_t selectable_robots_num,
    [[maybe_unused]] const std::vector<uint8_t> & selectable_robots,
    [[maybe_unused]] const std::unordered_map<uint8_t, RobotRole> & prev_roles)
    -> std::vector<uint8_t> override
  {
    auto base = std::make_shared<RobotCommandWrapperBase>(
      "goalie", world_model->getOurGoalieId(), world_model);
    skill = std::make_shared<skills::Goalie>(base);
    return {world_model->getOurGoalieId()};
  }
};

class BallPlacementSkillPlanner : public PlannerBase
{
public:
  std::shared_ptr<skills::SingleBallPlacement> skill = nullptr;

  COMPOSITION_PUBLIC explicit BallPlacementSkillPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("BallPlacement", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override;
};

class SubAttackerSkillPlanner : public PlannerBase
{
public:
  std::shared_ptr<skills::SubAttacker> skill = nullptr;

  COMPOSITION_PUBLIC explicit SubAttackerSkillPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("SubAttacker", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override;
};

class StealBallSkillPlanner : public PlannerBase
{
public:
  std::shared_ptr<skills::StealBall> skill = nullptr;

  COMPOSITION_PUBLIC explicit StealBallSkillPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("StealBall", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override;
};

class FreeKickSaverSkillPlanner : public PlannerBase
{
public:
  std::shared_ptr<skills::FreeKickSaver> skill = nullptr;

  COMPOSITION_PUBLIC explicit FreeKickSaverSkillPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("FreeKickSaver", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override;
};

class SimpleKickOffSkillPlanner : public PlannerBase
{
public:
  std::shared_ptr<skills::SimpleKickOff> skill = nullptr;

  COMPOSITION_PUBLIC explicit SimpleKickOffSkillPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("SimpleKickOff", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override;
};

class BallNearByPositionerSkillPlanner : public PlannerBase
{
public:
  std::vector<std::shared_ptr<skills::BallNearByPositioner>> skills;

  COMPOSITION_PUBLIC explicit BallNearByPositionerSkillPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("BallNearByPositionerSkill", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override;
};
}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__SKILL_PLANNER_HPP_
