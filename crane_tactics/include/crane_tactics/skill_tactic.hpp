// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__SKILL_TACTIC_HPP_
#define CRANE_TACTICS__SKILL_TACTIC_HPP_

#include <algorithm>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/ball_nearby_positioner.hpp>
#include <crane_robot_skills/goalie.hpp>
#include <crane_robot_skills/simple_kickoff.hpp>
#include <crane_robot_skills/single_ball_placement.hpp>
#include <crane_robot_skills/sub_attacker.hpp>
#include <crane_tactics/tactic_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
#define DEFINE_SKILL_PLANNER(CLASS_NAME)                                                       \
  class CLASS_NAME##SkillTactic : public TacticBase                                            \
  {                                                                                            \
  public:                                                                                      \
    std::shared_ptr<skills::CLASS_NAME> skill = nullptr;                                       \
    COMPOSITION_PUBLIC explicit CLASS_NAME##SkillTactic(                                       \
      WorldModelWrapper::SharedPtr & world_model, [[maybe_unused]] rclcpp::Node & node)        \
    : TacticBase(#CLASS_NAME, world_model)                                                     \
    {                                                                                          \
    }                                                                                          \
    std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> calculatePositionCommand( \
      const std::vector<RobotIdentifier> & robots) override                                    \
    {                                                                                          \
      if (robots.empty()) {                                                                    \
        return {TacticBase::Status::RUNNING, {}};                                              \
      }                                                                                        \
      if (not skill) {                                                                         \
        skill = std::make_shared<skills::CLASS_NAME>(robots.front().id, world_model);          \
      }                                                                                        \
      std::vector<crane_msgs::msg::PositionCommand> robot_commands;                            \
      auto status = skill->run();                                                              \
      return {static_cast<TacticBase::Status>(status), {skill->getRobotCommand()}};            \
    }                                                                                          \
                                                                                               \
  protected:                                                                                   \
    void onRobotsChanged() override { skill.reset(); }                                         \
  }

class GoalieSkillTactic : public TacticBase
{
public:
  std::shared_ptr<skills::Goalie> skill = nullptr;

  COMPOSITION_PUBLIC explicit GoalieSkillTactic(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : TacticBase("Goalie", world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> override;
protected:
  void onRobotsChanged() override { skill.reset(); }
};

class BallPlacementSkillTactic : public TacticBase
{
public:
  std::shared_ptr<skills::SingleBallPlacement> skill = nullptr;

  COMPOSITION_PUBLIC explicit BallPlacementSkillTactic(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : TacticBase("BallPlacement", world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> override;

protected:
  void onRobotsChanged() override { skill.reset(); }
};

class SubAttackerSkillTactic : public TacticBase
{
public:
  std::shared_ptr<skills::SubAttacker> skill = nullptr;

  COMPOSITION_PUBLIC explicit SubAttackerSkillTactic(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : TacticBase("SubAttacker", world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> override;

protected:
  void onRobotsChanged() override { skill.reset(); }
};

class SimpleKickOffSkillTactic : public TacticBase
{
public:
  std::shared_ptr<skills::SimpleKickOff> skill = nullptr;

  COMPOSITION_PUBLIC explicit SimpleKickOffSkillTactic(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : TacticBase("SimpleKickOff", world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> override;

protected:
  void onRobotsChanged() override { skill.reset(); }
};

class BallNearByPositionerSkillTactic : public TacticBase
{
public:
  std::vector<std::shared_ptr<skills::BallNearByPositioner>> skills;

  COMPOSITION_PUBLIC explicit BallNearByPositionerSkillTactic(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : TacticBase("BallNearByPositionerSkill", world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> override;
};

class PlacementTargetNearByPositionerSkillTactic : public TacticBase
{
public:
  std::vector<std::shared_ptr<skills::BallNearByPositioner>> skills;

  COMPOSITION_PUBLIC explicit PlacementTargetNearByPositionerSkillTactic(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : TacticBase("PlacementTargetNearByPositionerSkill", world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> override;
};
}  // namespace crane
#endif  // CRANE_TACTICS__SKILL_TACTIC_HPP_
