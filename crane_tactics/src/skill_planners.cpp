// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_geometry/ddps.hpp>
#include <crane_tactics/skill_tactic.hpp>
#include <range/v3/algorithm/max_element.hpp>

namespace crane
{
auto GoalieSkillTactic::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
  -> std::pair<TacticBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
{
  // GlobalRobotAllocator対応: robotsが変更されたらスキルを再生成
  if (robots.empty()) {
    return {TacticBase::Status::RUNNING, {}};
  }
  if (not skill) {
    skill = std::make_shared<skills::Goalie>("goalie", robots.front().id, world_model);
  }

  auto status = skill->run();
  return {static_cast<TacticBase::Status>(status), {skill->getRobotCommand()}};
}

auto BallPlacementSkillTactic::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
  -> std::pair<TacticBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
{
  // GlobalRobotAllocator対応: robotsが変更されたらスキルを再生成
  if (robots.empty()) {
    return {TacticBase::Status::RUNNING, {}};
  }
  if (not skill) {
    skill = std::make_shared<skills::SingleBallPlacement>(
      "ball_placement_skill_planner", robots.front().id, world_model);
  }

  if (auto target = world_model->getBallPlacementTarget(); target.has_value()) {
    skill->setParameter("placement_x", target->x());
    skill->setParameter("placement_y", target->y());
  }
  auto status = skill->run();
  return {static_cast<TacticBase::Status>(status), {skill->getRobotCommand()}};
}

auto SubAttackerSkillTactic::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
  -> std::pair<TacticBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
{
  // GlobalRobotAllocator対応: robotsが変更されたらスキルを再生成
  if (robots.empty()) {
    return {TacticBase::Status::RUNNING, {}};
  }
  if (not skill) {
    skill = std::make_shared<skills::SubAttacker>(
      "sub_attacker_skill_planner", robots.front().id, world_model);
  }

  auto status = skill->run();
  return {static_cast<TacticBase::Status>(status), {skill->getRobotCommand()}};
}

auto SimpleKickOffSkillTactic::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
  -> std::pair<TacticBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
{
  // GlobalRobotAllocator対応: robotsが変更されたらスキルを再生成
  if (robots.empty()) {
    return {TacticBase::Status::RUNNING, {}};
  }
  if (not skill) {
    skill = std::make_shared<skills::SimpleKickOff>(
      "simple_kick_off_skill_planner", robots.front().id, world_model);
  }

  auto status = skill->run();
  return {static_cast<TacticBase::Status>(status), {skill->getRobotCommand()}};
}

auto BallNearByPositionerSkillTactic::calculatePositionCommand(
  const std::vector<RobotIdentifier> & robots)
  -> std::pair<TacticBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
{
  // GlobalRobotAllocator対応: robotsが変更されたらスキルを再生成
  if (skills.size() != robots.size()) {
    skills.clear();

    int index = 0;
    for (const auto & robot_id : robots) {
      skills.emplace_back(
        std::make_shared<skills::BallNearByPositioner>(
          "ball_near_by_positioner_skill_planner", robot_id.id, world_model));
      skills.back()->setParameter("total_robot_number", static_cast<int>(robots.size()));
      skills.back()->setParameter("current_robot_index", index++);
      skills.back()->setParameter("line_policy", std::string("arc"));
      skills.back()->setParameter("positioning_policy", std::string("auto"));
      skills.back()->setParameter("robot_interval", 0.35);
      skills.back()->setParameter("margin_distance", 0.8);
    }
  }

  auto robot_commands = skills | ranges::views::transform([this](const auto & skill) {
                          skill->run();
                          return skill->getRobotCommand();
                        }) |
                        ranges::to<std::vector<crane_msgs::msg::PositionCommand>>();
  return {TacticBase::Status::RUNNING, robot_commands};
}

auto PlacementTargetNearByPositionerSkillTactic::calculatePositionCommand(
  const std::vector<RobotIdentifier> & robots)
  -> std::pair<TacticBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
{
  // GlobalRobotAllocator対応: robotsが変更されたらスキルを再生成
  if (skills.size() != robots.size()) {
    skills.clear();

    int index = 0;
    for (const auto & robot_id : robots) {
      skills.emplace_back(
        std::make_shared<skills::BallNearByPositioner>(
          "ball_near_by_positioner_skill_planner", robot_id.id, world_model));
      skills.back()->setParameter("total_robot_number", static_cast<int>(robots.size()));
      skills.back()->setParameter("current_robot_index", index++);
      skills.back()->setParameter("line_policy", std::string("arc"));
      skills.back()->setParameter("positioning_policy", std::string("goal"));
      skills.back()->setParameter("robot_interval", 0.35);
      skills.back()->setParameter("margin_distance", 0.8);
    }
  }

  auto target = world_model->getBallPlacementTarget().value_or(world_model->ball().pos);
  auto robot_commands = skills | ranges::views::transform([&](const auto & skill) {
                          skill->setParameter("alternative_target_mode", true);
                          skill->setParameter("alternative_target", target);
                          skill->run();
                          return skill->getRobotCommand();
                        }) |
                        ranges::to<std::vector<crane_msgs::msg::PositionCommand>>();
  return {TacticBase::Status::RUNNING, robot_commands};
}

}  // namespace crane
