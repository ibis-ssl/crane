// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_geometry/ddps.hpp>
#include <crane_planner_plugins/skill_tactic.hpp>
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

auto BallPlacementSkillTactic::getSelectedRobots(
  [[maybe_unused]] uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  // ボールに近いロボットを1台選択
  auto selected_robots = this->getSelectedRobotsByScore(
    1, selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      if (robot->id == world_model->getOurGoalieId()) {
        // ゴールキーパーは選出しない
        return -100.;
      } else {
        // ボールに近いほどスコアが高い
        return 100.0 / std::max(robot->getSquareDistance(world_model->ball().pos), 0.01);
      }
    },
    prev_roles);
  if (selected_robots.empty()) {
    return {};
  } else {
    skill = std::make_shared<skills::SingleBallPlacement>(
      "ball_placement_skill_planner", selected_robots.front(), world_model);

    if (auto target = world_model->getBallPlacementTarget(); target.has_value()) {
      skill->setParameter("placement_x", target->x());
      skill->setParameter("placement_y", target->y());
    }
    return {selected_robots.front()};
  }
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

auto SubAttackerSkillTactic::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  if (world_model->point_checker.isInOurHalf(world_model->ball().pos)) {
    // ボールが自陣にあるときはサブアタッカーを配置しない
    return {};
  }
  auto points = crane::getDPPSPoints(world_model->ball().pos, 0.25, 10., 64);
  auto dpps_points = points | ranges::views::filter([&](const Point & p) {
                       return world_model->point_checker.isFieldInside(p) &&
                              not world_model->point_checker.isPenaltyArea(p);
                     }) |
                     ranges::to<std::vector>();
  auto best_it = ranges::max_element(dpps_points, ranges::less{}, [this](const Point & p) {
    return skills::SubAttacker::getPointScore(p, world_model->ball().pos, world_model);
  });
  Point best_position = (best_it != ranges::end(dpps_points)) ? *best_it : Point::Zero();
  auto selected = this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [this, best_position](const std::shared_ptr<RobotInfo> & robot) {
      return 100. - robot->getSquareDistance(best_position);
    },
    prev_roles);

  if (selected.empty()) {
    return {};
  } else {
    skill = std::make_shared<skills::SubAttacker>(
      "sub_attacker_skill_planner", selected.front(), world_model);
    return {selected.front()};
  }
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

auto SimpleKickOffSkillTactic::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  auto selected = this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      return 100. / robot->getSquareDistance(world_model->ball().pos);
    },
    prev_roles);

  if (selected.empty()) {
    return {};
  } else {
    skill = std::make_shared<skills::SimpleKickOff>(
      "simple_kick_off_skill_planner", selected.front(), world_model);
    return {selected.front()};
  }
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

auto BallNearByPositionerSkillTactic::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  auto selected = this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      if (robot->id == world_model->getOurGoalieId()) {
        // ゴールキーパーは選出しない
        return -100.;
      } else {
        return 100. / robot->getSquareDistance(world_model->ball().pos);
      }
    },
    prev_roles);

  int index = 0;
  for (auto robot : selected) {
    skills.emplace_back(
      std::make_shared<skills::BallNearByPositioner>(
        "ball_near_by_positioner_skill_planner", robot, world_model));
    skills.back()->setParameter("total_robot_number", static_cast<int>(selected.size()));
    skills.back()->setParameter("current_robot_index", index++);
    skills.back()->setParameter("line_policy", std::string("arc"));
    skills.back()->setParameter("positioning_policy", std::string("auto"));
    skills.back()->setParameter("robot_interval", 0.35);
    skills.back()->setParameter("margin_distance", 0.8);
  }

  return selected;
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

auto PlacementTargetNearByPositionerSkillTactic::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  auto selected = this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      return 100. / robot->getSquareDistance(world_model->ball().pos);
    },
    prev_roles);

  int index = 0;
  for (auto robot : selected) {
    skills.emplace_back(
      std::make_shared<skills::BallNearByPositioner>(
        "ball_near_by_positioner_skill_planner", robot, world_model));
    skills.back()->setParameter("total_robot_number", static_cast<int>(selected.size()));
    skills.back()->setParameter("current_robot_index", index++);
    skills.back()->setParameter("line_policy", std::string("arc"));
    skills.back()->setParameter("positioning_policy", std::string("goal"));
    skills.back()->setParameter("robot_interval", 0.35);
    skills.back()->setParameter("margin_distance", 0.8);
  }

  return selected;
}
}  // namespace crane
