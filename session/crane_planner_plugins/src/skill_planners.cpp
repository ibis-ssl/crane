// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/skill_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
GoalieSkillPlanner::calculateRobotCommand(
  [[maybe_unused]] const std::vector<RobotIdentifier> & robots)
{
  if (not skill) {
    return {PlannerBase::Status::RUNNING, {}};
  } else {
    auto status = skill->run(visualizer);
    return {static_cast<PlannerBase::Status>(status), {skill->getRobotCommand()}};
  }
}
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>

BallPlacementSkillPlanner::calculateRobotCommand(
  [[maybe_unused]] const std::vector<RobotIdentifier> & robots)
{
  if (not skill) {
    return {PlannerBase::Status::RUNNING, {}};
  } else {
    if (auto target = world_model->getBallPlacementTarget(); target.has_value()) {
      skill->setParameter("placement_x", target->x());
      skill->setParameter("placement_y", target->y());
    }
    auto status = skill->run(visualizer);
    return {static_cast<PlannerBase::Status>(status), {skill->getRobotCommand()}};
  }
}

auto BallPlacementSkillPlanner::getSelectedRobots(
  [[maybe_unused]] uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  // ボールに近いロボットを1台選択
  auto selected_robots = this->getSelectedRobotsByScore(
    1, selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      // ボールに近いほどスコアが高い
      return 100.0 / std::max(world_model->getSquareDistanceFromRobotToBall(robot->id), 0.01);
    },
    prev_roles);
  if (selected_robots.empty()) {
    return {};
  } else {
    auto base = std::make_shared<RobotCommandWrapperBase>(
      "ball_placement_skill_planner", selected_robots.front(), world_model);
    skill = std::make_shared<skills::SingleBallPlacement>(base);

    if (auto target = world_model->getBallPlacementTarget(); target.has_value()) {
      skill->setParameter("placement_x", target->x());
      skill->setParameter("placement_y", target->y());
    }
    return {selected_robots.front()};
  }
}

std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
SubAttackerSkillPlanner::calculateRobotCommand(
  [[maybe_unused]] const std::vector<RobotIdentifier> & robots)
{
  if (not skill) {
    return {PlannerBase::Status::RUNNING, {}};
  } else {
    auto status = skill->run(visualizer);
    return {static_cast<PlannerBase::Status>(status), {skill->getRobotCommand()}};
  }
}

auto SubAttackerSkillPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  auto dpps_points =
    skills::SubAttacker::getDPPSPoints(world_model->ball.pos, 0.25, 64, world_model);
  double best_score = 0.0;
  Point best_position;
  for (const auto & dpps_point : dpps_points) {
    double score =
      skills::SubAttacker::getPointScore(dpps_point, world_model->ball.pos, world_model);
    if (score > best_score) {
      best_score = score;
      best_position = dpps_point;
    }
  }
  auto selected = this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [this, best_position](const std::shared_ptr<RobotInfo> & robot) {
      return 100. - world_model->getSquareDistanceFromRobot(robot->id, best_position);
    },
    prev_roles);

  if (selected.empty()) {
    return {};
  } else {
    auto base = std::make_shared<RobotCommandWrapperBase>(
      "sub_attacker_skill_planner", selected.front(), world_model);
    skill = std::make_shared<skills::SubAttacker>(base);
    return {selected.front()};
  }
}

std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
StealBallSkillPlanner::calculateRobotCommand(
  [[maybe_unused]] const std::vector<RobotIdentifier> & robots)
{
  if (not skill) {
    return {PlannerBase::Status::RUNNING, {}};
  } else {
    auto status = skill->run(visualizer);
    return {static_cast<PlannerBase::Status>(status), {skill->getRobotCommand()}};
  }
}

auto StealBallSkillPlanner::getSelectedRobots(
  [[maybe_unused]] uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  auto selected_robots = [&]() {
    if (world_model->ball.vel.norm() < 0.5) {
      // ボールが遅いときはボールに近いロボットを1台選択
      return this->getSelectedRobotsByScore(
        1, selectable_robots,
        [this](const std::shared_ptr<RobotInfo> & robot) {
          // ボールに近いほどスコアが高い
          return 100.0 / std::max(world_model->getSquareDistanceFromRobotToBall(robot->id), 0.01);
        },
        prev_roles);
    } else {
      // ボールが速いときはボールラインに近いロボットを1台選択
      return this->getSelectedRobotsByScore(
        1, selectable_robots,
        [this](const std::shared_ptr<RobotInfo> & robot) {
          // ボールラインに近いほどスコアが高い
          Segment ball_line{
            world_model->ball.pos,
            world_model->ball.pos + world_model->ball.vel.normalized() * 10.0};
          return 100.0 /
                 std::max(getClosestPointAndDistance(robot->pose.pos, ball_line).distance, 0.01);
        },
        prev_roles);
    }
  }();
  if (selected_robots.empty()) {
    return {};
  } else {
    auto base = std::make_shared<RobotCommandWrapperBase>(
      "steal_ball_skill_planner", selected_robots.front(), world_model);
    skill = std::make_shared<skills::StealBall>(base);
    return {selected_robots.front()};
  }
}

std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
StealBallVelSkillPlanner::calculateRobotCommand(
  [[maybe_unused]] const std::vector<RobotIdentifier> & robots)
{
  if (not skill) {
    return {PlannerBase::Status::RUNNING, {}};
  } else {
    auto status = skill->run(visualizer);
    return {static_cast<PlannerBase::Status>(status), {skill->getRobotCommand()}};
  }
}

auto StealBallVelSkillPlanner::getSelectedRobots(
  [[maybe_unused]] uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  auto selected_robots = this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      return 100. / world_model->getSquareDistanceFromRobotToBall(robot->id);
    },
    prev_roles);

  if (selected_robots.empty()) {
    return {};
  } else {
    auto base = std::make_shared<RobotCommandWrapperBase>(
      "steal_ball__vel_skill_planner", selected_robots.front(), world_model);
    skill = std::make_shared<skills::StealBallVel>(base);
    return {selected_robots.front()};
  }
}

std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
FreeKickSaverSkillPlanner::calculateRobotCommand(
  [[maybe_unused]] const std::vector<RobotIdentifier> & robots)
{
  if (not skill) {
    return {PlannerBase::Status::RUNNING, {}};
  } else {
    auto status = skill->run(visualizer);
    return {static_cast<PlannerBase::Status>(status), {skill->getRobotCommand()}};
  }
}

auto FreeKickSaverSkillPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  auto selected = this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      return 100. / world_model->getSquareDistanceFromRobotToBall(robot->id);
    },
    prev_roles);

  if (selected.empty()) {
    return {};
  } else {
    auto base = std::make_shared<RobotCommandWrapperBase>(
      "free_kick_saver_skill_planner", selected.front(), world_model);
    skill = std::make_shared<skills::FreeKickSaver>(base);
    return {selected.front()};
  }
}

std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
SimpleKickOffSkillPlanner::calculateRobotCommand(
  [[maybe_unused]] const std::vector<RobotIdentifier> & robots)
{
  if (not skill) {
    return {PlannerBase::Status::RUNNING, {}};
  } else {
    auto status = skill->run(visualizer);
    return {static_cast<PlannerBase::Status>(status), {skill->getRobotCommand()}};
  }
}

auto SimpleKickOffSkillPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  auto selected = this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      return 100. / world_model->getSquareDistanceFromRobotToBall(robot->id);
    },
    prev_roles);

  if (selected.empty()) {
    return {};
  } else {
    auto base = std::make_shared<RobotCommandWrapperBase>(
      "simple_kick_off_skill_planner", selected.front(), world_model);
    skill = std::make_shared<skills::SimpleKickOff>(base);
    return {selected.front()};
  }
}

std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
BallNearByPositionerSkillPlanner::calculateRobotCommand(
  [[maybe_unused]] const std::vector<RobotIdentifier> & robots)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands(skills.size());
  std::transform(skills.begin(), skills.end(), robot_commands.begin(), [&](const auto & skill) {
    skill->run(visualizer);
    return skill->getRobotCommand();
  });
  return {PlannerBase::Status::RUNNING, robot_commands};
}

auto BallNearByPositionerSkillPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  auto selected = this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      return 100. / world_model->getSquareDistanceFromRobotToBall(robot->id);
    },
    prev_roles);

  int index = 0;
  for (auto robot : selected) {
    auto base = std::make_shared<RobotCommandWrapperBase>(
      "ball_near_by_positioner_skill_planner", robot, world_model);
    skills.emplace_back(std::make_shared<skills::BallNearByPositioner>(base));
    skills.back()->setParameter("total_robot_number", static_cast<int>(selected.size()));
    skills.back()->setParameter("current_robot_index", index++);
    skills.back()->setParameter("line_policy", std::string("arc"));
    skills.back()->setParameter("positioning_policy", std::string("goal"));
    skills.back()->setParameter("robot_interval", 0.35);
    skills.back()->setParameter("margin_distance", 0.35);
  }

  return selected;
}

}  // namespace crane
