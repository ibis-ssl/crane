// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/skill_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
GoalieSkillPlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
{
  if (not skill) {
    return {PlannerBase::Status::RUNNING, {}};
  } else {
    auto status = skill->run(visualizer);
    return {static_cast<PlannerBase::Status>(status), {skill->getRobotCommand()}};
  }
}
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>

BallPlacementSkillPlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
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
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
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
    skill = std::make_shared<skills::SingleBallPlacement>(selected_robots.front(), world_model);

    if (auto target = world_model->getBallPlacementTarget(); target.has_value()) {
      skill->setParameter("placement_x", target->x());
      skill->setParameter("placement_y", target->y());
    }
    return {selected_robots.front()};
  }
}

std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
SubAttackerSkillPlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
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
    skill = std::make_shared<skills::SubAttacker>(selected.front(), world_model);
    return {selected.front()};
  }
}

std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
StealBallSkillPlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
{
  if (not skill) {
    return {PlannerBase::Status::RUNNING, {}};
  } else {
    auto status = skill->run(visualizer);
    return {static_cast<PlannerBase::Status>(status), {skill->getRobotCommand()}};
  }
}

auto StealBallSkillPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
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
    skill = std::make_shared<skills::StealBall>(selected_robots.front(), world_model);
    return {selected_robots.front()};
  }
}

std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
FreeKickSaverSkillPlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
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
    skill = std::make_shared<skills::FreeKickSaver>(selected.front(), world_model);
    return {selected.front()};
  }
}

std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
SimpleKickOffSkillPlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
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
    skill = std::make_shared<skills::SimpleKickOff>(selected.front(), world_model);
    return {selected.front()};
  }
}
}  // namespace crane
