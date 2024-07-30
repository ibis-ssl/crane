// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/attacker_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
AttackerPlanner::calculateRobotCommand([[maybe_unused]] const std::vector<RobotIdentifier> & robots)
{
  if (robots.empty()) {
    return {PlannerBase::Status::RUNNING, {}};
  }
  auto our_robots = world_model->ours.getAvailableRobots(attacker_->getID());
  our_robots.erase(
    std::remove_if(
      our_robots.begin(), our_robots.end(),
      [&](const auto & robot) {
        bool erase_flag = false;
        if (auto role = PlannerBase::robot_roles->find(robot->id);
            role != PlannerBase::robot_roles->end()) {
          if (role->second.planner_name == "defender") {
            // defenderにはパスしない
            erase_flag = true;
          } else if (role->second.planner_name.find("goalie") != std::string::npos) {
            // キーパーにもパスしない
            erase_flag = true;
          }
        }
        return erase_flag;
      }),
    our_robots.end());

  if (not our_robots.empty()) {
    auto nearest_robot =
      world_model->getNearestRobotWithDistanceFromPoint(world_model->ball.pos, our_robots);
    attacker_->setParameter("receiver_id", nearest_robot.first->id);
  } else {
    std::cout << "No available robots from attacker" << std::endl;
    return {PlannerBase::Status::RUNNING, {}};
  }

  auto status = attacker_->run(visualizer);
  return {static_cast<PlannerBase::Status>(status), {attacker_->getRobotCommand()}};
}

auto AttackerPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  auto selected = this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      // ボールに近いほどスコアが高い
      return 100.0 - std::max(world_model->getSquareDistanceFromRobotToBall(robot->id), 0.01);
    },
    prev_roles,
    [this]([[maybe_unused]] const std::shared_ptr<RobotInfo> & robot) {
      // ヒステリシスは1m
      return 1.;
    });
  if (selected.empty()) {
    return {};
  } else {
    auto attacker_base = std::make_shared<RobotCommandWrapperBase>(
      "attacker_planner/attacker", selected.front(), world_model);
    attacker_ = std::make_shared<skills::SimpleAttacker>(attacker_base);
    return {selected.front()};
  }
}
}  // namespace crane
