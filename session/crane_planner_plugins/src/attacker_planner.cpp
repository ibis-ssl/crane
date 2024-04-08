// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/attacker_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
AttackerPlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
{
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
      world_model->getNearestRobotsWithDistanceFromPoint(world_model->ball.pos, our_robots);
    attacker_->setParameter("receiver_id", nearest_robot.first->id);
  }

  auto status = attacker_->run(visualizer);
  return {static_cast<PlannerBase::Status>(status), {attacker_->getRobotCommand()}};
}
}  // namespace crane
