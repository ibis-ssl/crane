// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/receive_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
ReceivePlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
{
  if (not receiver_skill) {
    return {PlannerBase::Status::RUNNING, {}};
  } else {
    std::vector<crane_msgs::msg::RobotCommand> robot_commands;
    auto status = receiver_skill->run(visualizer);
    return {static_cast<PlannerBase::Status>(status), {receiver_skill->getRobotCommand()}};
  }
}

auto ReceivePlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  auto dpps_points = skills::Receiver::getDPPSPoints(world_model->ball.pos, 0.25, 64, world_model);
  double best_score = 0.0;
  Point best_position;
  for (const auto & dpps_point : dpps_points) {
    double score = skills::Receiver::getPointScore(dpps_point, world_model->ball.pos, world_model);
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
    prev_roles,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      // ヒステリシスは2m
      return 2.;
    });
  if (selected.empty()) {
    return {};
  } else {
    receiver_skill = std::make_shared<skills::Receiver>(selected.front(), world_model);
    return {selected.front()};
  }
}
}  // namespace crane
