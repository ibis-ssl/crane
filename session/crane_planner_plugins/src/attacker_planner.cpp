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
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  for (auto robot_id : robots) {
    crane::RobotCommandWrapper target(robot_id.robot_id, world_model);
    auto robot = world_model->getRobot(robot_id);

    auto [best_angle, goal_angle_width] =
      world_model->getLargestGoalAngleRangeFromPoint(world_model->ball.pos);
    Point best_target = world_model->ball.pos + getNormVec(best_angle) * 0.3;

    // シュートの隙がないときは仲間へパス
    if (goal_angle_width < 0.07) {
      auto our_robots = world_model->ours.getAvailableRobots(robot_id.robot_id);
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
        best_target = nearest_robot.first->pose.pos;
      }
    }

    // 経由ポイント
    Point intermediate_point =
      world_model->ball.pos + (world_model->ball.pos - best_target).normalized() * 0.2;

    double dot = (robot->pose.pos - world_model->ball.pos)
                   .normalized()
                   .dot((world_model->ball.pos - best_target).normalized());
    double target_theta = getAngle(best_target - world_model->ball.pos);
    // ボールと敵ゴールの延長線上にいない && 角度があってないときは，中間ポイントを経由
    if (dot < 0.9 || std::abs(getAngleDiff(target_theta, robot->pose.theta)) > 0.2) {
      std::cout << "中間地点経由" << std::endl;
      target.setTargetPosition(intermediate_point);
      target.enableCollisionAvoidance();
      target.liftUpDribbler();
    } else {
      std::cout << "キック" << std::endl;
      target.setTargetPosition(world_model->ball.pos);
      target.kickStraight(0.3).disableCollisionAvoidance();
      target.liftUpDribbler();
      target.enableCollisionAvoidance();
      target.disableBallAvoidance();
    }

    target.setTargetTheta(getAngle(best_target - world_model->ball.pos));

    bool is_in_defense = world_model->isDefenseArea(world_model->ball.pos);
    bool is_in_field = world_model->isFieldInside(world_model->ball.pos);

    if ((not is_in_field) or is_in_defense) {
      // stop here
      target.stopHere();
    }
    robot_commands.emplace_back(target.getMsg());
  }
  return {PlannerBase::Status::RUNNING, robot_commands};
}
}  // namespace crane
