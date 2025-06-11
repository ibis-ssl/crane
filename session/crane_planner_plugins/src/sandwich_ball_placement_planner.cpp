// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/sandwich_ball_placement_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
SandwichBallPlacementPlanner::calculateRobotCommand(
  const std::vector<RobotIdentifier> &, PlannerContext &)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  auto ball = world_model->ball().pos;

  switch (state) {
    case State::PREPARE: {
      double dx = std::abs(world_model->fieldSize().x() * 0.5 - std::abs(ball.x()));
      double dy = std::abs(world_model->fieldSize().y() * 0.5 - std::abs(ball.y()));
      Point target_1, target_2;
      if (dx > dy) {
        // 長辺に近い
        sandwich_direction << 1., 0.;
      } else {
        sandwich_direction << 0., 1.;
      }

      if (auto target = world_model->getBallPlacementTarget(); target) {
        // ターゲットの方向
        sandwich_direction = (target.value() - ball).normalized();
      }

      // 引く側
      placers.first->setTargetPosition(ball + sandwich_direction * 0.2)
        .lookAtBall()
        .setMaxVelocity(1.5)
        .enableBallAvoidance()
        .disableCollisionAvoidance()
        .disableGoalAreaAvoidance()
        .disablePlacementAvoidance();
      // 押す側
      placers.second->setTargetPosition(ball - sandwich_direction * 0.2)
        .lookAtBall()
        .setMaxVelocity(1.5)
        .enableBallAvoidance()
        .disableCollisionAvoidance()
        .disableGoalAreaAvoidance()
        .disablePlacementAvoidance();

      if (placers.first->getTargetDistance() < 0.05 && placers.second->getTargetDistance() < 0.05) {
        state = State::APPROACH;
        last_ball = world_model->ball().pos;
      }
      break;
    }
    case State::APPROACH: {
      placers.first->lookAt(last_ball)
        .setDribblerTargetPosition(last_ball)
        .lookAt(last_ball)
        .dribble(0.2)
        .setMaxVelocity(0.2)
        .disableBallAvoidance()
        .disableCollisionAvoidance()
        .disableGoalAreaAvoidance()
        .disablePlacementAvoidance();
      placers.second->lookAt(last_ball)
        .setDribblerTargetPosition(last_ball - sandwich_direction * 0.05)
        .dribble(0.2)
        .setMaxVelocity(0.2)
        .disableBallAvoidance()
        .disableCollisionAvoidance()
        .disableGoalAreaAvoidance()
        .disablePlacementAvoidance();

      if ((last_ball - world_model->ball().pos).norm() > 0.3) {
        state = State::PREPARE;
      } else if (
        placers.first->getRobot()->vel.linear.norm() < 0.05 &&
        placers.second->getRobot()->vel.linear.norm() < 0.05 &&
        placers.first->getTargetDistance() < 0.05 && placers.second->getTargetDistance() < 0.05) {
        state = State::MOVE;
      }
      break;
    }
    case State::MOVE: {
      if (auto target = world_model->getBallPlacementTarget(); target) {
        double to_target_angle = getAngle(target.value() - last_ball);
        placers.first->setDribblerTargetPosition(target.value())
          .setTargetTheta(to_target_angle + M_PI)
          .dribble(0.2)
          .setMaxVelocity(1.0);
        placers.second->setDribblerTargetPosition(target.value())
          .setTargetTheta(to_target_angle)
          .dribble(0.2)
          .setMaxVelocity(1.2);

        if (
          placers.first->getTargetDistance() < 0.10 && placers.second->getTargetDistance() < 0.10) {
          placers.first->stopHere();
          placers.second->stopHere();
          state = State::LEAVE;
        }
      } else {
        state = State::PREPARE;
      }
      break;
    }
    case State::LEAVE: {
      if (auto target = world_model->getBallPlacementTarget(); target) {
        double to_target_angle = getAngle(target.value() - last_ball);
        placers.first->setTargetPosition(target.value() + sandwich_direction * 0.4)
          .setTargetTheta(to_target_angle + M_PI)
          .dribble(0.0)
          .setMaxVelocity(0.2);
        placers.second->setTargetPosition(target.value() - sandwich_direction * 0.4)
          .setTargetTheta(to_target_angle)
          .dribble(0.0)
          .setMaxVelocity(0.2);
      } else {
        state = State::PREPARE;
      }
      break;
    }
  }
  robot_commands.emplace_back(placers.first->getMsg());
  robot_commands.emplace_back(placers.second->getMsg());

  return {PlannerBase::Status::RUNNING, robot_commands};
}

auto SandwichBallPlacementPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
  -> std::vector<uint8_t>
{
  if (selectable_robots_num < 2 or selectable_robots.size() < 2) {
    return {};
  } else {
    auto selected = this->getSelectedRobotsByScore(
      2, selectable_robots,
      [this](const std::shared_ptr<RobotInfo> & robot) {
        return 100. - robot->getDistance(world_model->ball().pos);
      },
      prev_roles, context);
    if (selected.size() == 2) {
      placers.first = std::make_shared<crane::RobotCommandWrapper>(
        "sandwich_ball_placement_planner", selected[0], world_model);
      placers.second = std::make_shared<crane::RobotCommandWrapper>(
        "sandwich_ball_placement_planner", selected[1], world_model);
      return selected;
    } else {
      return {};
    }
  }
}
}  // namespace crane
