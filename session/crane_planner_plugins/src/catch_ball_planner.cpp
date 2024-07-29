// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/catch_ball_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
CatchBallPlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
{
  std::vector<crane_msgs::msg::RobotCommand> commands;
  for (const auto & robot : robots) {
    auto command = std::make_shared<crane::RobotCommandWrapperPosition>(
      "catch_ball_planner", robot.robot_id, world_model);

    [[maybe_unused]] Point target_point = default_point;
    auto ball = world_model->ball.pos;

    // シュートチェック
    Vector2 norm_vec = getVerticalVec(getNormVec(command->getRobot()->pose.theta)) * 0.8;
    Segment receive_line(
      command->getRobot()->pose.pos + norm_vec, command->getRobot()->pose.pos - norm_vec);
    Segment ball_line(ball, ball + world_model->ball.vel.normalized() * 20.f);
    auto intersections =
      getIntersections(ball_line, Segment{receive_line.first, receive_line.second});

    if (not intersections.empty() && world_model->ball.vel.norm() > 0.3f) {
      // シュートブロック
      std::cout << "シュートブロック" << std::endl;
      auto result = getClosestPointAndDistance(ball_line, command->getRobot()->pose.pos);
      command->setTargetPosition(result.closest_point);
      command->setTargetTheta(getAngle(-world_model->ball.vel));
      if (command->getRobot()->getDistance(result.closest_point) > 0.2) {
        command->setTerminalVelocity(2.0);
      }
    } else {
      if (
        world_model->ball.isStopped(0.3) &&
        not world_model->point_checker.isFriendPenaltyArea(ball)) {
        // ボールが止まっていて，味方ペナルティエリア内にあるときは，ペナルティエリア外に出す
        std::cout << "ボール排出" << std::endl;
        // パスできるロボットのリストアップ
        auto passable_robot_list = world_model->ours.getAvailableRobots(command->getRobot()->id);
        passable_robot_list.erase(
          std::remove_if(
            passable_robot_list.begin(), passable_robot_list.end(),
            [&](const RobotInfo::SharedPtr & r) {
              // 敵に塞がれていたら除外
              Segment ball_to_robot_line(ball, r->pose.pos);
              for (const auto & enemy : world_model->theirs.getAvailableRobots()) {
                auto dist = bg::distance(ball_to_robot_line, enemy->pose.pos);
                if (dist < 0.2) {
                  return true;
                }
              }
              return false;
            }),
          passable_robot_list.end());

        std::cout << "パスターゲット: ";
        for (auto a : passable_robot_list) {
          std::cout << static_cast<int>(a->id) << ",";
        }
        std::cout << std::endl;

        Point pass_target = [&]() {
          if (not passable_robot_list.empty()) {
            // TODO(HansRobo): いい感じのロボットを選ぶようにする
            return passable_robot_list.front()->pose.pos;
          } else {
            return Point{0, 0};
          }
        }();

        std::cout << pass_target.x() << ", " << pass_target.y() << std::endl;
        Point intermediate_point = ball + (ball - pass_target).normalized() * 0.2f;
        double angle_ball_to_target = getAngle(pass_target - ball);
        double dot = (world_model->ball.pos - command->getRobot()->pose.pos)
                       .normalized()
                       .dot((pass_target - world_model->ball.pos).normalized());
        // ボールと目標の延長線上にいない && 角度があってないときは，中間ポイントを経由
        if (
          dot < 0.9 ||
          std::abs(getAngleDiff(angle_ball_to_target, command->getRobot()->pose.theta)) > 0.1) {
          std::cout << "中間ポイント経由" << std::endl;
          command->setTargetPosition(intermediate_point);
          command->enableCollisionAvoidance();
        } else {
          std::cout << "ボール突撃" << std::endl;
          command->setTargetPosition(world_model->ball.pos);
          command->liftUpDribbler();
          command->kickStraight(0.1).disableCollisionAvoidance();
          command->enableCollisionAvoidance();
          command->disableBallAvoidance();
        }
        command->setTargetTheta(getAngle(pass_target - ball));
        command->disableGoalAreaAvoidance();
      } else {
        //          phase = "";
        [[maybe_unused]] const double BLOCK_DIST = 0.15;
        // 範囲外のときは正面に構える
        if (not world_model->point_checker.isFieldInside(ball)) {
          //            phase += "正面で";
          ball << 0, 0;
        }

        //          phase = "ボールを待ち受ける";
        command->setTargetPosition(default_point);
        command->lookAtBall();
      }
    }

    commands.push_back(command->getMsg());
  }
  return {PlannerBase::Status::RUNNING, commands};
}

auto crane::CatchBallPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  return this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      return 100. / world_model->getSquareDistanceFromRobot(robot->id, default_point);
    },
    prev_roles);
}
}  // namespace crane
