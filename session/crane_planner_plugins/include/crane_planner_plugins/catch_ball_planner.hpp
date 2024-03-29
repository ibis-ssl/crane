// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__CATCH_BALL_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__CATCH_BALL_PLANNER_HPP_

#include <crane_game_analyzer/evaluations/evaluations.hpp>
#include <crane_geometry/eigen_adapter.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/pass_info.hpp>
#include <crane_msgs/msg/receiver_plan.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <crane_msgs/srv/pass_request.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <functional>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class CatchBallPlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit CatchBallPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("catch_ball", world_model, visualizer)
  {
    default_point << -1.0, 0.;
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::vector<crane_msgs::msg::RobotCommand> commands;
    for (const auto & robot : robots) {
      crane::RobotCommandWrapper target(robot.robot_id, world_model);

      Point target_point = default_point;
      auto ball = world_model->ball.pos;

      // シュートチェック
      Vector2 norm_vec = getVerticalVec(getNormVec(target.robot->pose.theta)) * 0.8;
      Segment receive_line(target.robot->pose.pos + norm_vec, target.robot->pose.pos - norm_vec);
      Segment ball_line(ball, ball + world_model->ball.vel.normalized() * 20.f);
      std::vector<Point> intersections;
      bg::intersection(ball_line, Segment{receive_line.first, receive_line.second}, intersections);

      if (not intersections.empty() && world_model->ball.vel.norm() > 0.3f) {
        // シュートブロック
        std::cout << "シュートブロック" << std::endl;
        ClosestPoint result;
        bg::closest_point(ball_line, target.robot->pose.pos, result);
        target.setTargetPosition(result.closest_point);
        target.setTargetTheta(getAngle(-world_model->ball.vel));
        if (target.robot->getDistance(result.closest_point) > 0.2) {
          target.setTerminalVelocity(2.0);
        }
      } else {
        if (world_model->ball.isStopped(0.3) && not world_model->isFriendDefenseArea(ball)) {
          // ボールが止まっていて，味方ペナルティエリア内にあるときは，ペナルティエリア外に出す
          std::cout << "ボール排出" << std::endl;
          // パスできるロボットのリストアップ
          auto passable_robot_list = world_model->ours.getAvailableRobots(target.robot->id);
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
          double dot = (world_model->ball.pos - target.robot->pose.pos)
                         .normalized()
                         .dot((pass_target - world_model->ball.pos).normalized());
          // ボールと目標の延長線上にいない && 角度があってないときは，中間ポイントを経由
          if (
            dot < 0.9 ||
            std::abs(getAngleDiff(angle_ball_to_target, target.robot->pose.theta)) > 0.1) {
            std::cout << "中間ポイント経由" << std::endl;
            target.setTargetPosition(intermediate_point);
            target.enableCollisionAvoidance();
          } else {
            std::cout << "ボール突撃" << std::endl;
            target.setTargetPosition(world_model->ball.pos);
            target.liftUpDribbler();
            target.kickStraight(0.1).disableCollisionAvoidance();
            target.enableCollisionAvoidance();
            target.disableBallAvoidance();
          }
          target.setTargetTheta(getAngle(pass_target - ball));
          target.disableGoalAreaAvoidance();
        } else {
          //          phase = "";
          const double BLOCK_DIST = 0.15;
          // 範囲外のときは正面に構える
          if (not world_model->isFieldInside(ball)) {
            //            phase += "正面で";
            ball << 0, 0;
          }

          //          phase = "ボールを待ち受ける";
          target.setTargetPosition(default_point);
          target.lookAtBall();
        }
      }

      commands.push_back(target.getMsg());
    }
    return {PlannerBase::Status::RUNNING, commands};
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override
  {
    return this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots,
      [this](const std::shared_ptr<RobotInfo> & robot) {
        return 100. / world_model->getSquareDistanceFromRobot(robot->id, default_point);
      },
      prev_roles);
  }

private:
  rclcpp::TimerBase::SharedPtr timer;

  Point default_point;
};

}  // namespace crane

#endif  // CRANE_PLANNER_PLUGINS__CATCH_BALL_PLANNER_HPP_
