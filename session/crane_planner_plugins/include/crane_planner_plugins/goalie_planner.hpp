// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__GOALIE_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__GOALIE_PLANNER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/control_target.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

namespace crane
{
class GoaliePlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit GoaliePlanner(WorldModelWrapper::SharedPtr & world_model)
  : PlannerBase("goalie", world_model)
  {
  }

  void emmitBallFromPenaltyArea(crane::RobotCommandWrapper & target)
  {
    auto ball = world_model->ball.pos;
    // パスできるロボットのリストアップ
    auto passable_robot_list = world_model->ours.getAvailableRobots();
    passable_robot_list.erase(
      std::remove_if(
        passable_robot_list.begin(), passable_robot_list.end(),
        [&](const RobotInfo::SharedPtr & r) {
          // 自分以外
          if (target.robot->id == r->id) {
            return true;
          }
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

    Point pass_target = [&]() {
      if (not passable_robot_list.empty()) {
        // TODO: いい感じのロボットを選ぶようにする
        return passable_robot_list.front()->pose.pos;
      } else {
        return Point{0, 0};
      }
    }();

    Point intermediate_point = ball + (ball - pass_target).normalized() * 0.2f;
    double angle_ball_to_target = getAngle(pass_target - ball);
    double dot = (world_model->ball.pos - target.robot->pose.pos)
                   .normalized()
                   .dot((pass_target - world_model->ball.pos).normalized());
    // ボールと目標の延長線上にいない && 角度があってないときは，中間ポイントを経由
    if (
      dot < 0.95 || std::abs(getAngleDiff(angle_ball_to_target, target.robot->pose.theta)) > 0.05) {
      target.setTargetPosition(intermediate_point);
      target.enableCollisionAvoidance();
    } else {
      target.setTargetPosition(world_model->ball.pos);
      target.kickStraight(0.4).disableCollisionAvoidance();
      target.enableCollisionAvoidance();
    }
    target.setTargetTheta(getAngle(pass_target - ball));
  }

  std::vector<crane_msgs::msg::RobotCommand> calculateControlTarget(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::vector<crane_msgs::msg::RobotCommand> control_targets;
    for (auto robot_id : robots) {
      crane::RobotCommandWrapper target(robot_id.robot_id, world_model);
      auto robot = world_model->getRobot(robot_id);
      auto ball = world_model->ball.pos;
      auto goals = world_model->getOurGoalPosts();

      // シュートチェック
      Segment goal_line(goals.first, goals.second);
      Segment ball_line(ball, ball + world_model->ball.vel.normalized() * 20.f);
      std::vector<Point> intersections;
      bg::intersection(ball_line, Segment{goals.first, goals.second}, intersections);

      std::cout << "goalie setup" << std::endl;

      if (not intersections.empty() && world_model->ball.vel.norm() > 0.5f) {
        // シュートブロック
        ClosestPoint result;
        bg::closest_point(ball_line, robot->pose.pos, result);
        target.setTargetPosition(result.closest_point);
        target.setTargetTheta(getAngle(-world_model->ball.vel));
      } else {
        if (world_model->ball.isStopped() && world_model->isFriendDefenseArea(ball)) {
          // ボールが止まっていて，味方ペナルティエリア内にあるときは，ペナルティエリア外に出す
          emmitBallFromPenaltyArea(target);
        } else {
          const double BLOCK_DIST = 0.15;
          // 範囲外のときは正面に構える
          if (not world_model->isFieldInside(ball)) {
            ball << 0, 0;
          }
          Point goal_center = world_model->getOurGoalCenter();
          goal_center << goals.first.x(), 0.0f;
          target.setTargetPosition(goal_center + (ball - goal_center).normalized() * BLOCK_DIST);
          target.setTargetTheta(getAngle(ball - robot->pose.pos));
        }
      }
      control_targets.emplace_back(target.getMsg());
    }
    return control_targets;
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots)
    -> std::vector<uint8_t> override
  {
    return this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots, [this](const std::shared_ptr<RobotInfo> & robot) {
        // choose id smaller first
        return 15. - static_cast<double>(-robot->id);
      });
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__GOALIE_PLANNER_HPP_
