// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/local_planner.hpp"

namespace crane
{
void LocalPlannerComponent::callbackControlTarget(
  crane_msgs::msg::RobotCommands::ConstSharedPtr msg)
{
  if (!world_model_->hasUpdated()) {
    return;
  }
  // 味方ロボット：RVO内の位置・速度（＝進みたい方向）の更新
  for (const auto & friend_robot : world_model_->ours.robots) {
    if (not friend_robot->available) {
      rvo_sim_->setAgentPosition(friend_robot->id, RVO::Vector2(20.f, 20.f + friend_robot->id));
      rvo_sim_->setAgentPrefVelocity(friend_robot->id, RVO::Vector2(0.f, 0.f));
    } else {
      // Visionからのロボット位置の更新
      const auto & pos = friend_robot->pose.pos;
      rvo_sim_->setAgentPosition(friend_robot->id, RVO::Vector2(pos.x(), pos.y()));

      auto robot_target = std::find_if(
        msg->robot_commands.begin(), msg->robot_commands.end(),
        [&](const auto & x) { return x.robot_id == friend_robot->id; });

      if (robot_target == msg->robot_commands.end()) {
        // ロボットがcontrol_targetsに含まれていない場合、
        // 観測された速度をpreferred velocityとして設定する
        rvo_sim_->setAgentPrefVelocity(
          friend_robot->id,
          RVO::Vector2(friend_robot->vel.linear.x(), friend_robot->vel.linear.y()));
      } else {
        if (robot_target->motion_mode_enable) {
          // 速度制御モードの場合：速度司令をそのままRVOのpreferred velocityとして設定する
          rvo_sim_->setAgentPrefVelocity(
            friend_robot->id, RVO::Vector2(robot_target->target.x, robot_target->target.y));
        } else {
          // 位置制御モードの場合：目標位置方向に移動する速度ベクトルをRVOのpreferred velocityとして設定する
          auto diff_pos = Point(robot_target->target.x, robot_target->target.y) - pos;

          // 台形加速制御
          // TODO : 外部からパラメータを設定できるようにする
          constexpr double MAX_ACC = 10.0;
          constexpr double FRAME_RATE = 300;
          constexpr double MAX_SPEED = 10.0;
          std::cout << "current_speed: " << int(robot_target->robot_id) << std::endl;
          std::cout << "from: " << pos.x() << ", " << pos.y() << std::endl;
          std::cout << "to: " << robot_target->target.x << ", " << robot_target->target.y
                    << std::endl;
          // 2ax = v^2 - v0^2
          // v^2 - 2ax = v0^2
          // v0 = sqrt(v^2 - 2ax)
          double max_speed_for_stop = std::sqrt(0 * 0 - 2.0 * (-MAX_ACC) * diff_pos.norm());
          double max_speed_for_acc = friend_robot->vel.linear.norm() + MAX_ACC / FRAME_RATE;

          double target_speed = -std::min({max_speed_for_acc, max_speed_for_stop, MAX_SPEED});
          auto vel = diff_pos * target_speed / diff_pos.norm();
          std::cout << "target_vel: " << vel.x() << ", " << vel.y() << std::endl;
          rvo_sim_->setAgentPrefVelocity(friend_robot->id, RVO::Vector2(vel.x(), vel.y()));
        }
      }
    }
  }

  for (const auto & enemy_robot : world_model_->theirs.robots) {
    if (enemy_robot->available) {
      const auto & pos = enemy_robot->pose.pos;
      const auto & vel = enemy_robot->vel.linear;
      rvo_sim_->setAgentPosition(enemy_robot->id + 20, RVO::Vector2(pos.x(), pos.y()));
      rvo_sim_->setAgentPrefVelocity(enemy_robot->id + 20, RVO::Vector2(vel.x(), vel.y()));
    } else {
      rvo_sim_->setAgentPosition(enemy_robot->id + 20, RVO::Vector2(20.f, 20.f));
      rvo_sim_->setAgentPrefVelocity(enemy_robot->id + 20, RVO::Vector2(0.f, 0.f));
    }
  }

  // RVOシミュレータ更新
  rvo_sim_->doStep();

  // RVOシミュレータの結果をコマンドにコピー
  crane_msgs::msg::RobotCommands commands;
  //    commands.header = msg->header;
  //    commands.is_yellow = msg->is_yellow;
  for (size_t i = 0; i < msg->robot_commands.size(); i++) {
    const auto & target = msg->robot_commands.at(i);
    crane_msgs::msg::RobotCommand command = target;
    command.current_theta = world_model_->getRobot({true, target.robot_id})->pose.theta;
    // 位置制御モードの場合のみ，RVOシミュレータの出力をコピーする
    if (not target.motion_mode_enable) {
      std::cout << "robot_id " << int(target.robot_id) << std::endl;
      // RVOシミュレータの出力は速度なので，速度制御モードにする
      command.motion_mode_enable = true;
      auto vel = rvo_sim_->getAgentVelocity(target.robot_id);
      std::cout << "vel : " << vel.x() << " " << vel.y() << std::endl;
      command.target.x = vel.x();
      command.target.y = vel.y();
      command.target.theta = target.target.theta;
    } else {
      //        std::cout << "ROBOT_ID " << target.robot_id << std::endl;
    }
    commands.robot_commands.emplace_back(command);
  }
  commnads_pub_->publish(commands);
}
}  // namespace crane
#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::LocalPlannerComponent)
