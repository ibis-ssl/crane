// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/local_planner.hpp"

namespace crane
{
void LocalPlannerComponent::reflectWorldToRVOSim(const crane_msgs::msg::RobotCommands & msg)
{
  // 味方ロボット：RVO内の位置・速度（＝進みたい方向）の更新
  for (const auto & command : msg.robot_commands) {
    rvo_sim->setAgentPosition(
      command.robot_id, RVO::Vector2(command.current_pose.x, command.current_pose.y));
    rvo_sim->setAgentPrefVelocity(command.robot_id, RVO::Vector2(0.f, 0.f));

    auto robot = world_model->getRobot({true, command.robot_id});

    float target_x = command.target_x.front();
    float target_y = command.target_y.front();

    if (command.motion_mode_enable) {
      // 速度制御モードの場合：速度司令をそのままRVOのpreferred velocityとして設定する
      rvo_sim->setAgentPrefVelocity(command.robot_id, RVO::Vector2(target_x, target_y));
    } else {
      // 位置制御モードの場合：目標位置方向に移動する速度ベクトルをRVOのpreferred velocityとして設定する
      const auto pos = robot->pose.pos;
      auto diff_pos = Point(target_x - pos.x(), target_y - pos.y());

      // 台形加速制御
      // TODO : 外部からパラメータを設定できるようにする
      constexpr double MAX_ACC = 4.0;
      constexpr double FRAME_RATE = 30;
      constexpr double MAX_SPEED = 10.0;
      // 2ax = v^2 - v0^2
      // v^2 - 2ax = v0^2
      // v0 = sqrt(v^2 - 2ax)
      double max_speed_for_stop = std::sqrt(0 * 0 - 2.0 * (-MAX_ACC) * diff_pos.norm());
      double max_speed_for_acc = robot->vel.linear.norm() + MAX_ACC / FRAME_RATE;

      double target_speed = std::min({max_speed_for_acc, max_speed_for_stop, MAX_SPEED});
      Velocity vel;
      double dist = diff_pos.norm();
      vel << target_speed / dist * diff_pos.x(), target_speed / dist * diff_pos.y();

      if (command.robot_id == 3) {
        std::cout << "current_robot: " << int(command.robot_id) << std::endl;
        std::cout << "from: " << pos.x() << ", " << pos.y() << std::endl;
        std::cout << "to: " << target_x << ", " << target_y << std::endl;
        std::cout << "diff_pos: " << diff_pos.x() << ", " << diff_pos.y() << std::endl;
        std::cout << "target_speed: " << target_speed << std::endl;
        std::cout << "target_vel: " << std::fixed << std::setprecision(5) << vel.x() << ", "
                  << vel.y() << std::endl;
      }
      rvo_sim->setAgentPrefVelocity(command.robot_id, RVO::Vector2(vel.x(), vel.y()));
    }
  }

  //  for (const auto & friend_robot : world_model->ours.robots) {
  //    if (not friend_robot->available) {
  //      rvo_sim->setAgentPosition(friend_robot->id, RVO::Vector2(20.f, 20.f + friend_robot->id));
  //      rvo_sim->setAgentPrefVelocity(friend_robot->id, RVO::Vector2(0.f, 0.f));
  //
  //    } else {
  //      // Visionからのロボット位置の更新
  //      const auto & pos = friend_robot->pose.pos;
  //      rvo_sim->setAgentPosition(friend_robot->id, RVO::Vector2(pos.x(), pos.y()));
  //
  //      auto robot_target = std::find_if(
  //        msg.robot_commands.begin(), msg.robot_commands.end(),
  //        [&](const auto & x) { return x.robot_id == friend_robot->id; });
  //
  //      if (robot_target == msg.robot_commands.end()) {
  //        // ロボットがcontrol_targetsに含まれていない場合、
  //        // 観測された速度をpreferred velocityとして設定する
  //        rvo_sim->setAgentPrefVelocity(
  //          friend_robot->id,
  //          RVO::Vector2(friend_robot->vel.linear.x(), friend_robot->vel.linear.y()));
  //      } else {
  //        float target_x = robot_target->target_x.front();
  //        float target_y = robot_target->target_y.front();
  //        if (robot_target->motion_mode_enable) {
  //          // 速度制御モードの場合：速度司令をそのままRVOのpreferred velocityとして設定する
  //          rvo_sim->setAgentPrefVelocity(friend_robot->id, RVO::Vector2(target_x, target_y));
  //        } else {
  //          // 位置制御モードの場合：目標位置方向に移動する速度ベクトルをRVOのpreferred velocityとして設定する
  //          auto diff_pos = Point(target_x, target_y) - pos;
  //
  //          // 台形加速制御
  //          // TODO : 外部からパラメータを設定できるようにする
  //          constexpr double MAX_ACC = 10.0;
  //          constexpr double FRAME_RATE = 300;
  //          constexpr double MAX_SPEED = 10.0;
  //          std::cout << "current_robot: " << int(robot_target->robot_id) << std::endl;
  //          std::cout << "from: " << pos.x() << ", " << pos.y() << std::endl;
  //          std::cout << "to: " << target_x << ", " << target_y << std::endl;
  //          // 2ax = v^2 - v0^2
  //          // v^2 - 2ax = v0^2
  //          // v0 = sqrt(v^2 - 2ax)
  //          double max_speed_for_stop = std::sqrt(0 * 0 - 2.0 * (-MAX_ACC) * diff_pos.norm());
  //          double max_speed_for_acc = friend_robot->vel.linear.norm() + MAX_ACC / FRAME_RATE;
  //
  //          double target_speed = -std::min({max_speed_for_acc, max_speed_for_stop, MAX_SPEED});
  //          auto vel = diff_pos * target_speed / diff_pos.norm();
  //          std::cout << "target_vel: " << vel.x() << ", " << vel.y() << std::endl;
  //          rvo_sim->setAgentPrefVelocity(friend_robot->id, RVO::Vector2(vel.x(), vel.y()));
  //        }
  //      }
  //    }
  //  }

  for (const auto & enemy_robot : world_model->theirs.robots) {
    if (enemy_robot->available) {
      const auto & pos = enemy_robot->pose.pos;
      const auto & vel = enemy_robot->vel.linear;
      rvo_sim->setAgentPosition(enemy_robot->id + 20, RVO::Vector2(pos.x(), pos.y()));
      rvo_sim->setAgentPrefVelocity(enemy_robot->id + 20, RVO::Vector2(vel.x(), vel.y()));
    } else {
      rvo_sim->setAgentPosition(enemy_robot->id + 20, RVO::Vector2(20.f, 20.f));
      rvo_sim->setAgentPrefVelocity(enemy_robot->id + 20, RVO::Vector2(0.f, 0.f));
    }
  }
}

// RVOシミュレータの結果をコマンドにコピー
crane_msgs::msg::RobotCommands LocalPlannerComponent::extractRobotCommandsFromRVOSim(
  const crane_msgs::msg::RobotCommands & msg)
{
  crane_msgs::msg::RobotCommands commands = msg;
  for (size_t i = 0; i < msg.robot_commands.size(); i++) {
    const auto & original_command = msg.robot_commands.at(i);
    const auto & robot = world_model->getRobot({true, original_command.robot_id});
    crane_msgs::msg::RobotCommand command = original_command;
    command.current_pose.x = robot->pose.pos.x();
    command.current_pose.y = robot->pose.pos.y();
    command.current_pose.theta = robot->pose.theta;
    // RVOシミュレータの出力をコピーする
    // NOTE: RVOシミュレータは角度を扱わないので角度はそのまま

    // RVOシミュレータの出力は速度なので，速度制御モードにする
    command.motion_mode_enable = true;
    auto vel = rvo_sim->getAgentVelocity(original_command.robot_id);
    if (original_command.robot_id == 3) {
      std::cout << "robot_id " << int(original_command.robot_id) << std::endl;
      std::cout << "vel : " << vel.x() << " " << vel.y() << std::endl;
    }
    command.target_velocity.x = vel.x();
    command.target_velocity.y = vel.y();

    commands.robot_commands.emplace_back(command);
  }
  return commands;
}

void LocalPlannerComponent::callbackControlTarget(const crane_msgs::msg::RobotCommands & msg)
{
  if (!world_model->hasUpdated()) {
    return;
  }

  if (enable_rvo) {
    reflectWorldToRVOSim(msg);

    // RVOシミュレータ更新
    rvo_sim->doStep();

    commnads_pub->publish(extractRobotCommandsFromRVOSim(msg));
  } else {
    crane_msgs::msg::RobotCommands commands = msg;
    for (auto & command : commands.robot_commands) {
      if ((not command.target_x.empty()) && (not command.target_y.empty())) {
        double dx = command.target_x.front() - command.current_pose.x;
        double dy = command.target_y.front() - command.current_pose.y;
        if (command.robot_id == 3) {
          std::cout << "diff : " << dx << " " << dy << std::endl;
        }
        double dist = std::sqrt(dx * dx + dy * dy);
        dist = (dist > 1.0) ? dist : 1.0;
        command.target_velocity.x = 4.0 * dx / dist;
        command.target_velocity.y = 4.0 * dy / dist;
      }
    }
    commnads_pub->publish(commands);
  }
}
}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::LocalPlannerComponent)
