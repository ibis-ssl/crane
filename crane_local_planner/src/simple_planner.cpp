// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/simple_planner.hpp"

namespace crane
{
SimplePlanner::SimplePlanner(rclcpp::Node & node) : logger(node.get_logger())
{
  //    logger = node.get_logger();
  node.declare_parameter("max_vel", MAX_VEL);
  MAX_VEL = node.get_parameter("max_vel").as_double();

  node.declare_parameter("p_gain", P_GAIN);
  P_GAIN = node.get_parameter("p_gain").as_double();
  node.declare_parameter("i_gain", I_GAIN);
  I_GAIN = node.get_parameter("i_gain").as_double();
  node.declare_parameter("d_gain", D_GAIN);
  D_GAIN = node.get_parameter("d_gain").as_double();

  for (auto & controller : vx_controllers) {
    controller.setGain(P_GAIN, I_GAIN, D_GAIN);
  }

  for (auto & controller : vy_controllers) {
    controller.setGain(P_GAIN, I_GAIN, D_GAIN);
  }
}
crane_msgs::msg::RobotCommands SimplePlanner::calculateRobotCommand(
  const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model)
{
  crane_msgs::msg::RobotCommands commands = msg;
  for (auto & command : commands.robot_commands) {
    if ((not command.target_x.empty()) && (not command.target_y.empty())) {
      auto robot = world_model->getOurRobot(command.robot_id);
      Point target;
      target << command.target_x.front(), command.target_y.front();

      Point robot_to_target = target - robot->pose.pos;

      double max_vel = command.local_planner_config.max_velocity > 0
                         ? command.local_planner_config.max_velocity
                         : MAX_VEL;
      // double max_acc = command.local_planner_config.max_acceleration > 0
      // ? command.local_planner_config.max_acceleration : GAIN;
      double max_omega = command.local_planner_config.max_omega > 0
                           ? command.local_planner_config.max_omega
                           : 600.0 * M_PI / 180;

      // 速度に変換する
      Velocity vel;
      vel << vx_controllers[command.robot_id].update(
        target.x() - command.current_pose.x, 1.f / 30.f),
        vy_controllers[command.robot_id].update(target.y() - command.current_pose.y, 1.f / 30.f);
      vel += vel.normalized() * command.local_planner_config.terminal_velocity;
      if (vel.norm() > max_vel) {
        vel = vel.normalized() * max_vel;
      }

      command.target_velocity.x = vel.x();
      command.target_velocity.y = vel.y();

      //    2023/11/12 出力の目標角度制限をしたらVisionの遅れと相まって
      //    ロボットが角度方向に発振したのでコメントアウトする
      //    そしてこの過ちを再びおかさぬようここに残しておく． R.I.P.
      //    double MAX_THETA_DIFF = max_omega / 30.0f;
      //    // 1フレームで変化するthetaの量が大きすぎると急に回転するので制限する
      //    if (not command.target_theta.empty()) {
      //      double theta_diff =
      //        getAngleDiff(command.target_theta.front(), command.current_pose.theta);
      //      if (std::fabs(theta_diff) > MAX_THETA_DIFF) {
      //        theta_diff = std::copysign(MAX_THETA_DIFF, theta_diff);
      //      }
      //
      //      command.target_theta.front() = command.current_pose.theta + theta_diff;
      //    }
    } else {
      command.target_velocity.x = 0.0;
      command.target_velocity.y = 0.0;
    }
  }
  return commands;
}
}  // namespace crane
