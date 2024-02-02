// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SIMPLE_PLANNER_HPP
#define CRANE_SIMPLE_PLANNER_HPP

#include <crane_geometry/pid_controller.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>

namespace crane
{
class SimplePlanner
{
public:
  SimplePlanner(rclcpp::Node & node) {}

  crane_msgs::msg::RobotCommands calculateControlTarget(
    const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model)
  {
    crane_msgs::msg::RobotCommands commands = msg;
    for (auto & command : commands.robot_commands) {
      auto robot = world_model->getOurRobot(command.robot_id);
      command.current_pose.x = robot->pose.pos.x();
      command.current_pose.y = robot->pose.pos.y();
      command.current_pose.theta = robot->pose.theta;

      //　2023/11/12 出力の目標角度制限をしたらVisionの遅れと相まってロボットが角度方向に発振したのでコメントアウトする
      // そしてこの過ちを再びおかさぬようここに残しておく． R.I.P.
      //        double MAX_THETA_DIFF = max_omega / 30.0f;
      //        // 1フレームで変化するthetaの量が大きすぎると急に回転するので制限する
      //        if (not command.target_theta.empty()) {
      //          double theta_diff =
      //            getAngleDiff(command.target_theta.front(), command.current_pose.theta);
      //          if (std::fabs(theta_diff) > MAX_THETA_DIFF) {
      //            theta_diff = std::copysign(MAX_THETA_DIFF, theta_diff);
      //          }
      //
      //          command.target_theta.front() = command.current_pose.theta + theta_diff;
      //        }

      command.current_ball_x = world_model->ball.pos.x();
      command.current_ball_y = world_model->ball.pos.y();
    }
    return commands;
  }
};
}  // namespace crane
#endif  //CRANE_SIMPLE_PLANNER_HPP
