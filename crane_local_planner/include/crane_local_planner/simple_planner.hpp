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
  SimplePlanner(rclcpp::Node & node)
  {
    node.declare_parameter("non_rvo_max_vel", NON_RVO_MAX_VEL);
    NON_RVO_MAX_VEL = node.get_parameter("non_rvo_max_vel").as_double();

    node.declare_parameter("non_rvo_p_gain", NON_RVO_P_GAIN);
    NON_RVO_P_GAIN = node.get_parameter("non_rvo_p_gain").as_double();
    node.declare_parameter("non_rvo_i_gain", NON_RVO_I_GAIN);
    NON_RVO_I_GAIN = node.get_parameter("non_rvo_i_gain").as_double();
    node.declare_parameter("non_rvo_d_gain", NON_RVO_D_GAIN);
    NON_RVO_D_GAIN = node.get_parameter("non_rvo_d_gain").as_double();

    for (auto & controller : vx_controllers) {
      controller.setGain(NON_RVO_P_GAIN, NON_RVO_I_GAIN, NON_RVO_D_GAIN);
    }

    for (auto & controller : vy_controllers) {
      controller.setGain(NON_RVO_P_GAIN, NON_RVO_I_GAIN, NON_RVO_D_GAIN);
    }
  }

  crane_msgs::msg::RobotCommands calculateControlTarget(
    const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model)
  {
    crane_msgs::msg::RobotCommands commands = msg;
    for (auto & command : commands.robot_commands) {
      if ((not command.target_x.empty()) && (not command.target_y.empty())) {
        auto robot = world_model->getOurRobot(command.robot_id);
        command.current_pose.x = robot->pose.pos.x();
        command.current_pose.y = robot->pose.pos.y();
        command.current_pose.theta = robot->pose.theta;
        Point target;
        target << command.target_x.front(), command.target_y.front();

        Point robot_to_target = target - robot->pose.pos;

        //        double dot1 = (world_model->ball.pos - robot->pose.pos).dot(robot_to_target);
        //        double dot2 = (-robot_to_target).dot(world_model->ball.pos - target);
        //        if (dot1 > 0 && dot2 > 0) {
        //          Point norm_vec;
        //          norm_vec << robot_to_target.y(), -robot_to_target.x();
        //          norm_vec = norm_vec.normalized();
        //          double dist_to_line = std::abs(norm_vec.dot(world_model->ball.pos - robot->pose.pos));
        //          if (dist_to_line < 0.1) {
        //            Point p1, p2;
        //            Point ball_est = world_model->ball.pos + world_model->ball.vel * 4.0;
        //            p1 = ball_est + 0.2 * norm_vec;
        //            p2 = ball_est - 0.2 * norm_vec;
        //            double d1 = (robot->pose.pos - p1).squaredNorm();
        //            double d2 = (robot->pose.pos - p2).squaredNorm();
        //            target = (d1 < d2) ? p1 : p2;
        //          }
        //          command.target_x.front() = target.x();
        //          command.target_y.front() = target.y();
        //        }

        double max_vel = command.local_planner_config.max_velocity > 0
                           ? command.local_planner_config.max_velocity
                           : NON_RVO_MAX_VEL;
        //        double max_acc = command.local_planner_config.max_acceleration > 0? command.local_planner_config.max_acceleration : NON_RVO_GAIN;
        double max_omega = command.local_planner_config.max_omega > 0
                             ? command.local_planner_config.max_omega
                             : 600.0 * M_PI / 180;

        // 速度に変換する
        Velocity vel;
        vel << vx_controllers[command.robot_id].update(
          command.target_x.front() - command.current_pose.x, 1.f / 30.f),
          vy_controllers[command.robot_id].update(
            command.target_y.front() - command.current_pose.y, 1.f / 30.f);
        vel += vel.normalized() * command.local_planner_config.terminal_velocity;
        if (vel.norm() > max_vel) {
          vel = vel.normalized() * max_vel;
        }

        command.target_velocity.x = vel.x();
        command.target_velocity.y = vel.y();

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
    }
    return commands;
  }

private:
  std::array<PIDController, 20> vx_controllers;
  std::array<PIDController, 20> vy_controllers;

  double NON_RVO_MAX_VEL = 4.0;
  double NON_RVO_P_GAIN = 4.0;
  double NON_RVO_I_GAIN = 0.0;
  double NON_RVO_D_GAIN = 0.0;
};
}  // namespace crane
#endif  //CRANE_SIMPLE_PLANNER_HPP
