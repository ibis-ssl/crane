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

  std::vector<Point> getAvoidancePoints(
    const Point & point, const Circle & circle, const double offset)
  {
    Vector2 norm_vec;
    auto to_target = circle.center - point;
    norm_vec << to_target.y(), -to_target.x();
    norm_vec = norm_vec.normalized();
    std::vector<Point> avoidance_points;
    avoidance_points.emplace_back(circle.center + norm_vec * (circle.radius + offset));
    avoidance_points.emplace_back(circle.center - norm_vec * (circle.radius + offset));
    return avoidance_points;
  }

  std::vector<Point> getAvoidancePoints(
    const Point & point, const Point & target, const double offset)
  {
    Vector2 norm_vec;
    auto to_target = target - point;
    norm_vec << to_target.y(), -to_target.x();
    norm_vec = norm_vec.normalized();
    std::vector<Point> avoidance_points;
    avoidance_points.emplace_back(target + norm_vec * offset);
    avoidance_points.emplace_back(target - norm_vec * offset);
    return avoidance_points;
  }

  std::vector<Point> getAvoidancePoints(const Point & point, const Box & box, const double offset)
  {
    std::vector<Point> avoidance_points;
    avoidance_points.emplace_back(box.min_corner().x() - offset, box.min_corner().y() - offset);
    avoidance_points.emplace_back(box.min_corner().x() - offset, box.max_corner().y() + offset);
    avoidance_points.emplace_back(box.max_corner().x() + offset, box.min_corner().y() - offset);
    avoidance_points.emplace_back(box.max_corner().x() + offset, box.max_corner().y() + offset);
    return avoidance_points;
  }

  std::vector<Point> getAvoidancePoints(
    const Point & point, const Capsule & capsule, const double offset)
  {
    Vector2 seg_norm = (capsule.segment.first - capsule.segment.second).normalized();
    seg_norm << seg_norm.y(), -seg_norm.x();

    // カプセルからPointの方を向くベクトルを選ぶ
    if (seg_norm.dot(point - capsule.segment.first) < 0.) {
      seg_norm = -seg_norm;
    }

    std::vector<Point> avoidance_points;
    // 手前側の点
    avoidance_points.emplace_back(capsule.segment.first + seg_norm * (capsule.radius + offset));
    avoidance_points.emplace_back(capsule.segment.second + seg_norm * (capsule.radius + offset));
    // 横の点
    avoidance_points.emplace_back(
      capsule.segment.first +
      (capsule.segment.first - capsule.segment.second).normalized() * (capsule.radius + offset));
    avoidance_points.emplace_back(
      capsule.segment.second +
      (capsule.segment.second - capsule.segment.first).normalized() * (capsule.radius + offset));

    return avoidance_points;
  }

  std::optional<Point> getNearestAvoidancePoint(
    const crane_msgs::msg::RobotCommand & command, WorldModelWrapper::SharedPtr world_model)
  {
    Point target;
    target << command.target_x.front(), command.target_y.front();
    auto robot = world_model->getOurRobot(command.robot_id);

    Segment latest_path;
    latest_path.first = robot->pose.pos;
    latest_path.second = target;
    std::vector<Point> avoidance_points;

    // 味方ロボットを回避
    for (const auto & r : world_model->ours.getAvailableRobots()) {
      if (robot->id != r->id && bg::distance(r->geometry(), latest_path) < 0.0) {
        auto avoidance_points_tmp = getAvoidancePoints(robot->pose.pos, r->geometry(), 0.0);
        avoidance_points.insert(
          avoidance_points.end(), avoidance_points_tmp.begin(), avoidance_points_tmp.end());
      }
    }

    // 敵ロボットを回避
    for (const auto & r : world_model->theirs.getAvailableRobots()) {
      if (bg::distance(r->geometry(), latest_path) < 0.0) {
        auto avoidance_points_tmp = getAvoidancePoints(robot->pose.pos, r->geometry(), 0.0);
        avoidance_points.insert(
          avoidance_points.end(), avoidance_points_tmp.begin(), avoidance_points_tmp.end());
      }
    }

    // ボールプレイスメントエリアを回避
    if (not command.local_planner_config.disable_placement_avoidance) {
      if (auto ball_placement_area = world_model->getBallPlacementArea()) {
        auto avoidance_points_tmp = getAvoidancePoints(robot->pose.pos, *ball_placement_area, 0.1);
        avoidance_points.insert(
          avoidance_points.end(), avoidance_points_tmp.begin(), avoidance_points_tmp.end());
      }
    }

    // ゴールエリアを回避
    if (not command.local_planner_config.disable_goal_area_avoidance) {
      auto ours = world_model->getOurDefenseArea();
      auto theirs = world_model->getTheirDefenseArea();
      if (bg::distance(ours, latest_path) < 0.1) {
        auto avoidance_points_tmp = getAvoidancePoints(robot->pose.pos, ours, 0.1);
        avoidance_points.insert(
          avoidance_points.end(), avoidance_points_tmp.begin(), avoidance_points_tmp.end());
      }
      if (bg::distance(theirs, latest_path) < 0.1) {
        auto avoidance_points_tmp = getAvoidancePoints(robot->pose.pos, theirs, 0.1);
        avoidance_points.insert(
          avoidance_points.end(), avoidance_points_tmp.begin(), avoidance_points_tmp.end());
      }
    }

    // ボールを回避
    if (not command.local_planner_config.disable_ball_avoidance) {
      if (bg::distance(world_model->ball.pos, latest_path) < 0.1) {
        auto avoidance_points_tmp = getAvoidancePoints(robot->pose.pos, world_model->ball.pos, 0.2);
        avoidance_points.insert(
          avoidance_points.end(), avoidance_points_tmp.begin(), avoidance_points_tmp.end());
      }
    }


    for (auto it = avoidance_points.begin(); it != avoidance_points.end();) {
      if (not world_model->isFieldInside(*it)) {
        // フィールド外のavoidance_pointsを削除
        it = avoidance_points.erase(it);
      } else if ((*it - robot->pose.pos).dot(target - robot->pose.pos) > 0.) {
        // 進行方向フィルタ
        it = avoidance_points.erase(it);
      } else if((*it - robot->pose.pos).norm() < 0.1){
        // ロボットの近くの点は削除
        it = avoidance_points.erase(it);
      }else {
        ++it;
      }
    }

    if (avoidance_points.empty()) {
      return std::nullopt;
    } else {
      // choose nearest point
      Point nearest_point;
      double min_dist = 1000000000;
      for (const auto & p : avoidance_points) {
        double dist = (p - robot->pose.pos).norm();
        if (dist < min_dist) {
          min_dist = dist;
          nearest_point = p;
        }
      }
      return nearest_point;
    }
  }

  crane_msgs::msg::RobotCommands calculateControlTarget(
    const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model)
  {
    crane_msgs::msg::RobotCommands commands = msg;
    //    std::vector<Circle>
    for (auto & command : commands.robot_commands) {
      if ((not command.target_x.empty()) && (not command.target_y.empty())) {
        auto robot = world_model->getOurRobot(command.robot_id);
        command.current_pose.x = robot->pose.pos.x();
        command.current_pose.y = robot->pose.pos.y();
        command.current_pose.theta = robot->pose.theta;
        Point target;
        target << command.target_x.front(), command.target_y.front();

        if (auto avoidance_point = getNearestAvoidancePoint(command, world_model)) {
          target = *avoidance_point;
        }

        Point robot_to_target = target - robot->pose.pos;

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
          target.x() - command.current_pose.x, 1.f / 30.f),
          vy_controllers[command.robot_id].update(target.y() - command.current_pose.y, 1.f / 30.f);
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
