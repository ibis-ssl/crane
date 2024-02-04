// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SIMPLE_AVOID_PLANNER_HPP
#define CRANE_SIMPLE_AVOID_PLANNER_HPP

#include <crane_geometry/pid_controller.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>

namespace crane
{
class SimpleAvoidPlanner
{
public:
  SimpleAvoidPlanner(rclcpp::Node & node) : logger(node.get_logger())
  {
    //    logger = node.get_logger();
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

  //  std::vectorM<Point> generateAvoidancePoints(
  //    const crane_msgs::msg::RobotCommand & command, WorldModelWrapper::SharedPtr world_model)
  //  {
  //    std::vector<Point> avoidance_points;
  //
  //  }

  std::optional<Point> getAvoidancePoint(
    const crane_msgs::msg::RobotCommand & command, WorldModelWrapper::SharedPtr world_model)
  {
    RCLCPP_INFO_STREAM(logger, "getAvoidancePoint");
    Point target;
    target << command.target_x.front(), command.target_y.front();
    auto robot = world_model->getOurRobot(command.robot_id);

    // 最初はロボット-ゴールの線分
    Segment latest_path;
    latest_path.first = robot->pose.pos;
    latest_path.second = target;

    std::vector<Point> avoidance_points;  // ここに回避点候補を入れる
    // 以下、ひたすら回避点候補を追加していく

    {
      // 味方ロボットを回避
      for (const auto & r : world_model->ours.getAvailableRobots()) {
        if (
          robot->id != r->id && bg::distance(r->geometry(), latest_path) <= r->geometry().radius) {
          auto avoidance_points_tmp =
            getAvoidancePoints(robot->pose.pos, r->geometry(), r->geometry().radius);
          avoidance_points.insert(
            avoidance_points.end(), avoidance_points_tmp.begin(), avoidance_points_tmp.end());
        }
      }

      // 敵ロボットを回避
      for (const auto & r : world_model->theirs.getAvailableRobots()) {
        if (bg::distance(r->geometry(), latest_path) <= r->geometry().radius) {
          auto avoidance_points_tmp =
            getAvoidancePoints(robot->pose.pos, r->geometry(), r->geometry().radius);
          avoidance_points.insert(
            avoidance_points.end(), avoidance_points_tmp.begin(), avoidance_points_tmp.end());
        }
      }

      // ボールプレイスメントエリアを回避
      if (not command.local_planner_config.disable_placement_avoidance) {
        if (auto ball_placement_area = world_model->getBallPlacementArea()) {
          auto avoidance_points_tmp =
            getAvoidancePoints(robot->pose.pos, *ball_placement_area, 0.1);
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
          auto avoidance_points_tmp =
            getAvoidancePoints(robot->pose.pos, world_model->ball.pos, 0.2);
          avoidance_points.insert(
            avoidance_points.end(), avoidance_points_tmp.begin(), avoidance_points_tmp.end());
        }
      }
    }

    // 回避点候補をフィルタ
    {
      for (auto it = avoidance_points.begin(); it != avoidance_points.end();) {
        if (not world_model->isFieldInside(*it)) {
          // フィールド外のavoidance_pointsを削除
          it = avoidance_points.erase(it);
        } else if ((*it - robot->pose.pos).dot(target - robot->pose.pos) > 0.) {
          // 進行方向フィルタ
          it = avoidance_points.erase(it);
        } else if ((*it - robot->pose.pos).norm() < 0.1) {
          // ロボットの近くの点は削除
          it = avoidance_points.erase(it);
        } else {
          ++it;
        }
      }
    }

    if (avoidance_points.empty()) {
      return std::nullopt;
    } else {
      // ゴールから一番近い回避点を選ぶ
      Point nearest_point;
      double min_dist = 1000000000;
      for (const auto & p : avoidance_points) {
        double dist = (p - target).norm();
        if (dist < min_dist) {
          min_dist = dist;
          nearest_point = p;
        }
      }
      return nearest_point;
    }
  }

  std::optional<Point> getAvoidancePoint(
    std::shared_ptr<RobotInfo> from_robot, crane_msgs::msg::LocalPlannerConfig config, Point to,
    WorldModelWrapper::SharedPtr world_model, int depth = 0)
  {
    //    Point target;
    //    target << command.target_x.front(), command.target_y.front();
    //    auto robot = world_model->getOurRobot(command.robot_id);
    //
    // 最初はロボット-ゴールの線分
    Segment latest_path;
    latest_path.first = from_robot->pose.pos;
    latest_path.second = to;

    std::vector<Point> avoidance_points;  // ここに回避点候補を入れる

    // // そのままが良いならそれが一番
    avoidance_points.push_back(to);
    filterAvoidancePointsByPlace(avoidance_points, config, world_model);
    filterAvoidancePointsByPath(
      avoidance_points, from_robot, config, from_robot->pose.pos, world_model);
    if (not avoidance_points.empty()) {
      return to;
    }

    // 以下、ひたすら回避点候補を追加していく
    // ただし、ゴールからたどっていく。
    {
      // 味方ロボットを回避
      for (const auto & r : world_model->ours.getAvailableRobots()) {
        if (
          from_robot->id != r->id &&
          bg::distance(r->geometry(), latest_path) <= r->geometry().radius) {
          auto avoidance_points_tmp =
            getAvoidancePoints(to, r->geometry(), r->geometry().radius + 0.2);
          avoidance_points.insert(
            avoidance_points.end(), avoidance_points_tmp.begin(), avoidance_points_tmp.end());
        }
      }

      // 敵ロボットを回避
      for (const auto & r : world_model->theirs.getAvailableRobots()) {
        if (bg::distance(r->geometry(), latest_path) <= r->geometry().radius) {
          auto avoidance_points_tmp =
            getAvoidancePoints(to, r->geometry(), r->geometry().radius + 0.2);
          avoidance_points.insert(
            avoidance_points.end(), avoidance_points_tmp.begin(), avoidance_points_tmp.end());
        }
      }

      // ボールプレイスメントエリアを回避
      if (not config.disable_placement_avoidance) {
        if (auto ball_placement_area = world_model->getBallPlacementArea()) {
          auto avoidance_points_tmp = getAvoidancePoints(to, *ball_placement_area, 0.1);
          avoidance_points.insert(
            avoidance_points.end(), avoidance_points_tmp.begin(), avoidance_points_tmp.end());
        }
      }

      // ゴールエリアを回避
      if (not config.disable_goal_area_avoidance) {
        auto ours = world_model->getOurDefenseArea();
        auto theirs = world_model->getTheirDefenseArea();
        if (bg::distance(ours, latest_path) <= 0.0) {
          auto avoidance_points_tmp = getAvoidancePoints(to, ours, 0.1);
          avoidance_points.insert(
            avoidance_points.end(), avoidance_points_tmp.begin(), avoidance_points_tmp.end());
        }
        if (bg::distance(theirs, latest_path) <= 0.0) {
          auto avoidance_points_tmp = getAvoidancePoints(to, theirs, 0.1);
          avoidance_points.insert(
            avoidance_points.end(), avoidance_points_tmp.begin(), avoidance_points_tmp.end());
        }
      }

      // ボールを回避
      if (not config.disable_ball_avoidance) {
        if (bg::distance(world_model->ball.pos, latest_path) < 0.1) {
          auto avoidance_points_tmp = getAvoidancePoints(to, world_model->ball.pos, 0.2);
          avoidance_points.insert(
            avoidance_points.end(), avoidance_points_tmp.begin(), avoidance_points_tmp.end());
        }
      }
    }

    RCLCPP_INFO_STREAM(
      logger, "avoidance points: " << avoidance_points.size() << " depth: " << depth);

    // 回避点候補をフィルタ
    filterAvoidancePointsByPlace(avoidance_points, config, world_model);
    RCLCPP_INFO_STREAM(logger, "avoidance points: " << avoidance_points.size());

    // ゴール -> 回避点のパスをチェック
    filterAvoidancePointsByPath(avoidance_points, from_robot, config, to, world_model);
    if (avoidance_points.empty()) {
      RCLCPP_INFO_STREAM(logger, "no acceptable avoidance points: target -> avoidance_points");
      return std::nullopt;
    }
    RCLCPP_INFO_STREAM(logger, "avoidance points: " << avoidance_points.size());
    std::vector<Point> first_path_filtered_points(avoidance_points);
    // 回避点 -> ロボットのパスをチェック
    filterAvoidancePointsByPath(
      avoidance_points, from_robot, config, from_robot->pose.pos, world_model);
    if (avoidance_points.empty()) {
      RCLCPP_INFO_STREAM(logger, "no acceptable avoidance points: avoidance_points -> robot");
      // 一回避けるだけではうまく行かないパターン。
      // ゴールから到達可能な回避点でゴールを置き換えて試してみる。
      Point best_candidate;
      double min_dist = 1000000000;
      for (const auto & p : first_path_filtered_points) {
        double dist = (p - from_robot->pose.pos).norm();
        if (dist < min_dist) {
          min_dist = dist;
          best_candidate = p;
        }
      }
      if (depth < 10) {
        return getAvoidancePoint(from_robot, config, best_candidate, world_model, depth + 1);
      } else {
        return from_robot->pose.pos;
      }
    }
    RCLCPP_INFO_STREAM(logger, "avoidance points: " << avoidance_points.size());

    // print all acceptable avoidance points
    std::stringstream ss;
    ss << "acceptable avoidance points: " << std::endl;
    for (const auto & p : avoidance_points) {
      ss << "\t" << p.x() << ", " << p.y() << std::endl;
    }
    RCLCPP_INFO_STREAM(logger, ss.str());

    //   OKならそのまま採用（最短を採用）
    Point best;
    double min_dist = 1000000000;

    for (const auto & p : avoidance_points) {
      std::vector<Point> tmp_points(avoidance_points);
      tmp_points.erase(std::remove(tmp_points.begin(), tmp_points.end(), p), tmp_points.end());
      double dist = (p - to).norm() + (p - from_robot->pose.pos).norm();
      if (dist < min_dist) {
        min_dist = dist;
        best = p;
      }
    }
    return best;
  }

  void filterAvoidancePointsByPlace(
    std::vector<Point> & points, crane_msgs::msg::LocalPlannerConfig config,
    WorldModelWrapper::SharedPtr world_model)
  {
    points.erase(
      std::remove_if(
        points.begin(), points.end(),
        [&](Point p) {
          if (not world_model->isFieldInside(p)) {
            // フィールド外のavoidance_pointsを削除
            return true;
          }
          //味方ロボットに近すぎる点を削除
          for (const auto & r : world_model->ours.getAvailableRobots()) {
            if ((p - r->pose.pos).norm() < 0.1) {
              return true;
            }
          }

          // 敵ロボットに近すぎる点を削除
          for (const auto & r : world_model->theirs.getAvailableRobots()) {
            if ((p - r->pose.pos).norm() < 0.1) {
              return true;
            }
          }

          // ボールプレイスメントエリア内の点を削除
          if (not config.disable_placement_avoidance) {
            if (auto ball_placement_area = world_model->getBallPlacementArea()) {
              if (bg::distance(p, ball_placement_area->segment) <= ball_placement_area->radius) {
                return true;
              }
            }
          }

          // ゴールエリア内の点を削除
          if (not config.disable_goal_area_avoidance) {
            auto ours = world_model->getOurDefenseArea();
            auto theirs = world_model->getTheirDefenseArea();
            if (bg::distance(ours, p) < 0.1) {
              return true;
            }
            if (bg::distance(theirs, p) < 0.1) {
              return true;
            }
          }

          // ボールに近すぎる点を削除
          if (not config.disable_ball_avoidance) {
            if (bg::distance(world_model->ball.pos, p) < 0.1) {
              return true;
            }
          }
          return false;
        }),
      points.end());
  }

  void filterAvoidancePointsByPath(
    std::vector<Point> & points, std::shared_ptr<RobotInfo> robot,
    crane_msgs::msg::LocalPlannerConfig config, Point from,
    WorldModelWrapper::SharedPtr world_model)
  {
    points.erase(
      std::remove_if(
        points.begin(), points.end(),
        [&](Point p) {
          Segment path{from, p};
          // 味方ロボットとの干渉をチェック
          for (const auto & r : world_model->ours.getAvailableRobots()) {
            if (r->id != robot->id && bg::distance(r->geometry(), path) <= 0.0) {
              return true;
            }
          }
          // 敵ロボットとの干渉をチェック
          for (const auto & r : world_model->theirs.getAvailableRobots()) {
            if (bg::distance(r->geometry(), path) <= 0.0) {
              return true;
            }
          }

          // ボールプレイスメントエリアとの干渉をチェック
          if (not config.disable_placement_avoidance) {
            if (auto ball_placement_area = world_model->getBallPlacementArea()) {
              if (bg::distance(ball_placement_area->segment, path) < ball_placement_area->radius) {
                return true;
              }
            }
          }

          // ゴールエリアとの干渉をチェック
          if (not config.disable_goal_area_avoidance) {
            auto ours = world_model->getOurDefenseArea();
            auto theirs = world_model->getTheirDefenseArea();
            if (bg::distance(ours, path) <= 0.0) {
              return true;
            }
            if (bg::distance(theirs, path) <= 0.0) {
              return true;
            }
          }

          // ボールとの干渉をチェック
          if (not config.disable_ball_avoidance) {
            if (bg::distance(world_model->ball.pos, path) <= 0.0) {
              return true;
            }
          }
          return false;
        }),
      points.end());
  }

  crane_msgs::msg::RobotCommands calculateControlTarget(
    const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model)
  {
    RCLCPP_INFO_STREAM(logger, "calculateControlTarget");
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

        if (
          auto avoidance_point =
            getAvoidancePoint(robot, command.local_planner_config, target, world_model)) {
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

  rclcpp::Logger logger;
};
}  // namespace crane
#endif  //  CRANE_SIMPLE_AVOID_PLANNER_HPP
