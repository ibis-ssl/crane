// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__VO_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__VO_PLANNER_HPP_

#include <osqp-cpp/osqp++.h>

#include <Eigen/Dense>
#include <algorithm>
#include <crane_basics/pid_controller.hpp>
#include <crane_msg_wrappers/consai_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace crane
{
inline Eigen::Vector2d optimizeVelocity(
  const RobotInfo::SharedPtr & robot, const Eigen::Vector2d & goal_pos,
  const std::vector<RobotInfo::SharedPtr> & other_robots, double d_min)
{
  const int n = 2;  // 2次元ベクトル

  // 目的地への希望速度
  Velocity v_des = (goal_pos - robot->pose.pos).normalized() * 1.0;  // スケーリング係数は1.0

  // OSQPパラメータ設定
  osqp::OsqpInstance instance;

  // 目的関数の二次項：min (1/2) * x' * P * x + q' * x で P = I, q = -v_des
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(n, n);  // 単位行列
  Eigen::VectorXd q = -v_des;  // 希望速度に基づく目的関数の線形項

  // 制約の数（他のロボットとの衝突回避制約）
  int num_constraints = other_robots.size();

  // 制約行列 A * x <= b を構築
  Eigen::MatrixXd A(num_constraints, n);
  Eigen::VectorXd b(num_constraints);

  // 他のロボットとの相対位置と速度を基に制約を設定
  for (int i = 0; i < num_constraints; ++i) {
    const auto & other_robot = other_robots[i];
    Eigen::Vector2d p_ij = robot->pose.pos - other_robot->pose.pos;
    Eigen::Vector2d v_ij = robot->vel.linear - other_robot->vel.linear;

    Eigen::Vector2d normal = p_ij.normalized();
    A.row(i) = normal.transpose();
    b(i) = d_min + normal.dot(v_ij);
  }

  // OSQPデータ設定
  instance.objective_matrix = P.sparseView();
  instance.objective_vector = q;
  instance.constraint_matrix = A.sparseView();
  instance.lower_bounds.resize(num_constraints);  // 制約の下限は不要なため
  instance.upper_bounds = b;

  // OSQPソルバー
  osqp::OsqpSolver solver;
  osqp::OsqpSettings settings;
  solver.Init(instance, settings);  // 問題を初期化

  // 最適化を実行
  const osqp::OsqpExitCode exit_code = solver.Solve();

  if (exit_code == osqp::OsqpExitCode::kOptimal) {
    Eigen::Vector2d optimal_vel(solver.primal_solution()(0), solver.primal_solution()(1));
    return optimal_vel;
  } else {
    std::cerr << "最適化に失敗しました。OSQP exit code: " << static_cast<int>(exit_code)
              << std::endl;
    return Eigen::Vector2d::Zero();  // エラー時はゼロ速度を返す
  }
}
class VOPlanner
{
public:
  explicit VOPlanner(rclcpp::Node & node)
  {
    node.declare_parameter("max_vel", MAX_VEL);
    MAX_VEL = node.get_parameter("max_vel").as_double();
    visualizer = std::make_shared<ConsaiVisualizerWrapper>(node, "gridmap_local_planner");
  }

  crane_msgs::msg::RobotCommands calculateRobotCommand(
    const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model)
  {
    crane_msgs::msg::RobotCommands commands = msg;
    for (auto & command : commands.robot_commands) {
      Velocity ideal_vel;

      auto robot = world_model->getOurRobot(command.robot_id);
      if (not command.position_target_mode.empty()) {
        visualizer->addLine(
          robot->pose.pos.x(), robot->pose.pos.y(), command.position_target_mode.front().target_x,
          command.position_target_mode.front().target_y, 1);
      }
      if (command.local_planner_config.max_velocity > MAX_VEL) {
        command.local_planner_config.max_velocity = MAX_VEL;
      }
    }
    visualizer->publish();
    return commands;
  }

private:
  ConsaiVisualizerWrapper::SharedPtr visualizer;

  double MAX_VEL = 4.0;
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__VO_PLANNER_HPP_
