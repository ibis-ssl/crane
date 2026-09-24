// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__VISIBILITY_GRAPH_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__VISIBILITY_GRAPH_PLANNER_HPP_

#include <array>
#include <chrono>
#include <vector>

#include "planner_base.hpp"
#include "visibility_graph.hpp"

namespace crane
{

class VisibilityGraphPlanner : public LocalPlannerBase
{
public:
  explicit VisibilityGraphPlanner(rclcpp::Node & node);
  /**
  * @brief 指令値の計算を行う
  *
  * @param msg 移動指令値
  * @param theta_offset
  * @return crane_msgs::msg::RobotCommands 計算後の指令値
  */
  auto calculateRobotCommand(const crane_msgs::msg::RobotCommands & msg, double theta_offset)
    -> crane_msgs::msg::RobotCommands override;

private:
  struct RobotPathState
  {
    std::vector<Point> path;
    Point goal = Point::Zero();
    bool valid = false;
    // 前周期に移動ロボットからの退避を出したか。退避判定のヒステリシスに使う
    bool escaping = false;
    std::chrono::steady_clock::time_point next_full_replan{};
  };
  /**
  * @brief 障害物を構築する
  *
  * @param robot_id 経路計画対象のロボットid
  * @param command 移動先の指令値
  * @return std::vector<visibility_graph::Obstacle> 構築した障害物
  */
  [[nodiscard]] auto buildObstacles(uint8_t robot_id, const crane_msgs::msg::RobotCommand & command)
    const -> std::vector<visibility_graph::Obstacle>;

  /**
  * @brief 再利用可能性などを考慮して経路を選択する
  *
  * @param robot_id 経路計画対象のロボットid
  * @param current 現在位置
  * @param goal 目標位置
  * @param obstacles 障害物
  * @return std::vector<Point> 選択された経路
  */
  [[nodiscard]] auto selectPath(
    uint8_t robot_id, const Point & current, const Point & goal,
    const std::vector<visibility_graph::Obstacle> & obstacles) -> std::vector<Point>;

  /**
   * @brief 単一ロボットの経路計画を行う
   *
   * @param command 移動先の指令値
   * @return crane_msgs::msg::RobotCommand 経路計画結果の移動先
   */
  [[nodiscard]] auto planSingleRobot(const crane_msgs::msg::RobotCommand & command)
    -> crane_msgs::msg::RobotCommand;

  visibility_graph::VisibilityGraph visibility_graph_;
  std::array<RobotPathState, 20> path_states_;

  double max_velocity_ = 5.0;
  double stop_state_max_velocity_ = 1.0;
  double prediction_horizon_ = 0.5;
  double safety_margin_ = 0.03;
  double lookahead_distance_ = 0.30;
  // 先読み点が不可視で中継点へフォールバックするときの、中継点までの最小弧長 [m]
  double min_subgoal_distance_ = 0.05;
  // 退避中に退避を解除するために必要な障害物外側への余裕 [m]
  double escape_release_margin_ = 0.03;
  // 退避先を障害物境界からどれだけ外側に置くか [m]。escape_release_margin_ より
  // ロボット側の到達許容誤差（CM4 は 0.01）ぶん以上大きくしないと退避が固着する
  double escape_clearance_ = 0.06;
  double replan_cross_track_distance_ = 0.50;
  double route_switch_improvement_ratio_ = 0.10;
  double goal_change_threshold_ = 0.05;
  double full_replan_interval_ = 0.5;
  double field_boundary_offset_ = 0.2;
  double penalty_area_offset_ = 0.1;
  double penalty_area_offset_stop_ = 0.4;
};

}  // namespace crane

#endif  // CRANE_LOCAL_PLANNER__VISIBILITY_GRAPH_PLANNER_HPP_
