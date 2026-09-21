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
  * @brief 既に通過した経路を切り捨てる
  *
  * @param current 現在位置
  * @param path 経路
  * @return std::vector<Point> 切り詰めた経路
  */
  [[nodiscard]] static auto trimPathFromCurrent(
    const Point & current, const std::vector<Point> & path) -> std::vector<Point>;

  /**
  * @brief 経路にそって移動量分移動した先の位置を求める
  *
  * @param path 複数ポイントからなる経路
  * @param distance 移動量
  * @return Point 移動量分移動した先の位置
  */
  [[nodiscard]] static auto pointAtDistance(const std::vector<Point> & path, double distance)
    -> Point;

  /**
   * @brief 単一ロボットの経路計画を行う
   *
   * @param command 移動先の指令値
   * @param theta_offset (現状未使用)
   * @return crane_msgs::msg::RobotCommand 経路計画結果の移動先
   */
  [[nodiscard]] auto planSingleRobot(
    const crane_msgs::msg::RobotCommand & command, double theta_offset)
    -> crane_msgs::msg::RobotCommand;

  visibility_graph::VisibilityGraph visibility_graph_;
  std::array<RobotPathState, 20> path_states_;

  double max_velocity_ = 5.0;
  double stop_state_max_velocity_ = 1.0;
  double prediction_horizon_ = 0.5;
  double safety_margin_ = 0.03;
  double lookahead_distance_ = 0.30;
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
