// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__ATEB_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__ATEB_PLANNER_HPP_

#include <array>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "ateb_cbf_filter.hpp"
#include "ateb_spatial_optimizer.hpp"
#include "ateb_time_parameterizer.hpp"
#include "ateb_types.hpp"
#include "ateb_visibility_graph.hpp"
#include "planner_base.hpp"

namespace crane
{

/// トポロジー考慮型・時間最適軌道プランナ（ATEB Planner）
///
/// 4フェーズアーキテクチャ:
///   Phase 1: 縮約可視グラフによるホモトピークラス抽出
///   Phase 2: 解析的TEB（A-TEB）による空間最適化
///   Phase 3: ER-Force式時間最適速度プロファイル生成
///   Phase 4: CBF安全フィルタによるリアクティブ衝突回避
class ATEBPlanner : public LocalPlannerBase
{
public:
  explicit ATEBPlanner(rclcpp::Node & node);

  auto calculateRobotCommand(const crane_msgs::msg::RobotCommands & msg, double theta_offset)
    -> crane_msgs::msg::RobotCommands override;

private:
  // サブモジュール
  ateb::VisibilityGraph visibility_graph_;
  ateb::SpatialOptimizer spatial_optimizer_;
  ateb::TimeParameterizer time_parameterizer_;
  ateb::CBFFilter cbf_filter_;

  // ロボットごとの計画状態キャッシュ
  std::array<ateb::RobotPlanState, 20> robot_states_;

  // パラメータ
  double MAX_VEL = 5.0;
  double STOP_STATE_MAX_VELOCITY = 1.0;
  double FIELD_BOUNDARY_OFFSET = 0.2;
  double PENALTY_AREA_OFFSET = 0.1;
  double PENALTY_AREA_OFFSET_STOP = 0.3;
  double CRASH_SPEED_LIMIT = 1.5;
  double CRASH_SAFETY_MARGIN = 0.3;
  double CRASH_AVOIDANCE_DISTANCE = 1.0;
  double CRASH_AVOIDANCE_DECEL_DISTANCE = 0.5;
  double REPLAN_THRESHOLD = 0.05;

  /// セットプレイ中（INPLAY/HALT以外）かどうかを判定
  static auto needsExpandedPenaltyAreaOffset(uint8_t cmd) -> bool;

  /// ロボットが使用できる障害物リストを構築する
  [[nodiscard]] auto buildObstacles(uint8_t ego_id, const crane_msgs::msg::RobotCommand & cmd) const
    -> std::vector<ateb::Obstacle>;

  /// 1台のロボットの速度命令を計算する
  [[nodiscard]] auto planSingleRobot(const crane_msgs::msg::RobotCommand & cmd, double theta_offset)
    -> crane_msgs::msg::RobotCommand;

  /// 目標位置を各種回避ロジックで調整する（RVO2Plannerと同じロジック）
  auto overrideTargetPosition(crane_msgs::msg::RobotCommands & msg) -> void;

  auto adjustForPenaltyAreaAvoidance(
    Point & target_pos, const Point & current_pos,
    const crane_msgs::msg::RobotCommand & command) const -> void;

  auto adjustForBallAvoidance(
    Point & target_pos, const Point & current_pos,
    const crane_msgs::msg::RobotCommand & command) const -> void;

  auto adjustForPlacementAvoidance(
    Point & target_pos, const Point & current_pos,
    const crane_msgs::msg::RobotCommand & command) const -> void;

  auto adjustForFieldBoundary(
    Point & target_pos, const Point & current_pos,
    const crane_msgs::msg::RobotCommand & command) const -> void;
};

}  // namespace crane

#endif  // CRANE_LOCAL_PLANNER__ATEB_PLANNER_HPP_
