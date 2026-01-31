// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PHYSICS__ALLOCATION_COST_HPP_
#define CRANE_PHYSICS__ALLOCATION_COST_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_physics/robot_info.hpp>
#include <memory>

namespace crane
{

/**
 * @brief ロボット割当コスト計算の設定パラメータ
 */
struct AllocationCostConfig
{
  // 距離コスト係数（1.0 = 1メートルあたりのコスト）
  double distance_weight = 1.0;

  // 速度アライメント係数（目標方向への速度を評価）
  // 例: 0.3 なら、速度1m/sで0.3mの距離に相当
  double velocity_alignment_weight = 0.3;

  // 到達時間係数（台形速度プロファイルでの到達時間を評価）
  // 例: 0.5 なら、1秒で0.5mの距離に相当
  double travel_time_weight = 0.5;

  // ヒステリシスボーナス（同一Session継続時のコスト減少）[m]
  double hysteresis_bonus = 0.5;

  // 速度ベースヒステリシス係数（高速移動中の再割当抑制）
  // 例: 0.2 なら、速度1m/sで0.2mのペナルティ
  double velocity_hysteresis_factor = 0.2;

  // Session優先度によるコスト調整（優先度差1あたりのコスト増加）[m]
  double priority_cost_multiplier = 10.0;

  // 最大加速度 [m/s^2]（到達時間計算用）
  double max_acceleration = 3.0;

  // 最大速度 [m/s]（到達時間計算用）
  double max_velocity = 3.0;

  // デフォルトコンストラクタ
  AllocationCostConfig() = default;
};

/**
 * @brief 割当コンテキスト情報
 */
struct AssignmentContext
{
  // 前回同じSessionに割り当てられていたか
  bool was_assigned_to_same_session = false;

  // 前回のターゲット位置（ヒステリシス計算用）
  std::optional<Point> prev_target_position;

  // Session優先度（数値が小さいほど優先度が高い）
  int session_priority = 100;

  // デフォルトコンストラクタ
  AssignmentContext() = default;
};

/**
 * @brief ロボット割当の統合コスト関数
 *
 * 距離、速度、ヒステリシス、優先度を考慮した総合的なコストを計算する。
 *
 * @param robot ロボット情報
 * @param target ターゲット位置
 * @param context 割当コンテキスト
 * @param config コスト計算設定
 * @return 総合コスト値（小さいほど良い）
 */
inline double calculateAllocationCost(
  const RobotInfo & robot, const Point & target, const AssignmentContext & context,
  const AllocationCostConfig & config = AllocationCostConfig())
{
  // 基本コスト: ユークリッド距離
  Vector2 to_target = target - robot.pose.pos;
  double distance = to_target.norm();
  double distance_cost = distance * config.distance_weight;

  // 速度考慮コスト: 目標方向への速度成分を評価
  double velocity_cost = 0.0;
  if (distance > 1e-6) {  // ゼロ除算回避
    Vector2 to_target_normalized = to_target.normalized();
    double velocity_alignment = robot.vel.linear.dot(to_target_normalized);
    // 目標方向への速度があればコスト減少、逆方向ならコスト増加
    velocity_cost = -velocity_alignment * config.velocity_alignment_weight;
  }

  // 到達時間ベースコスト（より正確な評価）
  double time_cost = 0.0;
  if (config.travel_time_weight > 0.0) {
    double travel_time = getTravelTimeTrapezoidal(
      robot.pose.pos, robot.vel.linear, target, config.max_acceleration, config.max_velocity);
    time_cost = travel_time * config.travel_time_weight;
  }

  // ヒステリシスボーナス（同じSessionに継続割当の場合コスト減少）
  double hysteresis_bonus = 0.0;
  if (context.was_assigned_to_same_session) {
    hysteresis_bonus = config.hysteresis_bonus;
  }

  // 速度ベースヒステリシス（Sumatra参考）
  // 高速移動中のロボットの再割当にペナルティを課す
  double velocity_hysteresis = 0.0;
  if (context.was_assigned_to_same_session) {
    double robot_speed = robot.vel.linear.norm();
    velocity_hysteresis = robot_speed * config.velocity_hysteresis_factor;
  }

  // 優先度コスト（優先度の低いSessionへの割当はコスト増加）
  double priority_cost = context.session_priority * config.priority_cost_multiplier;

  // 総合コスト
  // ボーナスは負の値として減算される
  return distance_cost + velocity_cost + time_cost - hysteresis_bonus + velocity_hysteresis +
         priority_cost;
}

/**
 * @brief 簡易版コスト関数（距離のみ）
 *
 * 後方互換性のための関数。既存コードとの互換性を保つ。
 *
 * @param robot_pos ロボット位置
 * @param target ターゲット位置
 * @return 距離コスト
 */
inline double calculateSimpleDistanceCost(const Point & robot_pos, const Point & target)
{
  return (target - robot_pos).norm();
}

/**
 * @brief SharedPtr版のコスト関数（WorldModelWrapper互換用）
 */
inline double calculateAllocationCost(
  const std::shared_ptr<RobotInfo> & robot, const Point & target, const AssignmentContext & context,
  const AllocationCostConfig & config = AllocationCostConfig())
{
  return calculateAllocationCost(*robot, target, context, config);
}

}  // namespace crane
#endif  // CRANE_PHYSICS__ALLOCATION_COST_HPP_
