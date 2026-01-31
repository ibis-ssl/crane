// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_COORDINATOR__GLOBAL_ROBOT_ALLOCATOR_HPP_
#define CRANE_SESSION_COORDINATOR__GLOBAL_ROBOT_ALLOCATOR_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_physics/allocation_cost.hpp>
#include <crane_physics/position_assignments.hpp>
#include <crane_physics/robot_info.hpp>
#include <crane_session_coordinator/allocation_state.hpp>
#include <functional>
#include <memory>
#include <rclcpp/logger.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace crane
{

/**
 * @brief Sessionのロボット要求情報
 */
struct SessionRequirement
{
  // Session名
  std::string name;

  // 優先度（数値が小さいほど優先度が高い、0が最高優先度）
  int priority;

  // 必要最小ロボット数
  int min_robots;

  // 必要最大ロボット数
  int max_robots;

  // ロボット適性評価関数（ロボット→コストを返す、小さいほど適している）
  std::function<double(const std::shared_ptr<RobotInfo> &)> suitability_func;

  // ハード制約フラグ（trueの場合、優先的に確保される）
  bool is_hard_constraint = false;

  SessionRequirement(
    std::string n, int p, int min_r, int max_r,
    std::function<double(const std::shared_ptr<RobotInfo> &)> func, bool hard = false)
  : name(std::move(n)),
    priority(p),
    min_robots(min_r),
    max_robots(max_r),
    suitability_func(std::move(func)),
    is_hard_constraint(hard)
  {
  }
};

/**
 * @brief 全体最適化ロボット割当クラス
 *
 * 複数のSessionからの要求を統一的に処理し、ハンガリアン法で全体最適な割当を行う。
 */
class GlobalRobotAllocator
{
public:
  explicit GlobalRobotAllocator(rclcpp::Logger logger) : logger_(logger) {}

  /**
   * @brief 全体最適化によるロボット割当
   *
   * @param requirements Sessionごとの要求リスト
   * @param available_robots 利用可能なロボットIDリスト
   * @param world_model ワールドモデル
   * @param prev_state 前フレームの割当状態
   * @param config コスト計算設定
   * @return Session名→割り当てられたロボットIDリストのマップ
   */
  auto allocate(
    const std::vector<SessionRequirement> & requirements,
    const std::vector<uint8_t> & available_robots, WorldModelWrapper::SharedPtr & world_model,
    const AllocationState & prev_state, const AllocationCostConfig & config)
    -> std::unordered_map<std::string, std::vector<uint8_t>>;

private:
  rclcpp::Logger logger_;

  /**
   * @brief ハード制約Sessionを先に処理
   */
  auto allocateHardConstraints(
    const std::vector<SessionRequirement> & hard_requirements,
    std::vector<uint8_t> & remaining_robots, WorldModelWrapper::SharedPtr & world_model,
    const AllocationState & prev_state, const AllocationCostConfig & config,
    std::unordered_map<std::string, std::vector<uint8_t>> & result) -> void;

  /**
   * @brief ソフト制約Sessionをハンガリアン法で処理
   */
  auto allocateSoftConstraints(
    const std::vector<SessionRequirement> & soft_requirements,
    const std::vector<uint8_t> & remaining_robots, WorldModelWrapper::SharedPtr & world_model,
    const AllocationState & prev_state, const AllocationCostConfig & config,
    std::unordered_map<std::string, std::vector<uint8_t>> & result) -> void;

  /**
   * @brief 仮想ターゲット（Session-Robot ペア）を生成
   */
  struct VirtualTarget
  {
    std::string session_name;
    int session_priority;
    size_t robot_index;
    std::shared_ptr<std::function<double(const std::shared_ptr<RobotInfo> &)>> suitability_func;
  };

  /**
   * @brief コスト行列を構築
   */
  auto buildCostMatrix(
    const std::vector<uint8_t> & robots, const std::vector<VirtualTarget> & targets,
    WorldModelWrapper::SharedPtr & world_model, const AllocationState & prev_state,
    const AllocationCostConfig & config)
    -> std::function<double(size_t robot_idx, size_t target_idx)>;
};

}  // namespace crane
#endif  // CRANE_SESSION_COORDINATOR__GLOBAL_ROBOT_ALLOCATOR_HPP_
