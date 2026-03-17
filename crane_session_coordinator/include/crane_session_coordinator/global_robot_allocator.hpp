// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_COORDINATOR__GLOBAL_ROBOT_ALLOCATOR_HPP_
#define CRANE_SESSION_COORDINATOR__GLOBAL_ROBOT_ALLOCATOR_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_physics/allocation_cost.hpp>
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

  // 必要最大ロボット数
  int max_robots;

  // ロボット適性評価関数（ロボット→コストを返す、小さいほど適している）
  std::function<double(const std::shared_ptr<RobotInfo> &)> suitability_func;

  SessionRequirement(
    std::string n, int p, int max_r, std::function<double(const std::shared_ptr<RobotInfo> &)> func)
  : name(std::move(n)), priority(p), max_robots(max_r), suitability_func(std::move(func))
  {
  }
};

/**
 * @brief グリーディ方式ロボット割当クラス
 *
 * 複数のSessionからの要求を優先度順に処理し、グリーディ方式で割当を行う。
 */
class GlobalRobotAllocator
{
public:
  explicit GlobalRobotAllocator(rclcpp::Logger logger) : logger_(logger) {}

  /**
   * @brief グリーディ方式によるロボット割当
   *
   * @param requirements Sessionごとの要求リスト
   * @param available_robots 利用可能なロボットIDリスト
   * @param world_model ワールドモデル
   * @param prev_state 前フレームの割当状態（ヒステリシス用）
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
};

}  // namespace crane
#endif  // CRANE_SESSION_COORDINATOR__GLOBAL_ROBOT_ALLOCATOR_HPP_
