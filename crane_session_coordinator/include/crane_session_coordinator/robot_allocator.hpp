// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_COORDINATOR__ROBOT_ALLOCATOR_HPP_
#define CRANE_SESSION_COORDINATOR__ROBOT_ALLOCATOR_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_select_results.hpp>
#include <crane_physics/allocation_cost.hpp>
#include <crane_physics/robot_info.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "allocation_state.hpp"
#include "configuration_manager.hpp"
#include "session_registry.hpp"

namespace crane
{

/**
 * @brief Sessionのロボット要求情報
 */
struct SessionRequirement
{
  std::string name;
  int priority;
  int max_robots;
  std::function<double(const std::shared_ptr<RobotInfo> &)> suitability_func;
  /// 固定割当ID。空でないとき suitability を無視してこのIDを優先取得する。
  std::vector<uint8_t> fixed_robots;

  SessionRequirement(
    std::string n, int p, int max_r,
    std::function<double(const std::shared_ptr<RobotInfo> &)> func,
    std::vector<uint8_t> fixed = {})
  : name(std::move(n)),
    priority(p),
    max_robots(max_r),
    suitability_func(std::move(func)),
    fixed_robots(std::move(fixed))
  {
  }
};

/**
 * @brief ロボットをプランナーに割り当てる管理クラス
 */
class RobotAllocator
{
public:
  explicit RobotAllocator(
    std::shared_ptr<ConfigurationManager> config_manager,
    std::shared_ptr<SessionRegistry> session_registry, rclcpp::Logger logger);

  /**
   * @brief セッション名と利用可能ロボットから割当を実行
   * @param session_name セッション名
   * @param selectable_robot_ids 選択可能なロボットID
   * @param world_model WorldModelの参照
   * @param node ROSノードの参照
   * @return ロボット選択結果
   */
  auto allocate(
    const std::string & session_name, std::vector<uint8_t> selectable_robot_ids,
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
    -> crane_msgs::msg::RobotSelectResults;

  /**
   * @brief ロボット変動を検出
   * @param observed_robot_ids 観測されたロボットID
   * @return ロボット変動があればtrue
   */
  auto detectRobotChange(const std::vector<uint8_t> & observed_robot_ids) const -> bool;

  /**
   * @brief 現在割り当て済みのロボットIDを取得
   */
  auto getAssignedRobotIds() const -> std::vector<uint8_t>;

  /**
   * @brief 割当状況のログ文字列を生成
   */
  auto buildAssignmentLog() const -> std::string;

  /**
   * @brief 前回と変わっていればログ出力
   */
  auto logAssignmentIfChanged(const std::string & current_assignment) -> void;

  /**
   * @brief コスト設定を取得/設定
   */
  void setAllocationCostConfig(const AllocationCostConfig & config)
  {
    allocation_cost_config_ = config;
  }
  const AllocationCostConfig & getAllocationCostConfig() const { return allocation_cost_config_; }

private:
  auto allocateRobotsGreedy(
    const std::vector<SessionRequirement> & requirements,
    const std::vector<uint8_t> & available_robots, WorldModelWrapper::SharedPtr & world_model,
    const AllocationState & prev_state, const AllocationCostConfig & config)
    -> std::unordered_map<std::string, std::vector<uint8_t>>;

  std::shared_ptr<ConfigurationManager> config_manager_;
  std::shared_ptr<SessionRegistry> session_registry_;
  rclcpp::Logger logger_;

  std::unordered_map<uint8_t, RobotRole> prev_robot_roles_;
  std::string prev_assignment_log_;

  AllocationState allocation_state_;
  AllocationCostConfig allocation_cost_config_;
};

}  // namespace crane

#endif  // CRANE_SESSION_COORDINATOR__ROBOT_ALLOCATOR_HPP_
