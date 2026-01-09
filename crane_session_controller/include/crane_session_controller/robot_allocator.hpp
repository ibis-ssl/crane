// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_CONTROLLER__ROBOT_ALLOCATOR_HPP_
#define CRANE_SESSION_CONTROLLER__ROBOT_ALLOCATOR_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_select_results.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "configuration_manager.hpp"
#include "planner_registry.hpp"

namespace crane
{

/**
 * @brief ロボットをプランナーに割り当てる管理クラス
 */
class RobotAllocator
{
public:
  explicit RobotAllocator(
    std::shared_ptr<ConfigurationManager> config_manager,
    std::shared_ptr<PlannerRegistry> planner_registry, rclcpp::Logger logger);

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

private:
  /**
   * @brief プランナーへのロボット割り当てを試行（エラーハンドリング含む）
   */
  auto tryAssignRobotToPlanner(
    const SessionCapacity & session_capacity, std::vector<uint8_t> & selectable_robot_ids,
    const std::vector<PlannerBase::SharedPtr> & prev_available_planners,
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node,
    crane_msgs::msg::RobotSelectResults & results) -> bool;

  std::shared_ptr<ConfigurationManager> config_manager_;
  std::shared_ptr<PlannerRegistry> planner_registry_;
  rclcpp::Logger logger_;

  std::unordered_map<uint8_t, RobotRole> prev_robot_roles_;
  std::string prev_assignment_log_;
};

}  // namespace crane

#endif  // CRANE_SESSION_CONTROLLER__ROBOT_ALLOCATOR_HPP_
