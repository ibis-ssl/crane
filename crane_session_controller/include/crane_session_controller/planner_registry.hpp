// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_CONTROLLER__PLANNER_REGISTRY_HPP_
#define CRANE_SESSION_CONTROLLER__PLANNER_REGISTRY_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_planner_plugins/planner_base.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace crane
{
/**
 * @brief プランナーのライフサイクル管理を担当するクラス
 *
 * 責務:
 * - プランナーの生成（generatePlannerを使用）
 * - 前回のプランナーとの比較と再利用判定
 * - アクティブなプランナーのリスト管理
 */
class PlannerRegistry
{
public:
  /**
   * @brief プランナーを取得または生成
   *
   * 前回のプランナーリストから同じ設定のプランナーを探し、
   * 見つかれば再利用、見つからなければ新規生成する
   *
   * @param planner_name プランナー名
   * @param world_model WorldModelへの参照
   * @param node ROSノード
   * @param prev_planners 前回のプランナーリスト
   * @return 取得または生成されたプランナー
   */
  auto getOrCreatePlanner(
    const std::string & planner_name, WorldModelWrapper::SharedPtr & world_model,
    rclcpp::Node & node, const std::vector<PlannerBase::SharedPtr> & prev_planners)
    -> PlannerBase::SharedPtr;

  /**
   * @brief 現在アクティブな全プランナーを取得
   * @return プランナーリスト（読み取り専用）
   */
  auto getAllPlanners() const -> const std::vector<PlannerBase::SharedPtr> &;

  /**
   * @brief プランナーを追加
   * @param planner 追加するプランナー
   */
  auto addPlanner(const PlannerBase::SharedPtr & planner) -> void;

  /**
   * @brief 全プランナーをクリア
   */
  auto clear() -> void;

  /**
   * @brief プランナーリストを置き換え
   * @param planners 新しいプランナーリスト
   */
  auto setPlanners(const std::vector<PlannerBase::SharedPtr> & planners) -> void;

private:
  // 現在アクティブなプランナーのリスト
  std::vector<PlannerBase::SharedPtr> active_planners_;
};

}  // namespace crane

#endif  // CRANE_SESSION_CONTROLLER__PLANNER_REGISTRY_HPP_
