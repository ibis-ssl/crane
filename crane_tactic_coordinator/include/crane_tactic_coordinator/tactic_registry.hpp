// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTIC_COORDINATOR__TACTIC_REGISTRY_HPP_
#define CRANE_TACTIC_COORDINATOR__TACTIC_REGISTRY_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_tactics/tactic_base.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "crane_tactic_coordinator/configuration_manager.hpp"

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
class TacticRegistry
{
public:
  /**
   * @brief プランナーを取得または生成
   *
   * 前回のプランナーリストから同じ設定のプランナーを探し、
   * 見つかれば再利用、見つからなければ新規生成する
   *
   * @param tactic_name プランナー名
   * @param world_model WorldModelへの参照
   * @param node ROSノード
   * @param prev_planners 前回のプランナーリスト
   * @return 取得または生成されたプランナー
   */
  auto getOrCreatePlanner(
    const std::string & tactic_name, WorldModelWrapper::SharedPtr & world_model,
    rclcpp::Node & node, const std::vector<TacticBase::SharedPtr> & prev_planners,
    const std::unordered_map<std::string, SessionParameterType> & params = {})
    -> TacticBase::SharedPtr;

  /**
   * @brief 現在アクティブな全プランナーを取得
   * @return プランナーリスト（読み取り専用）
   */
  auto getAllPlanners() const -> const std::vector<TacticBase::SharedPtr> &;

  /**
   * @brief プランナーを追加
   * @param tactic 追加するプランナー
   */
  auto addPlanner(const TacticBase::SharedPtr & tactic) -> void;

  /**
   * @brief 全プランナーをクリア
   */
  auto clear() -> void;

  /**
   * @brief プランナーリストを置き換え
   * @param tactics 新しいプランナーリスト
   */
  auto setPlanners(const std::vector<TacticBase::SharedPtr> & tactics) -> void;

private:
  // 現在アクティブなプランナーのリスト
  std::vector<TacticBase::SharedPtr> active_tactics_;
};

}  // namespace crane

#endif  // CRANE_TACTIC_COORDINATOR__TACTIC_REGISTRY_HPP_
