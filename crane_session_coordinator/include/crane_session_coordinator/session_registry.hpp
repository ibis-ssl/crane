// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_COORDINATOR__SESSION_REGISTRY_HPP_
#define CRANE_SESSION_COORDINATOR__SESSION_REGISTRY_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_sessions/session_base.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "crane_session_coordinator/configuration_manager.hpp"

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
class SessionRegistry
{
public:
  /// 前回のプランナーリストから同じ名前のプランナーを探し、
  /// 見つかれば再利用、見つからなければ新規生成する
  auto getOrCreatePlanner(
    const std::string & tactic_name, WorldModelWrapper::SharedPtr & world_model,
    rclcpp::Node & node, const std::vector<SessionBase::SharedPtr> & prev_planners,
    const std::unordered_map<std::string, SessionParameterType> & params = {})
    -> SessionBase::SharedPtr;

  auto getAllPlanners() const -> const std::vector<SessionBase::SharedPtr> &
  {
    return active_sessions_;
  }

  auto addPlanner(const SessionBase::SharedPtr & session) -> void
  {
    active_sessions_.push_back(session);
  }

  auto clear() -> void { active_sessions_.clear(); }

private:
  std::vector<SessionBase::SharedPtr> active_sessions_;
};

}  // namespace crane

#endif  // CRANE_SESSION_COORDINATOR__SESSION_REGISTRY_HPP_
