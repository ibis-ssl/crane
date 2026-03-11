// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_session_coordinator/session_registry.hpp"

#include <algorithm>
#include <crane_sessions/session_factory.hpp>
#include <ranges>

namespace crane
{
auto SessionRegistry::getOrCreatePlanner(
  const std::string & session_name, WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node,
  const std::vector<SessionBase::SharedPtr> & prev_planners,
  const std::unordered_map<std::string, SessionParameterType> & params) -> SessionBase::SharedPtr
{
  // 前回のプランナーリストから名前で探す（新規生成を回避）
  auto matched_planner = std::ranges::find_if(
    prev_planners, [&session_name](const auto & planner) { return planner->name == session_name; });

  // 見つかれば再利用、見つからなければ新規生成
  SessionBase::SharedPtr result_planner;
  if (matched_planner != prev_planners.end()) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("SessionRegistry"), "セッション再利用: %s", session_name.c_str());
    result_planner = *matched_planner;
  } else {
    RCLCPP_DEBUG(
      rclcpp::get_logger("SessionRegistry"), "セッション新規生成: %s (prev_planners.size=%zu)",
      session_name.c_str(), prev_planners.size());
    result_planner = generatePlanner(session_name, world_model, node);
  }

  // パラメータを設定（新規・再利用どちらでも）
  if (!params.empty()) {
    result_planner->setSessionParameters(params);
  }

  return result_planner;
}

auto SessionRegistry::getAllPlanners() const -> const std::vector<SessionBase::SharedPtr> &
{
  return active_sessions_;
}

auto SessionRegistry::addPlanner(const SessionBase::SharedPtr & session) -> void
{
  active_sessions_.push_back(session);
}

auto SessionRegistry::clear() -> void { active_sessions_.clear(); }

auto SessionRegistry::setPlanners(const std::vector<SessionBase::SharedPtr> & sessions) -> void
{
  active_sessions_ = sessions;
}

}  // namespace crane
