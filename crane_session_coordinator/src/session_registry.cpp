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
  const std::string & tactic_name, WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node,
  const std::vector<SessionBase::SharedPtr> & prev_planners,
  const std::unordered_map<std::string, SessionParameterType> & params) -> SessionBase::SharedPtr
{
  // 前回のプランナーリストから名前で探す（新規生成を回避）
  auto matched_planner = std::ranges::find_if(
    prev_planners, [&tactic_name](const auto & planner) { return planner->name == tactic_name; });

  // 見つかれば再利用、見つからなければ新規生成
  SessionBase::SharedPtr result_planner;
  if (matched_planner != prev_planners.end()) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("SessionRegistry"), "タクティクス再利用: %s", tactic_name.c_str());
    result_planner = *matched_planner;
  } else {
    RCLCPP_DEBUG(
      rclcpp::get_logger("SessionRegistry"), "タクティクス新規生成: %s (prev_planners.size=%zu)",
      tactic_name.c_str(), prev_planners.size());
    result_planner = generatePlanner(tactic_name, world_model, node);
  }

  // パラメータを設定（新規・再利用どちらでも）
  if (!params.empty()) {
    // SessionParameterType から SessionParameterType への変換
    std::unordered_map<std::string, SessionParameterType> tactic_params;
    for (const auto & [key, value] : params) {
      std::visit([&tactic_params, &key](const auto & v) { tactic_params[key] = v; }, value);
    }
    result_planner->setTacticParameters(tactic_params);
  }

  return result_planner;
}

auto SessionRegistry::getAllPlanners() const -> const std::vector<SessionBase::SharedPtr> &
{
  return active_tactics_;
}

auto SessionRegistry::addPlanner(const SessionBase::SharedPtr & tactic) -> void
{
  active_tactics_.push_back(tactic);
}

auto SessionRegistry::clear() -> void { active_tactics_.clear(); }

auto SessionRegistry::setPlanners(const std::vector<SessionBase::SharedPtr> & tactics) -> void
{
  active_tactics_ = tactics;
}

}  // namespace crane
