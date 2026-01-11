// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_tactic_coordinator/tactic_registry.hpp"

#include <algorithm>
#include <crane_tactics/tactic_factory.hpp>
#include <ranges>

namespace crane
{
auto TacticRegistry::getOrCreatePlanner(
  const std::string & tactic_name, WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node,
  const std::vector<TacticBase::SharedPtr> & prev_planners,
  const std::unordered_map<std::string, SessionParameterType> & params) -> TacticBase::SharedPtr
{
  // 前回のプランナーリストから名前で探す（新規生成を回避）
  auto matched_planner = std::ranges::find_if(
    prev_planners, [&tactic_name](const auto & planner) { return planner->name == tactic_name; });

  // 見つかれば再利用、見つからなければ新規生成
  TacticBase::SharedPtr result_planner;
  if (matched_planner != prev_planners.end()) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("TacticRegistry"), "タクティクス再利用: %s", tactic_name.c_str());
    result_planner = *matched_planner;
  } else {
    RCLCPP_DEBUG(
      rclcpp::get_logger("TacticRegistry"), "タクティクス新規生成: %s (prev_planners.size=%zu)",
      tactic_name.c_str(), prev_planners.size());
    result_planner = generatePlanner(tactic_name, world_model, node);
  }

  // パラメータを設定（新規・再利用どちらでも）
  if (!params.empty()) {
    // SessionParameterType から TacticParameterType への変換
    std::unordered_map<std::string, TacticParameterType> tactic_params;
    for (const auto & [key, value] : params) {
      std::visit([&tactic_params, &key](const auto & v) { tactic_params[key] = v; }, value);
    }
    result_planner->setTacticParameters(tactic_params);
  }

  return result_planner;
}

auto TacticRegistry::getAllPlanners() const -> const std::vector<TacticBase::SharedPtr> &
{
  return active_tactics_;
}

auto TacticRegistry::addPlanner(const TacticBase::SharedPtr & tactic) -> void
{
  active_tactics_.push_back(tactic);
}

auto TacticRegistry::clear() -> void { active_tactics_.clear(); }

auto TacticRegistry::setPlanners(const std::vector<TacticBase::SharedPtr> & tactics) -> void
{
  active_tactics_ = tactics;
}

}  // namespace crane
