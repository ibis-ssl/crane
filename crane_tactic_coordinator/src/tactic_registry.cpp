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
  // 新しいプランナーを生成
  auto new_planner = generatePlanner(tactic_name, world_model, node);

  // 前回のプランナーリストから同じ設定のプランナーを探す
  auto matched_planner =
    std::ranges::find_if(prev_planners, [&new_planner](const auto & prev_planner) {
      return prev_planner->isSameConfiguration(new_planner.get());
    });

  // 見つかれば再利用、見つからなければ新規プランナーを返す
  TacticBase::SharedPtr result_planner;
  if (matched_planner != prev_planners.end()) {
    result_planner = *matched_planner;
    RCLCPP_INFO(
      rclcpp::get_logger("TacticRegistry"), "Tactic「%s」を再利用 (robots: prev=%zu, new=%zu)",
      tactic_name.c_str(), (*matched_planner)->getRobots().size(), new_planner->getRobots().size());
  } else {
    result_planner = new_planner;
    RCLCPP_INFO(
      rclcpp::get_logger("TacticRegistry"), "Tactic「%s」を新規作成 (robots: new=%zu)",
      tactic_name.c_str(), new_planner->getRobots().size());
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
