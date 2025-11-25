// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_session_controller/planner_registry.hpp"

#include <algorithm>
#include <crane_planner_plugins/planner_factory.hpp>
#include <ranges>

namespace crane
{
auto PlannerRegistry::getOrCreatePlanner(
  const std::string & planner_name, WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node,
  const std::vector<PlannerBase::SharedPtr> & prev_planners) -> PlannerBase::SharedPtr
{
  // 新しいプランナーを生成
  auto new_planner = generatePlanner(planner_name, world_model, node);

  // 前回のプランナーリストから同じ設定のプランナーを探す
  auto matched_planner =
    std::ranges::find_if(prev_planners, [&new_planner](const auto & prev_planner) {
      return prev_planner->isSameConfiguration(new_planner.get());
    });

  // 見つかれば再利用、見つからなければ新規プランナーを返す
  if (matched_planner != prev_planners.end()) {
    return *matched_planner;
  } else {
    return new_planner;
  }
}

auto PlannerRegistry::getAllPlanners() const -> const std::vector<PlannerBase::SharedPtr> &
{
  return active_planners_;
}

auto PlannerRegistry::addPlanner(const PlannerBase::SharedPtr & planner) -> void
{
  active_planners_.push_back(planner);
}

auto PlannerRegistry::clear() -> void { active_planners_.clear(); }

auto PlannerRegistry::setPlanners(const std::vector<PlannerBase::SharedPtr> & planners) -> void
{
  active_planners_ = planners;
}

}  // namespace crane
