// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_tactic_coordinator/diagnostics_reporter.hpp"

namespace crane
{

DiagnosticsReporter::DiagnosticsReporter(
  rclcpp::Clock::SharedPtr clock, std::shared_ptr<TacticRegistry> tactic_registry,
  rclcpp::Logger logger)
: clock_(clock),
  tactic_registry_(tactic_registry),
  logger_(logger),
  last_planning_time_(clock_->now())
{
}

auto DiagnosticsReporter::recordCycle() -> void
{
  planning_count_++;
  last_planning_time_ = clock_->now();
}

auto DiagnosticsReporter::updateDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat, bool world_model_ready,
  const WorldModelWrapper::SharedPtr & world_model) -> void
{
  // WorldModelが準備できているかチェック
  if (!world_model_ready) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for world model to be ready");
    return;
  }

  // プランニングが実行されているかチェック
  auto time_since_last_planning = (clock_->now() - last_planning_time_).seconds();

  if (time_since_last_planning > 1.0) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Planning not running (no update for >1s)");
  } else if (time_since_last_planning > 0.5) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Planning update slow (>0.5s since last update)");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Planning is running normally");
  }

  stat.add("time_since_last_planning", time_since_last_planning);
  stat.add("planning_count", planning_count_);
  stat.add("active_planners", static_cast<int>(tactic_registry_->getAllPlanners().size()));
  stat.add("available_robots", static_cast<int>(world_model->ours().getAvailableRobotIds().size()));
}

}  // namespace crane
