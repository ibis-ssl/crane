// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTIC_COORDINATOR__DIAGNOSTICS_REPORTER_HPP_
#define CRANE_TACTIC_COORDINATOR__DIAGNOSTICS_REPORTER_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "tactic_registry.hpp"

namespace crane
{

/**
 * @brief セッションコントローラーの診断情報を管理・報告するクラス
 */
class DiagnosticsReporter
{
public:
  explicit DiagnosticsReporter(
    rclcpp::Clock::SharedPtr clock, std::shared_ptr<TacticRegistry> tactic_registry,
    rclcpp::Logger logger);

  /**
   * @brief プランニングサイクルを記録
   */
  auto recordCycle() -> void;

  /**
   * @brief 診断情報を更新（diagnostic_updater用コールバック）
   */
  auto updateDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper & stat, bool world_model_ready,
    const WorldModelWrapper::SharedPtr & world_model) -> void;

private:
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<TacticRegistry> tactic_registry_;
  rclcpp::Logger logger_;

  rclcpp::Time last_planning_time_;
  int planning_count_ = 0;
};

}  // namespace crane

#endif  // CRANE_TACTIC_COORDINATOR__DIAGNOSTICS_REPORTER_HPP_
