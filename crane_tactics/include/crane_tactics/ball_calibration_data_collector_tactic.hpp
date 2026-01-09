// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__BALL_CALIBRATION_DATA_COLLECTOR_TACTIC_HPP_
#define CRANE_TACTICS__BALL_CALIBRATION_DATA_COLLECTOR_TACTIC_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/ball_calibration_data_collector.hpp>
#include <crane_tactics/tactic_base.hpp>
#include <memory>
#include <unordered_map>
#include <vector>

#include "visibility_control.h"

namespace crane
{

/**
 * @brief ボールキャリブレーションデータ収集用プランナー（スキルランチャー）
 *
 * BallCalibrationDataCollectorスキルを実行するためのシンプルなプランナー。
 * 実際のロジックはスキル側で実装されている。
 */
class BallCalibrationDataCollectorTactic : public TacticBase
{
public:
  COMPOSITION_PUBLIC explicit BallCalibrationDataCollectorTactic(
    WorldModelWrapper::SharedPtr & world_model, [[maybe_unused]] rclcpp::Node &);

  std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override;

private:
  // スキルインスタンス
  std::shared_ptr<skills::BallCalibrationDataCollector> skill_;

  // 最後に選択されたロボットID
  std::optional<uint8_t> current_robot_id_;
};

}  // namespace crane

#endif  // CRANE_TACTICS__BALL_CALIBRATION_DATA_COLLECTOR_TACTIC_HPP_
