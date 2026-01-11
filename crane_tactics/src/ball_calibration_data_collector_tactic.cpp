// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_tactics/ball_calibration_data_collector_tactic.hpp>

namespace crane
{

BallCalibrationDataCollectorTactic::BallCalibrationDataCollectorTactic(
  WorldModelWrapper::SharedPtr & world_model, [[maybe_unused]] rclcpp::Node &)
: TacticBase("ball_calibration_data_collector", world_model), skill_(nullptr), current_robot_id_()
{
}

std::pair<TacticBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
BallCalibrationDataCollectorTactic::calculatePositionCommand(
  const std::vector<RobotIdentifier> & robots)
{
  if (robots.empty()) {
    return {TacticBase::Status::FAILURE, {}};
  }

  // ロボットIDが変更された場合、新しいスキルインスタンスを作成
  uint8_t robot_id = robots[0].id;
  if (!current_robot_id_ || current_robot_id_.value() != robot_id || !skill_) {
    current_robot_id_ = robot_id;
    skill_ = std::make_shared<skills::BallCalibrationDataCollector>(robot_id, world_model);

    // デフォルトパラメータの設定
    skill_->setParameter("kick_target", Point(0.0, 0.0));  // 原点にキック

    RCLCPP_INFO(
      rclcpp::get_logger("BallCalibrationDataCollectorTactic"),
      "BallCalibrationDataCollectorスキル初期化: robot_id=%d", robot_id);
  }

  // スキルを実行
  auto skill_status = skill_->run();

  std::vector<crane_msgs::msg::PositionCommand> robot_commands;
  robot_commands.emplace_back(skill_->getRobotCommand());

  // スキルステータスをプランナーステータスに変換
  TacticBase::Status tactic_status = TacticBase::Status::RUNNING;
  switch (skill_status) {
    case skills::Status::SUCCESS:
      tactic_status = TacticBase::Status::SUCCESS;
      break;
    case skills::Status::FAILURE:
      tactic_status = TacticBase::Status::FAILURE;
      break;
    case skills::Status::RUNNING:
    default:
      tactic_status = TacticBase::Status::RUNNING;
      break;
  }

  return {tactic_status, robot_commands};
}

}  // namespace crane
