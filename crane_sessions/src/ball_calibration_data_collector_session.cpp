// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_sessions/ball_calibration_data_collector_session.hpp>

namespace crane
{

BallCalibrationDataCollectorSession::BallCalibrationDataCollectorSession(
  WorldModelWrapper::SharedPtr & world_model, [[maybe_unused]] rclcpp::Node &)
: SessionBase("ball_calibration_data_collector", world_model), skill_(nullptr), current_robot_id_()
{
}

std::pair<SessionBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
BallCalibrationDataCollectorSession::calculatePositionCommand(
  const std::vector<RobotIdentifier> & robots)
{
  if (robots.empty()) {
    return {SessionBase::Status::FAILURE, {}};
  }

  // ロボットIDが変更された場合、新しいスキルインスタンスを作成
  uint8_t robot_id = robots[0].id;
  if (!current_robot_id_ || current_robot_id_.value() != robot_id || !skill_) {
    current_robot_id_ = robot_id;
    skill_ = std::make_shared<skills::BallCalibrationDataCollector>(robot_id, world_model);

    // デフォルトパラメータの設定
    skill_->setParameter("kick_target", Point(0.0, 0.0));  // 原点にキック

    RCLCPP_INFO(
      rclcpp::get_logger("BallCalibrationDataCollectorSession"),
      "BallCalibrationDataCollectorスキル初期化: robot_id=%d", robot_id);
  }

  // スキルを実行
  auto skill_status = skill_->run();

  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  robot_commands.emplace_back(skill_->getRobotCommand());

  // スキルステータスをプランナーステータスに変換
  SessionBase::Status tactic_status = SessionBase::Status::RUNNING;
  switch (skill_status) {
    case skills::Status::SUCCESS:
      tactic_status = SessionBase::Status::SUCCESS;
      break;
    case skills::Status::FAILURE:
      tactic_status = SessionBase::Status::FAILURE;
      break;
    case skills::Status::RUNNING:
    default:
      tactic_status = SessionBase::Status::RUNNING;
      break;
  }

  return {tactic_status, robot_commands};
}

}  // namespace crane
