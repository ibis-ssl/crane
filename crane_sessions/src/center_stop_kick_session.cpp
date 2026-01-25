// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_sessions/center_stop_kick_session.hpp>

namespace crane
{

CenterStopKickSession::CenterStopKickSession(
  WorldModelWrapper::SharedPtr & world_model, [[maybe_unused]] rclcpp::Node &)
: SessionBase("center_stop_kick", world_model), skill_(nullptr), current_robot_id_()
{
}

std::pair<SessionBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
CenterStopKickSession::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
{
  if (robots.empty()) {
    return {SessionBase::Status::FAILURE, {}};
  }

  // ロボットIDが変更された場合、新しいスキルインスタンスを作成
  uint8_t robot_id = robots[0].id;
  if (!current_robot_id_ || current_robot_id_.value() != robot_id || !skill_) {
    current_robot_id_ = robot_id;
    skill_ = std::make_shared<skills::CenterStopKick>(robot_id, world_model);

    // デフォルトパラメータの設定
    skill_->setParameter("target_position", Point(0.0, 0.0));  // フィールド中心にキック
    skill_->setParameter("kick_power_tolerance", 0.01);        // キック力計算精度
    skill_->setParameter("stop_distance_tolerance", 0.05);     // 停止距離許容誤差5cm

    RCLCPP_INFO(
      rclcpp::get_logger("CenterStopKickSession"), "CenterStopKickスキル初期化: robot_id=%d",
      robot_id);
  }

  // スキルを実行
  auto skill_status = skill_->run();

  std::vector<crane_msgs::msg::PositionCommand> robot_commands;
  robot_commands.emplace_back(skill_->getRobotCommand());

  // スキルステータスをプランナーステータスに変換
  SessionBase::Status tactic_status = SessionBase::Status::RUNNING;
  switch (skill_status) {
    case skills::Status::SUCCESS:
      tactic_status = SessionBase::Status::SUCCESS;
      RCLCPP_INFO(rclcpp::get_logger("CenterStopKickSession"), "フィールド中心停止キック完了");
      break;
    case skills::Status::FAILURE:
      tactic_status = SessionBase::Status::FAILURE;
      RCLCPP_WARN(rclcpp::get_logger("CenterStopKickSession"), "フィールド中心停止キック失敗");
      break;
    case skills::Status::RUNNING:
    default:
      tactic_status = SessionBase::Status::RUNNING;
      break;
  }

  return {tactic_status, robot_commands};
}

}  // namespace crane
