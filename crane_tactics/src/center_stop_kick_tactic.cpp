// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_tactics/center_stop_kick_tactic.hpp>

namespace crane
{

CenterStopKickTactic::CenterStopKickTactic(
  WorldModelWrapper::SharedPtr & world_model, [[maybe_unused]] rclcpp::Node &)
: TacticBase("CenterStopKick", world_model), skill_(nullptr), current_robot_id_()
{
}

std::pair<TacticBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
CenterStopKickTactic::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
{
  if (robots.empty()) {
    return {TacticBase::Status::FAILURE, {}};
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
      rclcpp::get_logger("CenterStopKickTactic"), "CenterStopKickスキル初期化: robot_id=%d",
      robot_id);
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
      RCLCPP_INFO(rclcpp::get_logger("CenterStopKickTactic"), "フィールド中心停止キック完了");
      break;
    case skills::Status::FAILURE:
      tactic_status = TacticBase::Status::FAILURE;
      RCLCPP_WARN(rclcpp::get_logger("CenterStopKickTactic"), "フィールド中心停止キック失敗");
      break;
    case skills::Status::RUNNING:
    default:
      tactic_status = TacticBase::Status::RUNNING;
      break;
  }

  return {tactic_status, robot_commands};
}

auto CenterStopKickTactic::getSelectedRobots(
  [[maybe_unused]] uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  auto selected = this->getSelectedRobotsByScore(
    1,  // 1台のロボットのみ必要
    selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) { return 10.0 - robot->id; }, prev_roles);

  if (!selected.empty()) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("CenterStopKickTactic"), "選択されたロボット: ID=%d, ボール距離=%.3fm",
      selected[0], world_model->getOurRobot(selected[0])->getDistance(world_model->ball().pos));
  }

  return selected;
}

}  // namespace crane
