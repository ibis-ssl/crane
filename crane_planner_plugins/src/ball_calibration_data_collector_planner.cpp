// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/ball_calibration_data_collector_planner.hpp>

namespace crane
{

BallCalibrationDataCollectorPlanner::BallCalibrationDataCollectorPlanner(
  WorldModelWrapper::SharedPtr & world_model, [[maybe_unused]] rclcpp::Node &)
: PlannerBase("BallCalibrationDataCollector", world_model), skill_(nullptr), current_robot_id_()
{
}

std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
BallCalibrationDataCollectorPlanner::calculatePositionCommand(
  const std::vector<RobotIdentifier> & robots)
{
  if (robots.empty()) {
    return {PlannerBase::Status::FAILURE, {}};
  }

  // ロボットIDが変更された場合、新しいスキルインスタンスを作成
  uint8_t robot_id = robots[0].id;
  if (!current_robot_id_ || current_robot_id_.value() != robot_id || !skill_) {
    current_robot_id_ = robot_id;
    skill_ = std::make_shared<skills::BallCalibrationDataCollector>(robot_id, world_model);

    // デフォルトパラメータの設定
    skill_->setParameter("kick_target", Point(0.0, 0.0));  // 原点にキック

    RCLCPP_INFO(
      rclcpp::get_logger("BallCalibrationDataCollectorPlanner"),
      "BallCalibrationDataCollectorスキル初期化: robot_id=%d", robot_id);
  }

  // スキルを実行
  auto skill_status = skill_->run();

  std::vector<crane_msgs::msg::PositionCommand> robot_commands;
  robot_commands.emplace_back(skill_->getRobotCommand());

  // スキルステータスをプランナーステータスに変換
  PlannerBase::Status planner_status = PlannerBase::Status::RUNNING;
  switch (skill_status) {
    case skills::Status::SUCCESS:
      planner_status = PlannerBase::Status::SUCCESS;
      break;
    case skills::Status::FAILURE:
      planner_status = PlannerBase::Status::FAILURE;
      break;
    case skills::Status::RUNNING:
    default:
      planner_status = PlannerBase::Status::RUNNING;
      break;
  }

  return {planner_status, robot_commands};
}

auto BallCalibrationDataCollectorPlanner::getSelectedRobots(
  [[maybe_unused]] uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  // 1台のロボットのみ選択
  auto selected = this->getSelectedRobotsByScore(
    1,  // 1台のロボットのみ必要
    selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      // 最もボールに近いロボットを選択
      return 15. - robot->id;
    },
    prev_roles);

  return selected;
}

}  // namespace crane
