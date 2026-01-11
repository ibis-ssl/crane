// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_tactics/emplace_error_robot_tactic.hpp>
#include <crane_robot_skills/emplace_error_robot.hpp>

namespace
{
  /**
   * @brief エラー状態のロボットIDを取得
   * 
   * @param world_model_wrapper 
   * @return std::vector<uint8_t> 
   */
  std::vector<uint8_t> getErrorRobotIds(const crane::WorldModelWrapper::SharedPtr world_model_wrapper)
  {
    std::vector<uint8_t> error_robot_ids;
    const auto & world_model = world_model_wrapper->getMsg();
    for (const auto & robot_status  : world_model.robot_info_ours) {
      if (robot_status.has_error) {
        error_robot_ids.emplace_back(robot_status.id);
      }
    }
    return error_robot_ids;
  }

  /**
   * @brief ロボットからタッチラインへの最短線分を取得
   * 
   * @param world_model_wrapper 
   * @param is_positive_side 
   * @param target_robot_id 1
   * @return crane::Segment 
   */
  crane::Segment getClosestLineToTouchLine(
    const crane::WorldModelWrapper::SharedPtr world_model_wrapper,
    bool is_positive_side,
    const uint8_t& target_robot_id)
  {
    // タッチライン  
    const double X = world_model_wrapper->fieldSize().x() / 2.0;
    const double Y = world_model_wrapper->fieldSize().y() / 2.0;
    crane::Segment touchline;
    touchline = is_positive_side ? 
        crane::Segment(crane::Point(X, Y), crane::Point(-X, Y))
      : crane::Segment(crane::Point(X, -Y), crane::Point(-X, -Y));

    const auto & target_robot_pose = world_model_wrapper->getOurRobot(target_robot_id)->pose.pos;

    auto [distance, point_to_touchline] = crane::getClosestPointAndDistance(target_robot_pose, touchline);
    crane::Segment closest_line_to_touchline = crane::Segment(target_robot_pose, point_to_touchline);
    return closest_line_to_touchline;
  }

  /**
   * @brief 一番タッチラインに近いロボットを返す
   * 
   * @param world_model_wrapper 
   * @param candidate_robot_ids 
   * @return std::optional<uint8_t>
   */
  std::optional<uint8_t> getClosestRobotToTouchLineWithoutConflict(
    const crane::WorldModelWrapper::SharedPtr world_model_wrapper,
    const std::vector<uint8_t>& candidate_robot_ids,
    bool is_positive_side)
  {
    // 敵ロボットがタッチライン上にいるとみなす距離閾値
    const double THRESHOLD = 0.2;
  
    uint8_t closest_robot_id = 255; 
    double min_distance = std::numeric_limits<double>::max();
    crane::Segment closest_line_to_touchline;
    
    // 一番タッチラインに近いロボットを探す
    for (const uint8_t robot_id : candidate_robot_ids) {
      crane::Segment candidate_robot_to_touchline = getClosestLineToTouchLine(world_model_wrapper, is_positive_side, robot_id);
      double distance = crane::bg::length(candidate_robot_to_touchline);
      
      // タッチラインとの方向に敵ロボットがいるか確認
      // 仲間ロボットは無視
      bool is_interfering = false;
      for (const auto& enemy : world_model_wrapper->theirs().getAvailableRobots()) {  
        double distance = crane::bg::distance(enemy->pose.pos, candidate_robot_to_touchline); 
        if (distance < THRESHOLD) {  
          is_interfering = true;
          break;
        }  
      }
      if(is_interfering) {
        continue; // 敵ロボットがタッチライン上にいる場合はスキップ
      }

      if (distance < min_distance) {  
        min_distance = distance;  
        closest_line_to_touchline = candidate_robot_to_touchline;
        closest_robot_id = robot_id;
      }  
    }

    if (closest_robot_id == 255) {
      // タッチラインに近いロボットが見つからなかった場合
      return std::nullopt; 
    }
    else {
      return closest_robot_id;
    }
  }
} // anonymous namespace
namespace crane
{
std::pair<TacticBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
EmplaceErrorRobotTactic::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
{
  if (m_skill_map.size() != robots.size()) {
    m_skill_map.clear();
    for (const auto & robot_id : robots) {
      m_skill_map.try_emplace(
        robot_id.id, std::make_shared<skills::EmplaceErrorRobot>(robot_id.id, world_model));
      m_skill_map[robot_id.id]->setParameter("current_robot_index", robot_id.id);
    }
  }
  std::vector<crane_msgs::msg::PositionCommand> robot_commands;

  for (auto & [id, skill] : m_skill_map) {
    skill->run();
    robot_commands.emplace_back(skill->getRobotCommand());
  }
  return {TacticBase::Status::RUNNING, robot_commands};
}

auto EmplaceErrorRobotTactic::getRobotSuitabilityFunc() const
  -> std::function<double(const std::shared_ptr<RobotInfo> &)>
{
  auto wm = world_model;
  return [wm](const std::shared_ptr<RobotInfo> & robot) {
    // 1. Goalieを除外
    if (robot->id == wm->getOurGoalieId()) {
      return 100.0;
    }

    // 2. エラーロボットを取得
    std::vector<uint8_t> error_robot_ids = getErrorRobotIds(wm);
    if (error_robot_ids.empty()) {
      return 1000.0;  // エラーロボットがいなければ全員1000
    }

    // 3. タッチラインに最も近いエラーロボットを見つける
    auto closest_error_robot = getClosestRobotToTouchLineWithoutConflict(
      wm, error_robot_ids, wm->isEmplacePositiveSide());
    if (!closest_error_robot.has_value()) {
      return 1000.0;  // 見つからなければ1000
    }

    // 4. そのエラーロボットからの距離で評価
    const auto & error_robot_pos = wm->getOurRobot(closest_error_robot.value())->pose.pos;
    return robot->getDistance(error_robot_pos);
  };
}

}  // namespace crane
