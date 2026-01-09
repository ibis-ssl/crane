// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_tactics/emplace_robot_tactic.hpp>

namespace crane
{
std::pair<TacticBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
EmplaceRobotTactic::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
{
  // GlobalRobotAllocator対応: robotsが変更されたらスキルマップを再生成
  if (m_skill_map.size() != robots.size()) {
    m_skill_map.clear();

    int select_num = robots.size();
    int selected_robots_index = 0;
    for (const auto & robot_id : robots) {
      m_skill_map.try_emplace(
        robot_id.id, std::make_shared<skills::EmplaceRobot>(robot_id.id, world_model));

      m_skill_map[robot_id.id]->setParameter("total_robot_number", select_num);
      m_skill_map[robot_id.id]->setParameter("current_robot_index", selected_robots_index);

      // どちら側に退場するか
      m_skill_map[robot_id.id]->setParameter(
        "emplace_line_positive", world_model->isEmplacePositiveSide());
      ++selected_robots_index;
    }
  }

  std::vector<crane_msgs::msg::PositionCommand> robot_commands;

  for (auto & [id, skill] : m_skill_map) {
    skill->run();
    robot_commands.emplace_back(skill->getRobotCommand());
  }
  return {TacticBase::Status::RUNNING, robot_commands};
}

}  // namespace crane
