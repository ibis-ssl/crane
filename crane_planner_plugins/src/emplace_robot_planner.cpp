// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/emplace_robot_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
EmplaceRobotPlanner::calculatePositionCommand(const std::vector<RobotIdentifier> &)
{
  std::vector<crane_msgs::msg::PositionCommand> robot_commands;

  for (auto & [id, skill] : m_skill_map) {
    skill->run();
    robot_commands.emplace_back(skill->getRobotCommand());
  }
  return {PlannerBase::Status::RUNNING, robot_commands};
}

/// @brief 出場可能台数分だけフィールドに残す
/// @param selectable_robots_num  yamlの値 テスト用で100が入っていたときはすべて退場させる
/// @param selectable_robots      選択可能な残りのロボット
/// @param prev_roles
/// @param context
/// @return                        このプランナーで選択されたロボット
auto EmplaceRobotPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> &) -> std::vector<uint8_t>
{
  m_skill_map.clear();

  // 退場するロボットの数を計算
  auto allowed_robots_num = static_cast<uint8_t>(world_model->getOurMaxAllowedBots());
  uint8_t exist_robots_num = selectable_robots.size();
  uint8_t select_num = 0;

  // フィールド上のロボットが許可されている台数より多いとき、select
  if (allowed_robots_num < exist_robots_num) {
    select_num = exist_robots_num - allowed_robots_num;
  }
  // yamlが100のときすべて退場!!!!
  if (100 == selectable_robots_num) {
    select_num = exist_robots_num;
  }

  std::vector<uint8_t> selected_robots = [&]() {
    // Todo: バッテリー残量が少ないものを優先するとか
    const uint8_t goalie_id = world_model->getOurGoalieId();
    std::vector<uint8_t> selected;

    for (const auto & id : selectable_robots) {
      if (select_num <= selected.size()) {
        break;
      }
      if (100 == selectable_robots_num || goalie_id != id) {
        selected.emplace_back(id);
      }
    }
    return selected;
  }();
  select_num = selected_robots.size();
  int selected_robots_index = 0;
  for (uint8_t select_index : selected_robots) {
    m_skill_map.try_emplace(
      select_index, std::make_shared<skills::EmplaceRobot>(select_index, world_model));

    m_skill_map[select_index]->setParameter("total_robot_number", select_num);
    m_skill_map[select_index]->setParameter("current_robot_index", selected_robots_index);

    // どちら側に退場するか
    m_skill_map[select_index]->setParameter(
      "emplace_line_positive", world_model->isEmplacePositiveSide());
    ++selected_robots_index;
  }
  return selected_robots;
}

}  // namespace crane
