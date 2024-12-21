// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/emplace_robot_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
EmplaceRobotPlanner::calculateRobotCommand(
  const std::vector<RobotIdentifier> & robots, PlannerContext & context)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;

  for (auto & [id, skill] : m_skill_map) {
    skill->run();
    robot_commands.emplace_back(skill->getRobotCommand());
  }
  return {PlannerBase::Status::RUNNING, robot_commands};
}

/// @brief プランナーでロボットを選択する
/// @param selectable_robots_num  yamlの値
/// @param selectable_robots      選択可能な残りのロボット
/// @param prev_roles
/// @param context
/// @return                        このプランナーで選択されたロボット
auto EmplaceRobotPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
  -> std::vector<uint8_t>
{
  if (selectable_robots_num >= selectable_robots.size()) {
    selectable_robots_num = selectable_robots.size();
  }
  m_skill_map.clear();

  // 退場するロボットの数を計算
  uint8_t allowed_robots_num = static_cast<uint8_t>(world_model->getOurMaxAllowedBots());
  uint8_t select_num = selectable_robots_num - allowed_robots_num;

  std::vector<uint8_t> selected_robots = [&]() {
    // Todo: バッテリー残量が少ないものを優先するとか
    const uint8_t golie_id = world_model->getOurGoalieId();
    std::vector<uint8_t> selected;

    for (const auto & id : selectable_robots) {
      if (select_num <= selected.size()) {
        break;
      }
      if (golie_id != id) {
        selected.emplace_back(id);
      }
    }
    return selected;
  }();
  select_num = selected_robots.size();
  int selected_robots_index = 0;
  for (uint8_t select_index : selected_robots) {
    auto command_base = std::make_shared<RobotCommandWrapperBase>(
      "emplace_planner", selectable_robots[select_index], world_model);
    m_skill_map.try_emplace(
      selectable_robots[select_index], std::make_shared<skills::EmplaceRobot>(command_base));

    m_skill_map[selectable_robots[select_index]]->setParameter("total_robot_number", select_num);
    m_skill_map[selectable_robots[select_index]]->setParameter(
      "current_robot_index", selected_robots_index);

    // 暫定対応 どちら側に退場するか
    m_skill_map[selectable_robots[select_index]]->setParameter("emplace_line_positive", true);
    ++selected_robots_index;
  }
  return selected_robots;
}

}  // namespace crane
