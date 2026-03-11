// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_sessions/ball_near_by_positioner_skill_session.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>

namespace crane
{
auto BallNearByPositionerSkillSession::calculatePositionCommand(
  const std::vector<RobotIdentifier> & robots)
  -> std::pair<SessionBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
{
  // GlobalRobotAllocator対応: robotsが変更されたらスキルを再生成
  if (skills.size() != robots.size()) {
    skills.clear();

    int index = 0;
    for (const auto & robot_id : robots) {
      skills.emplace_back(
        std::make_shared<skills::BallNearByPositioner>(
          "ball_near_by_positioner_skill_planner", robot_id.id, world_model));
      skills.back()->setParameter("total_robot_number", static_cast<int>(robots.size()));
      skills.back()->setParameter("current_robot_index", index++);
      skills.back()->setParameter("line_policy", std::string("arc"));
      skills.back()->setParameter("positioning_policy", getPositioningPolicy());
      skills.back()->setParameter("robot_interval", 0.35);
      skills.back()->setParameter("margin_distance", 0.8);
    }
  }

  auto robot_commands = skills | ranges::views::transform([this](const auto & skill) {
                          setupBeforeRun(skill);
                          skill->run();
                          return skill->getRobotCommand();
                        }) |
                        ranges::to<std::vector<crane_msgs::msg::RobotCommand>>();
  return {SessionBase::Status::RUNNING, robot_commands};
}

}  // namespace crane
