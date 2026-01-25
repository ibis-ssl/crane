// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_sessions/sub_attacker_skill_session.hpp>

namespace crane
{
auto SubAttackerSkillSession::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
  -> std::pair<SessionBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
{
  // GlobalRobotAllocator対応: robotsが変更されたらスキルを再生成
  if (robots.empty()) {
    return {SessionBase::Status::RUNNING, {}};
  }
  if (not skill) {
    skill = std::make_shared<skills::SubAttacker>(
      "sub_attacker_skill_planner", robots.front().id, world_model);
  }

  auto status = skill->run();
  return {static_cast<SessionBase::Status>(status), {skill->getRobotCommand()}};
}

}  // namespace crane
