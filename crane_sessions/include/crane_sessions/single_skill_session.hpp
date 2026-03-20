// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__SINGLE_SKILL_SESSION_HPP_
#define CRANE_SESSIONS__SINGLE_SKILL_SESSION_HPP_

#include <crane_sessions/session_base.hpp>
#include <memory>
#include <vector>

namespace crane
{

/// 単一スキルラッパーセッションの共通基底クラス
/// calculatePositionCommand と onRobotsChanged の定型的な実装を提供する。
/// 派生クラスは createSkill() を実装し、getRobotSuitabilityFunc() をオーバーライドすること。
template <typename SkillType>
class SingleSkillSession : public SessionBase
{
public:
  std::shared_ptr<SkillType> skill = nullptr;

  using SessionBase::SessionBase;

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> override
  {
    if (robots.empty()) {
      return {SessionBase::Status::RUNNING, {}};
    }
    if (not skill) {
      skill = createSkill(robots.front().id);
      visualizer->layer = "skill/" + skill->name;
    }
    auto status = skill->run();
    return {static_cast<SessionBase::Status>(status), {skill->getRobotCommand()}};
  }

protected:
  void onRobotsChanged() override { skill.reset(); }

  /// スキルインスタンスを生成するファクトリメソッド（派生クラスで実装）
  virtual auto createSkill(uint8_t robot_id) -> std::shared_ptr<SkillType> = 0;
};

}  // namespace crane
#endif  // CRANE_SESSIONS__SINGLE_SKILL_SESSION_HPP_
