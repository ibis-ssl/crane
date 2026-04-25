// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__SUB_ATTACKER_FIXED_SESSION_HPP_
#define CRANE_SESSIONS__SUB_ATTACKER_FIXED_SESSION_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_sessions/sub_attacker_skill_session.hpp>
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

namespace crane
{
/// 固定IDで SubAttacker スキルを実行する練習用セッション。
/// 「固定割当モード」であることだけを宣言し、実際のIDは YAML の fixed_robots: で指定する。
class SubAttackerFixedSession : public SubAttackerSkillSession
{
public:
  COMPOSITION_PUBLIC explicit SubAttackerFixedSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  : SubAttackerSkillSession(world_model, node)
  {
    setUseFixedRobots(true);
  }
};

}  // namespace crane
#endif  // CRANE_SESSIONS__SUB_ATTACKER_FIXED_SESSION_HPP_
