// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__GOALIE_FIXED_SESSION_HPP_
#define CRANE_SESSIONS__GOALIE_FIXED_SESSION_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_sessions/goalie_skill_session.hpp>
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

namespace crane
{
/// 固定IDで Goalie スキルを実行する練習用セッション。
/// 「固定割当モード」であることだけを宣言し、実際のIDは YAML の fixed_robots: で指定する。
class GoalieFixedSession : public GoalieSkillSession
{
public:
  COMPOSITION_PUBLIC explicit GoalieFixedSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  : GoalieSkillSession(world_model, node)
  {
    setUseFixedRobots(true);
  }
};

}  // namespace crane
#endif  // CRANE_SESSIONS__GOALIE_FIXED_SESSION_HPP_
