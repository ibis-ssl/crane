// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__DEFENDER_FIXED_SESSION_HPP_
#define CRANE_SESSIONS__DEFENDER_FIXED_SESSION_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_sessions/defender_session.hpp>
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

namespace crane
{
/// 固定IDで防御陣形を組む練習用セッション。
/// 「固定割当モード」であることだけを宣言し、実際のIDは YAML の fixed_robots: で指定する。
class DefenderFixedSession : public DefenderSession
{
public:
  COMPOSITION_PUBLIC explicit DefenderFixedSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  : DefenderSession(world_model, node)
  {
    setUseFixedRobots(true);
  }
};

}  // namespace crane
#endif  // CRANE_SESSIONS__DEFENDER_FIXED_SESSION_HPP_
