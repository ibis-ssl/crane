// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__FORWARD_TACTIC_HPP_
#define CRANE_SESSIONS__FORWARD_TACTIC_HPP_

#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/forward.hpp>
#include <crane_sessions/session_base.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class ForwardSession : public SessionBase
{
public:
  COMPOSITION_PUBLIC
  explicit ForwardSession(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : SessionBase("forward", world_model)
  {
  }

  auto createForwardLines() const -> std::vector<Segment>;

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> override;

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    // フォワードライン（敵ゴール前）に近いロボットを優先
    auto wm = world_model;  // shared_ptrをコピー
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      // 敵ゴール位置に近いほど適している
      return robot->getDistance(wm->getTheirGoalCenter());
    };
  }

protected:
  void onRobotsChanged() override { forward_skills.clear(); }

private:
  std::vector<std::shared_ptr<skills::Forward>> forward_skills;
};
}  // namespace crane
#endif  // CRANE_SESSIONS__FORWARD_TACTIC_HPP_
