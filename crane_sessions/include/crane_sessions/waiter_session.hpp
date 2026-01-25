// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__WAITER_SESSION_HPP_
#define CRANE_SESSIONS__WAITER_SESSION_HPP_

#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_sessions/session_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class WaiterSession : public SessionBase
{
public:
  COMPOSITION_PUBLIC
  explicit WaiterSession(
    WorldModelWrapper::SharedPtr & world_model, [[maybe_unused]] rclcpp::Node &)
  : SessionBase("waiter", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    // ID番号の小さいロボットを優先（待機ロボットとして低IDを使用）
    return [](const std::shared_ptr<RobotInfo> & robot) { return static_cast<double>(robot->id); };
  }

private:
  std::unordered_map<uint8_t, Pose2D> stop_poses;
};
}  // namespace crane
#endif  // CRANE_SESSIONS__WAITER_SESSION_HPP_
