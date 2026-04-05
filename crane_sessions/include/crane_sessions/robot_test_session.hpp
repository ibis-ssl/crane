// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__ROBOT_TEST_SESSION_HPP_
#define CRANE_SESSIONS__ROBOT_TEST_SESSION_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_command.hpp>
#include <crane_sessions/session_base.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class RobotTestSession : public SessionBase
{
public:
  COMPOSITION_PUBLIC explicit RobotTestSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node);

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    uint8_t target_id = target_robot_id_;
    return [target_id](const std::shared_ptr<RobotInfo> & robot) {
      return (robot->id == target_id) ? 0.0 : 1000.0;
    };
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override;

private:
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface_;
  rclcpp::Subscription<crane_msgs::msg::RobotCommand>::SharedPtr target_sub_;

  mutable std::mutex target_mutex_;
  crane_msgs::msg::RobotCommand::SharedPtr latest_target_;

  uint8_t target_robot_id_ = 0;
  double max_velocity_ = 2.0;
  double max_acceleration_ = 2.5;

  std::shared_ptr<crane::PositionCommandWrapper> command_ = nullptr;
};

}  // namespace crane
#endif  // CRANE_SESSIONS__ROBOT_TEST_SESSION_HPP_
