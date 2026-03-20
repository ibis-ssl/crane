// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__PASS_RECEIVER_SESSION_HPP_
#define CRANE_SESSIONS__PASS_RECEIVER_SESSION_HPP_

#include <algorithm>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/ddps.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/receive.hpp>
#include <crane_sessions/session_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class PassReceiverSession : public SessionBase
{
public:
  std::shared_ptr<skills::Receive> receive_skill = nullptr;

  int pass_receiver_id = 0;

  Point pass_target;

  COMPOSITION_PUBLIC explicit PassReceiverSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : SessionBase("pass_receive", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    // GlobalRobotAllocator対応: robotsが変更されたらスキルを再生成
    if (robots.empty()) {
      return {SessionBase::Status::RUNNING, {}};
    }
    if (!receive_skill) {
      receive_skill = std::make_shared<skills::Receive>("receiver", robots.front().id, world_model);
      receive_skill->setParameter("policy", std::string("closest"));
      visualizer->layer = "skill/" + receive_skill->name;
    }

    // If a kick is ongoing by our team or ball is moving sufficiently, actively receive
    const bool our_kick_ongoing = [&]() {
      const auto & ks = world_model->getMsg().game_analysis.ongoing_kick;
      return !ks.empty() && ks.front().is_kicker_friend;
    }();

    if (world_model->ball().isMoving(1.0) || our_kick_ongoing) {
      // Configure receive behavior
      receive_skill->setParameter("policy", std::string("closest"));
      // Mark reserved receiver clearly
      auto pos = receive_skill->commander()->getRobot()->pose.pos;
      visualizer->drawCircle(pos, 0.25, "cyan", 18);
      visualizer->drawCenteredLabel(pos + Vector2(0.0, 0.32), "RECEIVER RESERVED", "cyan", 90);
      auto status = receive_skill->run();
      return {static_cast<SessionBase::Status>(status), {receive_skill->getRobotCommand()}};
    }

    // Pre-pass: no alignment needed; just stop and face the ball
    receive_skill->commander()->stopHere().lookAtBall();
    return {SessionBase::Status::RUNNING, {receive_skill->getRobotCommand()}};
  }

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    auto game_analysis = getGameAnalysis();
    return [game_analysis](const std::shared_ptr<RobotInfo> & robot) {
      // recommended_pass_receiver_id が設定されている場合そのロボットを最優先
      if (
        game_analysis.recommended_pass_receiver_id >= 0 &&
        robot->id == static_cast<uint8_t>(game_analysis.recommended_pass_receiver_id)) {
        return 0.0;
      }
      return 10.0;  // その他は低優先
    };
  }

protected:
  void onRobotsChanged() override { receive_skill.reset(); }
};

}  // namespace crane
#endif  // CRANE_SESSIONS__PASS_RECEIVER_SESSION_HPP_
