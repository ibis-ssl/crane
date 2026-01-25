// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__OUR_PENALTY_KICK_SESSION_HPP_
#define CRANE_SESSIONS__OUR_PENALTY_KICK_SESSION_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/penalty_kick.hpp>
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
class OurPenaltyKickSession : public SessionBase
{
private:
  std::shared_ptr<skills::PenaltyKick> kicker = nullptr;

  std::vector<std::shared_ptr<PositionCommandWrapper>> other_robots;

public:
  COMPOSITION_PUBLIC
  explicit OurPenaltyKickSession(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : SessionBase("our_penalty_kick", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    auto wm = world_model;
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      if (robot->id == wm->getOurGoalieId()) {
        return 10000.0;  // ゴールキーパーは除外
      }
      return robot->getDistance(wm->ball().pos);
    };
  }

protected:
  void onRobotsChanged() override
  {
    // ロボット割り当てが変更されたら、kickerとother_robotsを再初期化
    kicker.reset();
    other_robots.clear();

    if (!robots.empty()) {
      // 最初のロボットをキッカーとして選択
      kicker = std::make_shared<skills::PenaltyKick>(robots[0].id, world_model);

      // 残りのロボットをother_robotsに割り当て
      for (size_t i = 1; i < robots.size(); ++i) {
        auto command =
          std::make_shared<PositionCommandWrapper>("our_penalty_kick", robots[i].id, world_model);
        other_robots.push_back(command);
      }
    }
  }
};
}  // namespace crane
#endif  // CRANE_SESSIONS__OUR_PENALTY_KICK_SESSION_HPP_
