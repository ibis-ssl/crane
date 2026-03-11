// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__OUR_FREE_KICK_SESSION_HPP_
#define CRANE_SESSIONS__OUR_FREE_KICK_SESSION_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
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
class OurDirectFreeKickSession : public SessionBase
{
private:
  std::shared_ptr<PositionCommandWrapper> kicker = nullptr;

  std::vector<std::shared_ptr<PositionCommandWrapper>> other_robots;

  bool fake_over = false;

  int fake_count = 0;

  // 前回のロボットロール情報を保存（calculatePositionCommand()で使用）
  std::unordered_map<uint8_t, RobotRole> cached_prev_roles;

public:
  COMPOSITION_PUBLIC
  explicit OurDirectFreeKickSession(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : SessionBase("our_direct_free", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    auto wm = world_model;
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      if (robot->id == wm->getOurGoalieId()) {
        return GOALIE_EXCLUSION_COST;  // ゴールキーパーは除外
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
    fake_over = false;
    fake_count = 0;

    if (!robots.empty()) {
      // 最初のロボットをキッカーとして選択
      kicker =
        std::make_shared<PositionCommandWrapper>("our_direct_free_kick", robots[0].id, world_model);

      // 残りのロボットをother_robotsに割り当て
      for (size_t i = 1; i < robots.size(); ++i) {
        auto command = std::make_shared<PositionCommandWrapper>(
          "our_direct_free_kick", robots[i].id, world_model);
        other_robots.push_back(command);
      }
    }
  }
};
}  // namespace crane
#endif  // CRANE_SESSIONS__OUR_FREE_KICK_SESSION_HPP_
