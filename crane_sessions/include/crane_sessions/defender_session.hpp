// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__DEFENDER_SESSION_HPP_
#define CRANE_SESSIONS__DEFENDER_SESSION_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_physics/position_assignments.hpp>
#include <crane_sessions/defense_functions.hpp>
#include <crane_sessions/session_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class DefenderSession : public SessionBase
{
public:
  COMPOSITION_PUBLIC
  explicit DefenderSession(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : SessionBase("defender", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override;

  int getDesiredRobotNumber(int min_robots, int max_robots) const override
  {
    if (world_model->point_checker.isInOurHalf(world_model->ball().pos)) {
      return max_robots;
    }
    return min_robots;
  }

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    // 守備ライン位置に近いロボットを優先
    auto wm = world_model;  // shared_ptrをコピー
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      if (robot->id == wm->getOurGoalieId()) {
        return GOALIE_EXCLUSION_COST;
      }
      Segment ball_line{wm->goal(), wm->ball().pos};
      auto parameter = getDefenseLinePointParameter(ball_line, wm);
      if (not parameter) {
        return 1000.0;  // パラメータが無効な場合は大きなコスト
      }
      const auto defense_point = getDefenseLinePoint(parameter.value(), wm);
      return robot->getDistance(defense_point);
    };
  }
};

}  // namespace crane
#endif  // CRANE_SESSIONS__DEFENDER_SESSION_HPP_
