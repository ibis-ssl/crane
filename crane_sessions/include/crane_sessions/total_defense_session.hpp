// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__TOTAL_DEFENSE_SESSION_HPP_
#define CRANE_SESSIONS__TOTAL_DEFENSE_SESSION_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/goalie.hpp>
#include <crane_robot_skills/marker.hpp>
#include <crane_robot_skills/second_threat_defender.hpp>
#include <crane_sessions/defense_functions.hpp>
#include <crane_sessions/marker_functions.hpp>
#include <crane_sessions/session_base.hpp>
#include <crane_utils/stream.hpp>
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class TotalDefenseSession : public SessionBase
{
public:
  std::shared_ptr<skills::Goalie> goalie;

  std::shared_ptr<skills::SecondThreatDefender> second_threat_defender;

  std::vector<std::shared_ptr<skills::Marker>> markers;

private:
  bool m_is_goalie_total_defense_mode = true;

  std::mutex markers_mutex;

  /// マーキングターゲットを割り当て
  auto assignMarkingTargets(const std::vector<uint8_t> & available_robots) -> std::vector<uint8_t>;

public:
  COMPOSITION_PUBLIC
  explicit TotalDefenseSession(
    WorldModelWrapper::SharedPtr & world_model, [[maybe_unused]] rclcpp::Node & node)
  : SessionBase("total_defense", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    // 守備位置に近いロボットを優先
    auto wm = world_model;  // shared_ptrをコピー
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      Segment ball_line{wm->goal(), wm->ball().pos};
      auto parameter = getDefenseLinePointParameter(ball_line, wm);
      if (not parameter) {
        // パラメータが取得できない場合はゴール位置への距離を使用
        return robot->getDistance(wm->getOurGoalCenter());
      }
      const auto defense_point = getDefenseLinePoint(parameter.value(), wm);
      return robot->getDistance(defense_point);
    };
  }
};

}  // namespace crane
#endif  // CRANE_SESSIONS__TOTAL_DEFENSE_SESSION_HPP_
