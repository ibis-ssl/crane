// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__MARKER_TACTIC_HPP_
#define CRANE_TACTICS__MARKER_TACTIC_HPP_

#include <algorithm>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/marker.hpp>
#include <crane_tactics/marker_functions.hpp>
#include <crane_tactics/tactic_base.hpp>
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
class MarkerTactic : public TacticBase
{
public:
  COMPOSITION_PUBLIC
  explicit MarkerTactic(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : TacticBase("marker", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    // 危険な敵ロボットの中心位置に近いロボットを優先
    auto wm = world_model;  // shared_ptrをコピー
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      auto danger_enemies = getDangerEnemies(wm);
      if (danger_enemies.empty()) {
        // 敵がいない場合はフィールド中央への距離を返す
        return robot->getDistance(Point(0.0, 0.0));
      }
      // 危険な敵ロボットの平均位置を計算
      Point center(0.0, 0.0);
      for (const auto & [enemy, score] : danger_enemies) {
        center += enemy->pose.pos;
      }
      center /= static_cast<double>(danger_enemies.size());
      return robot->getDistance(center);
    };
  }

  int getDesiredRobotNumber(int /* min_robots */, int /* max_robots */) const override
  {
    // 危険な敵の数に基づいてロボット数を決定
    auto danger_enemies = getDangerEnemies(world_model);
    return static_cast<int>(danger_enemies.size());
  }

private:
  auto assignMarkingTarget(
    uint8_t selectable_robots_num, const std::vector<uint8_t> selectable_robots)
    -> std::vector<uint8_t>;

  std::vector<std::shared_ptr<skills::Marker>> markers;

  std::mutex markers_mutex;
};

}  // namespace crane
#endif  // CRANE_TACTICS__MARKER_TACTIC_HPP_
