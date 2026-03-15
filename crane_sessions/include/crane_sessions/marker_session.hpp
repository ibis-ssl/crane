// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__MARKER_SESSION_HPP_
#define CRANE_SESSIONS__MARKER_SESSION_HPP_

#include <algorithm>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/marker.hpp>
#include <crane_sessions/marker_functions.hpp>
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
class MarkerSession : public SessionBase
{
public:
  COMPOSITION_PUBLIC
  explicit MarkerSession(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : SessionBase("marker", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    // 危険な敵ロボットの平均位置を1回だけ計算してキャッシュ
    auto danger_enemies = getDangerEnemies(world_model);
    Point center(0.0, 0.0);
    if (!danger_enemies.empty()) {
      for (const auto & [enemy, score] : danger_enemies) {
        center += enemy->pose.pos;
      }
      center /= static_cast<double>(danger_enemies.size());
    }
    bool has_enemies = !danger_enemies.empty();
    return [center, has_enemies](const std::shared_ptr<RobotInfo> & robot) {
      if (!has_enemies) {
        // 敵がいない場合はフィールド中央への距離を返す
        return robot->getDistance(Point(0.0, 0.0));
      }
      // 危険な敵ロボットの中心位置に近いロボットを優先
      return robot->getDistance(center);
    };
  }

  int getDesiredRobotNumber(int /* min_robots */, int /* max_robots */) const override
  {
    // 危険な敵の数に基づいてロボット数を決定
    auto danger_enemies = getDangerEnemies(world_model);
    int count = static_cast<int>(danger_enemies.size());
    // ボールが高速移動中はマーキング需要を下げてパスカット要員を確保
    if (world_model->ball().isMoving(1.0)) {
      count = std::max(0, count - 2);
    }
    return count;
  }

private:
  std::vector<std::shared_ptr<skills::Marker>> markers;

  std::mutex markers_mutex;

  // 前フレームのマーカー割り当て（ヒステリシス用: enemy_id -> our_robot_id）
  std::unordered_map<uint8_t, uint8_t> prev_assignments_;
};

}  // namespace crane
#endif  // CRANE_SESSIONS__MARKER_SESSION_HPP_
