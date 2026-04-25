// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__ATTACKER_HEAT_ROTATION_SESSION_HPP_
#define CRANE_SESSIONS__ATTACKER_HEAT_ROTATION_SESSION_HPP_

#include <algorithm>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_sessions/attacker_skill_session.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_set>
#include <vector>

#include "visibility_control.h"

namespace crane
{
/// 候補IDプール内でモーター温度ベースに1台を選ぶ Attacker 練習用セッション。
///
/// 候補IDは YAML の `candidate_robots: [1, 5, 9]` から受け取る
/// （SessionBase::setCandidateRobots 経由）。
/// 基本は同じロボットを使い続け、その最大モータ温度が閾値（params.rotation_trigger_celsius、
/// 既定 60℃）を超えた瞬間に、候補内で最も冷えたロボットへ切り替える。
/// allocator の `fixed_robots` 機構は使わず、suitability 関数の中で
/// 「候補外排除」「現在割当の維持」「閾値超でローテ」を表現することで、
/// ローテーションロジックをこのセッション内に閉じ込めている。
class AttackerHeatRotationSession : public AttackerSkillSession
{
public:
  /// ローテーションが発火するモータ温度の既定しきい値（℃）。
  /// YAML の params: { rotation_trigger_celsius: ... } で上書き可。
  static constexpr double DEFAULT_ROTATION_TRIGGER_CELSIUS = 60.0;

  /// 候補外ロボットへの大きなペナルティ（実用上選ばれない値）。
  static constexpr double NON_CANDIDATE_COST = 1e6;

  /// 現在割当ロボットを「閾値未満の間は確実に維持する」ための負コスト。
  /// 他候補は max_temp（実温度）を返すので、負値であれば必ず勝つ。
  static constexpr double KEEP_CURRENT_COST = -1.0e3;

  COMPOSITION_PUBLIC explicit AttackerHeatRotationSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  : AttackerSkillSession(world_model, node)
  {
    setUseCandidateRobots(true);
  }

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    const std::unordered_set<uint8_t> candidates(
      candidate_robots_.begin(), candidate_robots_.end());

    // 前ティックの割当をキャプチャ。閾値判定の基準になる。
    std::vector<uint8_t> currently_assigned;
    currently_assigned.reserve(robots.size());
    for (const auto & r : robots) {
      currently_assigned.push_back(r.id);
    }

    const double trigger =
      getSessionParameter<double>("rotation_trigger_celsius", DEFAULT_ROTATION_TRIGGER_CELSIUS);

    return [candidates, currently_assigned, trigger](const std::shared_ptr<RobotInfo> & robot) {
      // 候補外ロボットは事実上選ばれない高コストにする
      if (!candidates.empty() && candidates.count(robot->id) == 0) {
        return NON_CANDIDATE_COST;
      }

      double max_temp = 0.0;
      for (auto t : robot->motor_temperatures) {
        max_temp = std::max(max_temp, static_cast<double>(t));
      }

      const bool is_current =
        std::find(currently_assigned.begin(), currently_assigned.end(), robot->id) !=
        currently_assigned.end();

      // 現在割当ロボットが閾値未満なら、温度に関係なく維持する。
      // KEEP_CURRENT_COST は他候補が返す温度値（正）より必ず小さいため必ず選ばれる。
      if (is_current && max_temp < trigger) {
        return KEEP_CURRENT_COST;
      }

      // 現在割当が閾値を超えた、または非current。
      // 温度を素直にコストとして返し、候補内で最も冷えたロボットに切替える。
      return max_temp;
    };
  }
};

}  // namespace crane
#endif  // CRANE_SESSIONS__ATTACKER_HEAT_ROTATION_SESSION_HPP_
