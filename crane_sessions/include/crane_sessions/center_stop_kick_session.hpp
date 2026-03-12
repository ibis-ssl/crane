// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__CENTER_STOP_KICK_SESSION_HPP_
#define CRANE_SESSIONS__CENTER_STOP_KICK_SESSION_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/center_stop_kick.hpp>
#include <crane_sessions/single_skill_session.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

namespace crane
{

/**
 * @brief フィールド中心停止キック用プランナー
 *
 * CenterStopKickスキルを実行するためのシンプルなプランナー。
 * ボールをフィールド中心(0,0)で正確に停止させるストレートキックを行う。
 * 実際のロジックはスキル側で実装されている。
 */
class CenterStopKickSession : public SingleSkillSession<skills::CenterStopKick>
{
public:
  COMPOSITION_PUBLIC explicit CenterStopKickSession(
    WorldModelWrapper::SharedPtr & world_model, [[maybe_unused]] rclcpp::Node &)
  : SingleSkillSession("center_stop_kick", world_model)
  {
  }

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    return [](const std::shared_ptr<RobotInfo> & robot) {
      return static_cast<double>(robot->id);  // ID小優先
    };
  }

protected:
  auto createSkill(uint8_t robot_id) -> std::shared_ptr<skills::CenterStopKick> override
  {
    auto skill = std::make_shared<skills::CenterStopKick>(robot_id, world_model);

    // デフォルトパラメータの設定
    skill->setParameter("target_position", Point(0.0, 0.0));  // フィールド中心にキック
    skill->setParameter("kick_power_tolerance", 0.01);        // キック力計算精度
    skill->setParameter("stop_distance_tolerance", 0.05);     // 停止距離許容誤差5cm

    RCLCPP_INFO(
      rclcpp::get_logger("CenterStopKickSession"), "CenterStopKickスキル初期化: robot_id=%d",
      robot_id);

    return skill;
  }
};

}  // namespace crane

#endif  // CRANE_SESSIONS__CENTER_STOP_KICK_SESSION_HPP_
