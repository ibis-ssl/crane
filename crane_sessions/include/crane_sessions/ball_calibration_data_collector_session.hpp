// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__BALL_CALIBRATION_DATA_COLLECTOR_SESSION_HPP_
#define CRANE_SESSIONS__BALL_CALIBRATION_DATA_COLLECTOR_SESSION_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/ball_calibration_data_collector.hpp>
#include <crane_sessions/single_skill_session.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

namespace crane
{

/**
 * @brief ボールキャリブレーションデータ収集用プランナー（スキルランチャー）
 *
 * BallCalibrationDataCollectorスキルを実行するためのシンプルなプランナー。
 * 実際のロジックはスキル側で実装されている。
 */
class BallCalibrationDataCollectorSession
: public SingleSkillSession<skills::BallCalibrationDataCollector>
{
public:
  COMPOSITION_PUBLIC explicit BallCalibrationDataCollectorSession(
    WorldModelWrapper::SharedPtr & world_model, [[maybe_unused]] rclcpp::Node &)
  : SingleSkillSession("ball_calibration_data_collector", world_model)
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
  auto createSkill(uint8_t robot_id)
    -> std::shared_ptr<skills::BallCalibrationDataCollector> override
  {
    auto skill = std::make_shared<skills::BallCalibrationDataCollector>(robot_id, world_model);

    // デフォルトパラメータの設定
    skill->setParameter("kick_target", Point(0.0, 0.0));  // 原点にキック

    RCLCPP_INFO(
      rclcpp::get_logger("BallCalibrationDataCollectorSession"),
      "BallCalibrationDataCollectorスキル初期化: robot_id=%d", robot_id);

    return skill;
  }
};

}  // namespace crane

#endif  // CRANE_SESSIONS__BALL_CALIBRATION_DATA_COLLECTOR_SESSION_HPP_
