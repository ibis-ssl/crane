// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__BALL_CALIBRATION_DATA_COLLECTOR_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__BALL_CALIBRATION_DATA_COLLECTOR_PLANNER_HPP_

#include <algorithm>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_plugins/planner_base.hpp>
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

/**
 * @brief キャリブレーション用データ収集プランナー
 *
 * 良質なキックデータを自動収集するための専用プランナー。
 * 1台のキッカーロボットと1台の球拾いロボットを使用して、
 * 様々なパワー設定での体系的なキックデータを収集する。
 */
class BallCalibrationDataCollectorPlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC explicit BallCalibrationDataCollectorPlanner(
    WorldModelWrapper::SharedPtr & world_model, [[maybe_unused]] rclcpp::Node &);

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots, PlannerContext & context) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
    -> std::vector<uint8_t> override;

private:
  enum class NewCollectorState {
    INITIALIZE,      ///< 初期配置: システム全体の初期化
    KICK_APPROACH,   ///< キック接近: ボール後方への精密位置取り
    KICK_EXECUTE,    ///< キック実行: 角度調整→突進→完了の全プロセス
    BALL_INTERCEPT,  ///< ボール迎撃: 予測位置での効率的回収
    BALL_RETURN      ///< ボール返却: キッカーへの正確な返球
  };

  /**
   * @brief キッカーの目標位置を取得
   * @return キッカーの目標位置
   */
  Point getKickerTargetPosition() const;

  /**
   * @brief 球拾いロボットの待機位置を取得
   * @return 球拾いロボットの待機位置
   */
  Point getRetrieverWaitingPosition() const;

  /**
   * @brief 次のキックパワーを取得
   * @param kick_power_sequence キックパワーシーケンス
   * @return 次のキックパワー値
   */
  double getNextKickPower(const std::vector<double> & kick_power_sequence);

  /**
   * @brief ボール迎撃の最適位置を計算
   * @return 迎撃位置
   */
  Point calculateOptimalInterceptPosition() const;

  // 状態管理
  NewCollectorState current_state_;
  rclcpp::Time state_start_time_;

  // ロボット管理
  std::shared_ptr<RobotCommandWrapper> kicker;
  std::shared_ptr<RobotCommandWrapper> retriever;

  // パラメータ
  size_t current_power_index_;  ///< 現在のパワーインデックス
  size_t current_cycle_;        ///< 現在のサイクル数

  // 状態追跡
  rclcpp::Time last_ball_motion_time_;  ///< 最後にボールが動いた時刻
  Point ball_stop_position_;            ///< ボール停止位置
};

}  // namespace crane

#endif  // CRANE_PLANNER_PLUGINS__BALL_CALIBRATION_DATA_COLLECTOR_PLANNER_HPP_
