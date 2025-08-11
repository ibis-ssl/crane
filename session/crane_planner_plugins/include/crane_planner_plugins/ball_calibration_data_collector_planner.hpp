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
#include <crane_robot_skills/kick.hpp>
#include <crane_robot_skills/receive.hpp>
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
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node);

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots, PlannerContext & context) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
    -> std::vector<uint8_t> override;

private:
  enum class CollectorState {
    SETUP_POSITIONS,    ///< 初期配置
    KICK_PREPARATION,   ///< キック準備
    EXECUTING_KICK,     ///< キック実行
    WAITING_BALL_STOP,  ///< ボール停止待機
    BALL_RETRIEVAL,     ///< ボール回収
    RETURN_PASS,        ///< 返球
    CYCLE_COMPLETE      ///< サイクル完了
  };

  /**
   * @brief 現在の状態を文字列で取得
   * @param state 状態
   * @return 状態の文字列表現
   */
  std::string getStateString(CollectorState state) const;

  /**
   * @brief ボールがフィールド外で停止するかを予測
   * @return フィールド外停止の場合true
   */
  bool willBallStopOutsideField() const;

  /**
   * @brief ボールが完全に停止しているかを判定
   * @return 停止している場合true
   */
  bool isBallFullyStopped() const;

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
   * @return 次のキックパワー値
   */
  double getNextKickPower();

  /**
   * @brief 現在のサイクルをリセット
   */
  void resetCycle();

  // 状態管理
  CollectorState current_state_;
  rclcpp::Time state_start_time_;

  // ロボット管理
  uint8_t kicker_robot_id_;
  uint8_t retriever_robot_id_;

  // スキル
  std::shared_ptr<skills::Kick> kicker_skill_;
  std::shared_ptr<skills::Receive> retriever_skill_;

  // パラメータ
  double kicker_x_offset_;                   ///< キッカーのx方向オフセット（デフォルト1.0m）
  std::vector<double> kick_power_sequence_;  ///< キックパワーシーケンス
  size_t current_power_index_;               ///< 現在のパワーインデックス
  double ball_stop_timeout_;                 ///< ボール停止判定タイムアウト（秒）
  double field_boundary_margin_;             ///< フィールド境界マージン（m）
  size_t data_collection_cycles_;            ///< 収集サイクル数
  size_t current_cycle_;                     ///< 現在のサイクル数

  // 状態追跡
  rclcpp::Time last_ball_motion_time_;  ///< 最後にボールが動いた時刻
  bool kick_executed_;                  ///< キックが実行されたフラグ
  Point ball_stop_position_;            ///< ボール停止位置

  // ノードハンドル
  rclcpp::Node & node_;
};

}  // namespace crane

#endif  // CRANE_PLANNER_PLUGINS__BALL_CALIBRATION_DATA_COLLECTOR_PLANNER_HPP_
