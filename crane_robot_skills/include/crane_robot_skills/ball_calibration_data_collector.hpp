// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__BALL_CALIBRATION_DATA_COLLECTOR_HPP_
#define CRANE_ROBOT_SKILLS__BALL_CALIBRATION_DATA_COLLECTOR_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/vector2d_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <vector>

namespace crane::skills
{
enum class BallCalibrationState {
  ENTRY_POINT,           ///< エントリーポイント（初期化用）
  WAIT_BALL_STOP,        ///< ボール停止待機
  POSITION_BEHIND_BALL,  ///< ボール後方での位置・角度調整
  KICK_EXECUTE           ///< キック実行
};

/**
 * @brief ボールキャリブレーション用データ収集スキル
 *
 * ボール停止確認後に段階的に位置取りを行い、
 * 様々なパワーでキックを実行してデータ収集を行う
 */
class BallCalibrationDataCollector : public SkillBaseWithState<BallCalibrationState>
{
public:
  template <typename... Args>
  explicit BallCalibrationDataCollector(Args &&... args)
  : SkillBaseWithState<BallCalibrationState>(
      "BallCalibrationDataCollector", std::forward<Args>(args)...)
  {
    initialize();
  }

  void initialize();

  void print(std::ostream & os) const override { os << "[BallCalibrationDataCollector]"; }

private:
  /**
   * @brief キック実行位置を取得
   * @return ボール後方のキック位置
   */
  Point getKickPosition() const;

  /**
   * @brief 現在のキックパワーを取得
   * @return 現在のキックパワー値
   */
  double getCurrentKickPower() const;

  /**
   * @brief 次のキックパワーインデックスに進める
   */
  void advanceKickPowerIndex();

  // パラメータ
  Point kick_target_{Point(0.0, 0.0)};           ///< キックターゲット位置
  int current_power_index_ = 0;                  ///< 現在のパワーインデックス
  double ball_stop_threshold_ = 0.2;             ///< ボール停止判定閾値 (m/s)
  double approach_distance_ = 0.2;               ///< キック位置までの距離 (m)
  double position_tolerance_ = 0.05;             ///< 位置許容誤差 (m)
  double stop_time_threshold_ = 1.0;             ///< 停止時間閾値 (s)
  double ball_motion_velocity_threshold_ = 0.5;  ///< ボール移動検出閾値 (m/s)
  double ball_avoidance_margin_ = 0.3;           ///< ボール回避時のマージン距離 (m)
  std::vector<double> kick_power_sequence_{0.2, 0.3, 0.4, 0.5, 0.6,
                                           0.7, 0.8, 0.9, 1.0};  ///< キックパワーシーケンス

  // 内部状態追跡
  rclcpp::Time last_ball_motion_time_;  ///< 最後にボールが動いた時刻

  // ボール回避用状態追跡
  bool has_started_positioning_;  ///< 位置取り開始フラグ
  Point last_ball_position_;      ///< 前回のボール位置（テレポート検出用）
};

}  // namespace crane::skills

#endif  // CRANE_ROBOT_SKILLS__BALL_CALIBRATION_DATA_COLLECTOR_HPP_
