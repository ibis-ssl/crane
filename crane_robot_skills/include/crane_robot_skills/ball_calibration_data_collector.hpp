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
  ENTRY_POINT,          ///< エントリーポイント（初期化用）
  WAIT_BALL_STOP,       ///< ボール停止待機
  POSITION_BEHIND_BALL, ///< ボール後方での位置・角度調整
  KICK_EXECUTE          ///< キック実行
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
  : SkillBaseWithState<BallCalibrationState>("BallCalibrationDataCollector", std::forward<Args>(args)...),
    kick_target_(getContextReference<Point>("kick_target", Point(0.0, 0.0))),
    current_power_index_(getContextReference<int>("current_power_index", 0)),
    ball_stop_threshold_(getContextReference<double>("ball_stop_threshold", 0.1)),
    approach_distance_(getContextReference<double>("approach_distance", 0.2)),
    position_tolerance_(getContextReference<double>("position_tolerance", 0.05)),
    stop_time_threshold_(getContextReference<double>("stop_time_threshold", 1.0)),
    ball_motion_velocity_threshold_(getContextReference<double>("ball_motion_velocity_threshold", 0.5)),
    ball_avoidance_margin_(getContextReference<double>("ball_avoidance_margin", 0.3)),
    intermediate_reach_threshold_(getContextReference<double>("intermediate_reach_threshold", 0.1)),
    kick_power_sequence_({0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0})
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
   * @brief ボールが停止しているかを判定
   * @return 停止している場合true
   */
  bool isBallStopped() const;

  /**
   * @brief 現在のキックパワーを取得
   * @return 現在のキックパワー値
   */
  double getCurrentKickPower() const;

  /**
   * @brief 次のキックパワーインデックスに進める
   */
  void advanceKickPowerIndex();

  // コンテキスト変数（永続化されるパラメータ）
  Point & kick_target_;                      ///< キックターゲット位置
  int & current_power_index_;                ///< 現在のパワーインデックス
  double & ball_stop_threshold_;             ///< ボール停止判定閾値 (m/s)
  double & approach_distance_;               ///< キック位置までの距離 (m)
  double & position_tolerance_;              ///< 位置許容誤差 (m)
  double & stop_time_threshold_;             ///< 停止時間閾値 (s)
  double & ball_motion_velocity_threshold_;  ///< ボール移動検出閾値 (m/s)
  double & ball_avoidance_margin_;           ///< ボール回避時のマージン距離 (m)
  double & intermediate_reach_threshold_;    ///< 中間点到達判定閾値 (m)
  
  // 固定配列（contextに保存できない複雑な型）
  std::vector<double> kick_power_sequence_;  ///< キックパワーシーケンス

  // 内部状態追跡
  rclcpp::Time last_ball_motion_time_;       ///< 最後にボールが動いた時刻
  
  // ボール回避用状態追跡
  bool has_started_positioning_;             ///< 位置取り開始フラグ
  bool has_passed_intermediate_;             ///< 中間点通過フラグ
  Point final_target_pos_;                   ///< 最終目標位置
  Point intermediate_pos_1_;                 ///< 中間経由点1
  Point intermediate_pos_2_;                 ///< 中間経由点2
};

}  // namespace crane::skills

#endif  // CRANE_ROBOT_SKILLS__BALL_CALIBRATION_DATA_COLLECTOR_HPP_