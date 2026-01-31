// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__CENTER_STOP_KICK_HPP_
#define CRANE_ROBOT_SKILLS__CENTER_STOP_KICK_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/vector2d_adapter.hpp>
#include <crane_physics/ball_physics_model.hpp>
#include <crane_physics/kicker_model.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <vector>

namespace crane::skills
{
enum class CenterStopKickState {
  ENTRY_POINT,           ///< エントリーポイント（初期化用）
  WAIT_BALL_STOP,        ///< ボール停止待機
  POSITION_BEHIND_BALL,  ///< ボール後方での位置・角度調整
  KICK_EXECUTE,          ///< キック実行
  KICK_COMPLETE          ///< キック完了確認
};

/**
 * @brief フィールド中心停止キックスキル
 *
 * ボールをフィールド中心(0,0)で正確に停止させるストレートキックを実行する。
 * KickerModelとBallPhysicsModelを統合して高精度な停止距離計算を行う。
 */
class CenterStopKick : public SkillBaseWithState
{
private:
  static std::string getStateName(int s);

public:
  template <typename... Args>
  explicit CenterStopKick(Args &&... args)
  : SkillBaseWithState(
      static_cast<int>(CenterStopKickState::ENTRY_POINT), &CenterStopKick::getStateName,
      "CenterStopKick", std::forward<Args>(args)...)
  {
    initialize();
  }

  void initialize();

  CenterStopKickState getCurrentState() const
  {
    return static_cast<CenterStopKickState>(SkillBaseWithState::getCurrentState());
  }

private:
  /**
   * @brief キック実行位置を取得
   * @return ボール後方のキック位置
   */
  Point getKickPosition() const;

  /**
   * @brief フィールド中心への停止距離を計算
   * @return 現在のボール位置からフィールド中心までの距離
   */
  double calculateTargetStopDistance() const;

  /**
   * @brief 必要なキック力を計算
   * @param target_distance 目標停止距離
   * @return 計算されたキック力（0.0-1.0）
   */
  double calculateRequiredKickPower(double target_distance);

  /**
   * @brief 統合物理モデルの初期化
   */
  void initializePhysicsModels();

  /**
   * @brief キック完了の確認
   * @return キックが完了し、ボールが目標方向に移動しているかどうか
   */
  bool isKickCompleted() const;

  // パラメータ
  Point target_position_{Point(0.0, 0.0)};       ///< 目標停止位置（フィールド中心）
  double kick_power_tolerance_ = 0.01;           ///< キック力計算の許容誤差
  double stop_distance_tolerance_ = 0.05;        ///< 停止距離の許容誤差 (m)
  double ball_stop_threshold_ = 0.1;             ///< ボール停止判定閾値 (m/s)
  double approach_distance_ = 0.2;               ///< キック位置までの距離 (m)
  double position_tolerance_ = 0.05;             ///< 位置許容誤差 (m)
  double stop_time_threshold_ = 1.0;             ///< 停止時間閾値 (s)
  double ball_motion_velocity_threshold_ = 0.5;  ///< ボール移動検出閾値 (m/s)
  double ball_avoidance_margin_ = 0.3;           ///< ボール回避時のマージン距離 (m)
  double calculated_kick_power_ = 0.5;           ///< 計算されたキック力
  double target_stop_distance_ = 0.0;            ///< 目標停止距離
  int max_retry_count_ = 3;                      ///< 最大リトライ回数
  double center_tolerance_ = 0.15;               ///< 中心判定許容距離 (m)

  // 物理モデル
  std::shared_ptr<KickerModel> kicker_model_;             ///< キッカーモデル
  std::shared_ptr<BallPhysicsModel> ball_physics_model_;  ///< ボール物理モデル

  // 内部状態追跡
  rclcpp::Time last_ball_motion_time_;  ///< 最後にボールが動いた時刻

  // ボール回避用状態追跡
  bool has_started_positioning_;  ///< 位置取り開始フラグ
  Point last_ball_position_;      ///< 前回のボール位置（テレポート検出用）

  // キック完了追跡
  bool kick_executed_;            ///< キック実行フラグ
  rclcpp::Time kick_start_time_;  ///< キック開始時刻

  // リトライ制御
  int retry_count_;                  ///< 現在のリトライ回数
  rclcpp::Time result_check_start_;  ///< 結果確認開始時刻

  /**
   * @brief リトライのための状態リセット
   */
  void resetForRetry();
};

}  // namespace crane::skills

#endif  // CRANE_ROBOT_SKILLS__CENTER_STOP_KICK_HPP_
