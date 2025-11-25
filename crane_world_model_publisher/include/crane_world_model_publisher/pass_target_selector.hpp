// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__PASS_TARGET_SELECTOR_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__PASS_TARGET_SELECTOR_HPP_

#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/ball_info.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <crane_physics/slack_time_config.hpp>
#include <deque>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace crane
{
/**
 * @brief パススコアの計算およびパス先IDの選定（ヒステリシス付き）を担当するクラス
 */
class PassTargetSelector
{
public:
  PassTargetSelector() = default;

  /**
   * @brief スイッチ抑制に関するパラメータを設定
   * @param min_hold_sec ターゲット保持の最短秒数
   * @param min_improvement 早期切替のための最小スコア改善幅
   */
  auto setHysteresisParams(double min_hold_sec, double min_improvement) -> void
  {
    min_hold_duration_sec_ = min_hold_sec;
    min_improvement_margin_ = min_improvement;
  }

  auto setEnemySlackConfig(const SlackTimeConfig & config, double slack_scale = 0.5) -> void
  {
    enemy_slack_config_ = config;
    slack_scale_ = slack_scale;
  }

  /**
   * @brief パススコアを算出し、ヒステリシス込みでpass_target_idを選定。可視化も行う。
   * @param world_model WorldModelの参照
   * @param ball_history 直近のBallInfo履歴（パス起点フォールバックに使用）
   * @param pass_builder 可視化ビルダ（パス描画）
   * @param[out] analysis_msg 出力先GameAnalysis（pass_scores, pass_target_idを設定）
   */
  auto update(
    const WorldModelWrapper::SharedPtr & world_model,
    const std::deque<crane_msgs::msg::BallInfo> & ball_history,
    const VisualizerMessageBuilder::SharedPtr & pass_builder,
    crane_msgs::msg::GameAnalysis & analysis_msg) -> void;

private:
  // 評価に使用するパス起点（停止/移動/履歴/キック起点を考慮）を求める
  [[nodiscard]] auto computePassOrigin(
    const WorldModelWrapper::SharedPtr & world_model,
    const std::deque<crane_msgs::msg::BallInfo> & ball_history,
    const crane_msgs::msg::GameAnalysis & analysis_msg) const -> Point;

  // スコア関数
  [[nodiscard]] auto calcScore(
    const WorldModelWrapper::SharedPtr & world_model, const Point & pass_origin,
    const Point & p) const -> double;

  // 可視化（選定されたパスの描画）
  auto visualize(
    const WorldModelWrapper::SharedPtr & world_model, const Point & pass_origin, uint8_t target_id,
    const VisualizerMessageBuilder::SharedPtr & pass_builder) -> void;

  // 内部状態（ヒステリシス用）
  std::optional<int> last_pass_target_id_{std::nullopt};
  rclcpp::Time last_switch_time_{static_cast<int64_t>(0), RCL_ROS_TIME};

  // パラメータ（ヒステリシス）
  double min_hold_duration_sec_ = 0.5;
  double min_improvement_margin_ = 0.2;

  // 敵インターセプト評価用パラメータ
  SlackTimeConfig enemy_slack_config_{
    .robot_max_acceleration = 3.0,  // 敵は自チームより高め（安全マージン）
    .robot_max_velocity = 5.5       // 敵は自チームより高め（安全マージン）
  };
  double slack_scale_ = 0.5;  // スコア正規化用

  // 時刻取得用クロック
  rclcpp::Clock ros_clock_{RCL_ROS_TIME};
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__PASS_TARGET_SELECTOR_HPP_
