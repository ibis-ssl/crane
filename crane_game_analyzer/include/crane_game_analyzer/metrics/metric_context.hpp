// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__METRICS__METRIC_CONTEXT_HPP_
#define CRANE_GAME_ANALYZER__METRICS__METRIC_CONTEXT_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/ball_info.hpp>
#include <crane_msgs/msg/game_analysis.hpp>
#include <deque>
#include <rclcpp/rclcpp.hpp>

namespace crane::metrics
{

/**
 * @brief メトリクス計算時に共有される計算コンテキスト
 *
 * 全メトリクスが共通してアクセスする入力データと出力先を保持
 */
struct MetricContext
{
  /// ワールドモデル（ロボット位置、ボール情報等）
  /// ポインタで保持（参照と異なりnullableだが、実際には常に有効）
  WorldModelWrapper * world_model;

  /// ボール位置履歴（キック検出等に使用）
  std::deque<crane_msgs::msg::BallInfo> * ball_history;

  /// ROS2クロック（時刻取得用）
  rclcpp::Clock::SharedPtr clock;

  /// 計算済みメトリクス結果の出力先
  /// 各メトリクスはこのメッセージの該当フィールドに結果を書き込む
  crane_msgs::msg::GameAnalysis & analysis;
};

}  // namespace crane::metrics

#endif  // CRANE_GAME_ANALYZER__METRICS__METRIC_CONTEXT_HPP_
