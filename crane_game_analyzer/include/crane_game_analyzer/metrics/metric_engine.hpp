// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__METRICS__METRIC_ENGINE_HPP_
#define CRANE_GAME_ANALYZER__METRICS__METRIC_ENGINE_HPP_

#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <vector>

#include "metric_base.hpp"

namespace crane::metrics
{

/**
 * @brief メトリクス計算エンジン
 *
 * 登録されたメトリクスの依存関係を解決し、適切な順序で計算を実行する
 * - トポロジカルソート（Kahnのアルゴリズム）による計算順序決定
 * - 循環依存の検出
 * - 全メトリクスの一括計算・可視化
 */
class MetricEngine
{
public:
  /**
   * @brief コンストラクタ
   * @param logger ROS2ロガー（エラー・情報出力用）
   */
  explicit MetricEngine(rclcpp::Logger logger) : logger_(logger) {}

  /**
   * @brief メトリクスを登録
   *
   * 登録順序は不問（エンジンが依存関係を解決）
   * @param metric 登録するメトリクス
   */
  auto registerMetric(MetricBase::Ptr metric) -> void;

  /**
   * @brief 初期化（トポロジカルソート・循環依存検出）
   *
   * 全メトリクス登録後、一度だけ呼び出す
   * @return 成功: true, 失敗（循環依存等）: false
   */
  auto initialize() -> bool;

  /**
   * @brief 全メトリクスを計算順序に従って実行
   *
   * @param ctx 計算コンテキスト
   */
  auto computeAll(MetricContext & ctx) -> void;

  /**
   * @brief 全メトリクスの可視化を実行
   *
   * @param ctx 計算コンテキスト
   * @param visualizer 可視化メッセージビルダー
   */
  auto visualizeAll(MetricContext & ctx, const VisualizerMessageBuilder::SharedPtr & visualizer)
    -> void;

private:
  /**
   * @brief トポロジカルソートを実行し、実行順序を構築
   *
   * Kahnのアルゴリズムを使用:
   * 1. 入次数0のノード（依存なし）をキューに追加
   * 2. キューから取り出してexecution_order_に追加
   * 3. そのノードから出ている辺を削除し、入次数を更新
   * 4. 全ノードを処理できれば成功、残りがあれば循環依存
   *
   * @return 成功: true, 失敗（循環依存）: false
   */
  auto buildExecutionOrder() -> bool;

  rclcpp::Logger logger_;  ///< ROS2ロガー

  /// メトリクスID -> メトリクスのマップ
  std::unordered_map<MetricId, MetricBase::Ptr> metrics_;

  /// 実行順序（トポロジカルソート済み）
  std::vector<MetricId> execution_order_;
};

}  // namespace crane::metrics

#endif  // CRANE_GAME_ANALYZER__METRICS__METRIC_ENGINE_HPP_
