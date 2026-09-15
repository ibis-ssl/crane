// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__METRICS__METRIC_BASE_HPP_
#define CRANE_GAME_ANALYZER__METRICS__METRIC_BASE_HPP_

#include <crane_visualization_interfaces/crane_visualizer_wrapper.hpp>
#include <memory>
#include <string>
#include <vector>

#include "metric_context.hpp"

namespace crane::metrics
{

/**
 * @brief メトリクスの識別子
 *
 * 各メトリクスを一意に識別するための列挙型
 */
enum class MetricId {
  // 基礎メトリクス
  BALL_HORIZON,  ///< ボールライン長
  OUR_SLACK,     ///< 味方ロボットのSlack時間
  THEIR_SLACK,   ///< 敵ロボットのSlack時間

  // 脅威評価
  BALL_THREAT,            ///< ボール脅威
  ROBOT_THREATS,          ///< ロボット脅威（優先度順）
  RECOMMENDED_DEFENDERS,  ///< 推奨守備者数

  // 役割決定（新規）
  ATTACKER_CANDIDATE,     ///< 推奨アタッカー
  SUB_ATTACKER_POSITION,  ///< SubAttacker推奨位置

  // キック検出
  ONGOING_KICK,  ///< 進行中キック検出

  // パス評価
  PASS_TARGET,  ///< パスターゲット選定
  PASS_PLAN,    ///< パス計画（誰が・誰に・どの受領点へ・シャドー運用）
};

/**
 * @brief メトリクスの基底クラス
 *
 * 全てのメトリクスはこのクラスを継承し、依存関係と計算処理を実装する
 */
class MetricBase
{
public:
  using Ptr = std::shared_ptr<MetricBase>;

  /**
   * @brief コンストラクタ
   * @param id メトリクス識別子
   * @param name メトリクス名（ログ出力用）
   */
  explicit MetricBase(MetricId id, const std::string & name) : id_(id), name_(name) {}

  virtual ~MetricBase() = default;

  // コピー・ムーブを禁止
  MetricBase(const MetricBase &) = delete;
  MetricBase(MetricBase &&) = delete;
  auto operator=(const MetricBase &) -> MetricBase & = delete;
  auto operator=(MetricBase &&) -> MetricBase & = delete;

  /**
   * @brief メトリクス識別子を取得
   * @return メトリクスID
   */
  [[nodiscard]] auto getId() const -> MetricId { return id_; }

  /**
   * @brief メトリクス名を取得
   * @return メトリクス名
   */
  [[nodiscard]] auto getName() const -> const std::string & { return name_; }

  /**
   * @brief このメトリクスが依存する他のメトリクスのIDリストを返す
   *
   * 依存関係を宣言することで、MetricEngineが自動的に計算順序を決定する
   * @return 依存するメトリクスIDのリスト
   */
  [[nodiscard]] virtual auto getDependencies() const -> std::vector<MetricId> = 0;

  /**
   * @brief メトリクスを計算し、結果をctx.analysisに書き込む
   *
   * @param ctx 計算コンテキスト（ワールドモデル、出力先等）
   */
  virtual auto compute(MetricContext & ctx) -> void = 0;

  /**
   * @brief メトリクスの計算結果を可視化（オプション）
   *
   * デフォルト実装は何もしない。必要に応じて派生クラスでオーバーライド
   * @param ctx 計算コンテキスト
   * @param visualizer 可視化メッセージビルダー
   */
  virtual auto visualize(
    MetricContext & ctx, const VisualizerMessageBuilder::SharedPtr & visualizer) -> void
  {
    (void)ctx;
    (void)visualizer;
  }

protected:
  MetricId id_;       ///< メトリクスID
  std::string name_;  ///< メトリクス名
};

}  // namespace crane::metrics

#endif  // CRANE_GAME_ANALYZER__METRICS__METRIC_BASE_HPP_
