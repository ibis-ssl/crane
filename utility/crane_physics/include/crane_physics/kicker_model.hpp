// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PHYSICS__KICKER_MODEL_HPP_
#define CRANE_PHYSICS__KICKER_MODEL_HPP_

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// 前方宣言でBallPhysicsModelとの循環依存を回避
namespace crane
{
class BallPhysicsModel;  // TODO: PR5で実装予定
struct Ball;
}  // namespace crane

namespace crane
{

/**
 * @brief キック力とボール初速度の変換を行うキッカーモデル
 *
 * このクラスは以下の機能を提供：
 * - キック力からボール初速度への変換（キック予測）
 * - ボール初速度からキック力への変換（逆算）
 * - 目標停止距離からのキック力計算（BallPhysicsModel連携）
 * - ストレートキックとチップキックの両方をサポート
 */
class KickerModel
{
public:
  /**
   * @brief キッカーモデルの設定パラメータ
   */
  struct Config
  {
    // ストレートキック設定（キック力 <-> ボール初速度）
    std::vector<double> straight_kick_powers;  // キック力の配列 [0.0-1.0]
    std::vector<double> straight_kick_speeds;  // 対応するボール初速度 [m/s]

    // チップキック設定（キック力 <-> 飛行距離）
    std::vector<double> chip_kick_powers;     // キック力の配列 [0.0-1.0]
    std::vector<double> chip_kick_distances;  // 対応する飛行距離 [m]

    // デフォルト設定値（grSim用）
    Config()
    {
      straight_kick_powers = {0.0, 0.25, 0.6, 0.9};
      straight_kick_speeds = {0.0, 2.0, 4.0, 6.0};
      chip_kick_powers = {0.0, 0.5, 0.75, 1.0};
      chip_kick_distances = {0.0, 0.3, 1.0, 2.5};
    }
  };

  /**
   * @brief デフォルト設定でKickerModelを構築
   */
  KickerModel();

  /**
   * @brief 指定された設定でKickerModelを構築
   * @param config キッカーモデルの設定
   */
  explicit KickerModel(const Config & config);

  /**
   * @brief 指定された設定とBallPhysicsModelでKickerModelを構築
   * @param config キッカーモデルの設定
   * @param ball_physics BallPhysicsModelの共有ポインタ
   */
  KickerModel(const Config & config, std::shared_ptr<BallPhysicsModel> ball_physics);

  /**
   * @brief デストラクタ
   */
  ~KickerModel() = default;

  // コピー・ムーブ演算子
  KickerModel(const KickerModel &) = default;
  KickerModel(KickerModel &&) = default;
  KickerModel & operator=(const KickerModel &) = default;
  KickerModel & operator=(KickerModel &&) = default;

  // ===== YAML設定ファイル読み込み =====

  /**
   * @brief YAMLファイルからキッカーモデル設定を読み込み
   * @param yaml_file_path YAMLファイルのパス
   * @return 読み込まれた設定
   * @throws std::runtime_error ファイル読み込みまたは解析に失敗した場合
   */
  static auto loadConfigFromYAML(const std::string & yaml_file_path) -> Config;

  /**
   * @brief YAML設定ファイルからKickerModelを作成
   * @param yaml_file_path YAMLファイルのパス
   * @return 作成されたKickerModel
   */
  static auto createWithYAMLConfig(const std::string & yaml_file_path) -> KickerModel;

  // ===== キック力 ↔ ボール初速度変換 =====

  /**
   * @brief キック力からボール初速度を予測（ストレートキック）
   * @param kick_power キック力 [0.0-1.0]
   * @return 予測されるボール初速度 [m/s]
   */
  [[nodiscard]] auto predictStraightKickSpeed(double kick_power) const -> double;

  /**
   * @brief キック力からチップキック飛行距離を予測
   * @param kick_power キック力 [0.0-1.0]
   * @return 予測される飛行距離 [m]
   */
  [[nodiscard]] auto predictChipKickDistance(double kick_power) const -> double;

  /**
   * @brief 目標ボール初速度からストレートキック力を計算
   * @param target_speed 目標ボール初速度 [m/s]
   * @return 必要なキック力 [0.0-1.0]
   */
  [[nodiscard]] auto calculateStraightKickPower(double target_speed) const -> double;

  /**
   * @brief 目標飛行距離からチップキック力を計算
   * @param target_distance 目標飛行距離 [m]
   * @return 必要なキック力 [0.0-1.0]
   */
  [[nodiscard]] auto calculateChipKickPower(double target_distance) const -> double;

  // ===== BallPhysicsModel連携による高度な計算 =====

  /**
   * @brief 目標停止距離からストレートキック力を計算
   * @param target_stop_distance 目標停止距離 [m]
   * @return 必要なキック力 [0.0-1.0]
   * @throws std::runtime_error BallPhysicsModelが設定されていない場合
   */
  [[nodiscard]] auto calculateKickPowerForStopDistance(double target_stop_distance) const -> double;

  /**
   * @brief 指定されたキック力でのボール停止距離を予測
   * @param kick_power キック力 [0.0-1.0]
   * @return 予測される停止距離 [m]
   * @throws std::runtime_error BallPhysicsModelが設定されていない場合
   */
  [[nodiscard]] auto predictStopDistance(double kick_power) const -> double;

  /**
   * @brief チップキックの着地後停止距離を予測
   * @param kick_power キック力 [0.0-1.0]
   * @return 着地後の停止距離 [m]
   * @throws std::runtime_error BallPhysicsModelが設定されていない場合
   */
  [[nodiscard]] auto predictChipKickTotalDistance(double kick_power) const -> double;

  // ===== BallPhysicsModel管理 =====

  /**
   * @brief BallPhysicsModelを設定
   * @param ball_physics BallPhysicsModelの共有ポインタ
   */
  auto setBallPhysicsModel(std::shared_ptr<BallPhysicsModel> ball_physics) -> void;

  /**
   * @brief BallPhysicsModelを取得
   * @return BallPhysicsModelの共有ポインタ（未設定の場合はnullptr）
   */
  [[nodiscard]] auto getBallPhysicsModel() const -> std::shared_ptr<BallPhysicsModel>;

  // ===== 設定アクセサ =====

  /**
   * @brief 現在の設定を取得
   * @return 設定の定数参照
   */
  [[nodiscard]] auto getConfig() const -> const Config &;

  /**
   * @brief 設定を更新
   * @param config 新しい設定
   */
  auto setConfig(const Config & config) -> void;

  // ===== 検証関数 =====

  /**
   * @brief 設定の妥当性を検証
   * @return 設定が有効な場合はtrue
   */
  [[nodiscard]] auto validateConfig() const -> bool;

  /**
   * @brief キック力の有効範囲を検証
   * @param kick_power キック力
   * @return 有効な場合はtrue
   */
  [[nodiscard]] static auto isValidKickPower(double kick_power) -> bool;

private:
  Config config_;                                         ///< キッカーモデルの設定
  std::shared_ptr<BallPhysicsModel> ball_physics_model_;  ///< ボール物理モデル

  // ===== 内部ヘルパー関数 =====

  /**
   * @brief 線形補間による値の計算
   * @param x 補間する値
   * @param x_array X軸の配列
   * @param y_array Y軸の配列
   * @return 補間された値
   * @throws std::runtime_error 配列サイズが不一致または空の場合
   */
  [[nodiscard]] auto getLinearInterpolation(
    double x, const std::vector<double> & x_array, const std::vector<double> & y_array) const
    -> double;

  /**
   * @brief 逆線形補間による値の計算（Y値からX値を求める）
   * @param y 逆補間するY値
   * @param x_array X軸の配列
   * @param y_array Y軸の配列
   * @return 逆補間されたX値
   * @throws std::runtime_error 配列サイズが不一致または値が範囲外の場合
   */
  [[nodiscard]] auto getInverseLinearInterpolation(
    double y, const std::vector<double> & x_array, const std::vector<double> & y_array) const
    -> double;

  /**
   * @brief 設定配列の妥当性を検証
   * @param x_array X軸配列
   * @param y_array Y軸配列
   * @param array_name 配列名（エラーメッセージ用）
   * @return 有効な場合はtrue
   */
  [[nodiscard]] auto validateArrays(
    const std::vector<double> & x_array, const std::vector<double> & y_array,
    const std::string & array_name) const -> bool;
};

// ===== ファクトリー関数 =====

/**
 * @brief デフォルト設定でKickerModelを作成
 * @return KickerModelの共有ポインタ
 */
[[nodiscard]] auto createDefaultKickerModel() -> std::shared_ptr<KickerModel>;

/**
 * @brief YAML設定ファイルからKickerModelを作成
 * @param yaml_file_path YAMLファイルのパス
 * @return KickerModelの共有ポインタ
 */
[[nodiscard]] auto createKickerModelFromYAML(const std::string & yaml_file_path)
  -> std::shared_ptr<KickerModel>;

/**
 * @brief BallPhysicsModelと統合されたKickerModelを作成
 * @param yaml_file_path YAMLファイルのパス
 * @param ball_physics BallPhysicsModelの共有ポインタ
 * @return KickerModelの共有ポインタ
 */
[[nodiscard]] auto createIntegratedKickerModel(
  const std::string & yaml_file_path, std::shared_ptr<BallPhysicsModel> ball_physics)
  -> std::shared_ptr<KickerModel>;

}  // namespace crane

#endif  // CRANE_PHYSICS__KICKER_MODEL_HPP_