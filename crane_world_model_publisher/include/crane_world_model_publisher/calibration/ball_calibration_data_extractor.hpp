// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__BALL_CALIBRATION_DATA_EXTRACTOR_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__BALL_CALIBRATION_DATA_EXTRACTOR_HPP_

#include <crane_msgs/msg/robot_command.hpp>
#include <crane_physics/ball_info.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <vector>

namespace crane
{

/**
 * @brief キック時のデータポイント構造体
 */
struct KickDataPoint
{
  rclcpp::Time timestamp;        // キック発生時刻
  Point kick_position;           // キック発生位置
  uint8_t kicker_id;             // キッカーロボットID
  bool is_our_robot;             // 自チームのロボットかどうか
  double kick_power;             // キックパワー（0.0-1.0）
  bool is_chip_kick;             // チップキックかストレートキックか
  Ball initial_ball_state;       // キック直後のボール状態
  std::vector<Ball> trajectory;  // キック後のボール軌道データ

  // 軌道品質評価
  size_t valid_trajectory_points;  // 有効な軌道点数
  double trajectory_duration;      // 軌道継続時間[秒]
  double max_speed;                // 軌道中の最大速度
};

/**
 * @brief ROSBAGからキャリブレーション用データを抽出するクラス
 */
class BallCalibrationDataExtractor
{
public:
  /**
   * @brief コンストラクタ
   */
  BallCalibrationDataExtractor();

  /**
   * @brief ROSBAGファイルからキックデータを抽出
   * @param bag_path ROSBAGファイルのパス
   * @return 抽出されたキックデータポイントのベクター
   */
  auto extractKickDataFromBag(const std::string & bag_path) -> std::vector<KickDataPoint>;

  /**
   * @brief 抽出フィルタ設定
   */
  struct ExtractorConfig
  {
    double min_kick_speed = 0.5;              // 最小キック速度閾値[m/s]
    double max_kick_speed = 30.0;             // 最大キック速度閾値[m/s]（物理的上限）
    double max_acceleration = 500.0;          // 最大加速度[m/s²]（物理的上限）
    double max_pre_kick_speed = 0.05;         // キック前最大速度閾値[m/s]（ほぼ静止状態）
    size_t required_stationary_frames = 10;   // 必要な静止フレーム数
    size_t min_trajectory_points = 10;        // 最小軌道点数
    bool extract_straight_kicks_only = true;  // ストレートキックのみ抽出
  };

  /**
   * @brief 抽出設定の更新
   */
  auto setConfig(const ExtractorConfig & config) -> void;

  /**
   * @brief 抽出されたデータの統計情報を取得
   */
  struct ExtractionStats
  {
    size_t total_kick_events = 0;
    size_t valid_kick_events = 0;
    size_t straight_kick_count = 0;
    size_t chip_kick_count = 0;
    size_t trajectories_ended_at_boundary = 0;  // フィールド境界で終了した軌道数
    double avg_trajectory_duration = 0.0;
    double avg_trajectory_points = 0.0;
    std::vector<uint8_t> active_robot_ids;
  };

  auto getLastExtractionStats() const -> const ExtractionStats &;

  /**
   * @brief キックイベント前後のボール軌道をキック力情報付きで可視化
   * @param ball_data 全ボールデータ
   * @param kick_data_points キック力情報を含むキックデータポイント
   * @param output_prefix 出力ファイル名のプレフィックス
   * @param rosbag_path 使用したROSBAGのパス（出力先ディレクトリ決定用）
   */
  auto visualizeKickEventsWithPower(
    const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data,
    const std::vector<KickDataPoint> & kick_data_points,
    const std::string & output_prefix = "kick_event", const std::string & rosbag_path = "") -> void;

private:
  ExtractorConfig config_;
  ExtractionStats last_stats_;
  size_t temp_boundary_ended_count_ = 0;  // 一時的な境界終了カウント
  size_t teleport_detection_count_ = 0;   // テレポート検出回数
  bool teleport_detection_disabled_ = false; // テレポート検出無効化フラグ
  
  // フィールド情報（world_modelから取得）
  double field_length_half_ = 6.0;  // デフォルト値：SSL規格フィールド長の半分 [m]
  double field_width_half_ = 4.5;   // デフォルト値：SSL規格フィールド幅の半分 [m]
  bool field_info_updated_ = false; // フィールド情報が更新されたかのフラグ

  /**
   * @brief ボールデータとロボットコマンドの時系列マッチング
   */
  auto matchBallTrajectoryWithKicks(
    const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data,
    const std::vector<std::pair<rclcpp::Time, crane_msgs::msg::RobotCommand>> & command_data)
    -> std::vector<KickDataPoint>;

  /**
   * @brief キックイベントの検出
   */
  auto detectKickEvents(const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data)
    -> std::vector<std::pair<rclcpp::Time, Point>>;

  /**
   * @brief ボール速度の物理的妥当性チェック
   */
  auto validateBallPhysics(
    const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data, size_t index) -> bool;

  /**
   * @brief キックパワー情報の抽出
   */
  auto extractKickPower(
    const rclcpp::Time & kick_time,
    const std::vector<std::pair<rclcpp::Time, crane_msgs::msg::RobotCommand>> & command_data,
    uint8_t robot_id) -> std::pair<double, bool>;  // power, is_chip

  /**
   * @brief 抽出統計情報の更新
   */
  auto updateExtractionStats(const std::vector<KickDataPoint> & kick_points) -> void;

  /**
   * @brief フィールド境界判定
   * @param position 判定する位置
   * @param offset フィールド境界からのオフセット距離[m]
   * @return フィールド内（オフセット領域を含む）の場合true
   */
  auto isFieldInside(const Point & position, double offset = 0.0) const -> bool;

  /**
   * @brief world_modelメッセージからフィールド情報を更新
   * @param world_model_msg world_modelメッセージ
   */
  auto updateFieldInfo(const crane_msgs::msg::WorldModel & world_model_msg) -> void;

  /**
   * @brief 位置データに移動平均フィルタを適用（速度計算前のノイズ除去）
   * @param ball_data 過去のボールデータ
   * @param current_pos 現在の位置
   * @return フィルタ適用後の位置
   */
  auto applySmoothingFilter(
    const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data, const Point & current_pos) const
    -> Point;

  /**
   * @brief スカラー値に移動平均フィルタを適用
   * @param ball_data 過去のボールデータ
   * @param current_value 現在の値
   * @return フィルタ適用後の値
   */
  auto applySmoothingFilterScalar(
    const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data, double current_value) const
    -> double;

  /**
   * @brief 速度の妥当性チェックと外れ値フィルタリング
   * @param ball_data 過去のボールデータ
   * @param raw_velocity 計算された生速度
   * @param dt 時間間隔
   * @return (妥当性フラグ, フィルタ後速度)
   */
  auto validateAndFilterVelocity(
    const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data, const Point & raw_velocity,
    double dt) const -> std::pair<bool, Point>;

  /**
   * @brief Z方向速度の妥当性チェックと外れ値フィルタリング
   * @param ball_data 過去のボールデータ
   * @param raw_velocity 計算された生Z方向速度
   * @param dt 時間間隔
   * @return (妥当性フラグ, フィルタ後Z方向速度)
   */
  auto validateAndFilterVelocityScalar(
    const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data, double raw_velocity,
    double dt) const -> std::pair<bool, double>;

  /**
   * @brief キック力情報付きの可視化用Pythonスクリプトを生成
   * @param json_data_file JSONデータファイルのパス
   * @param output_dir 出力ディレクトリ
   * @param output_prefix 出力ファイル名のプレフィックス
   * @param event_idx イベントインデックス
   */
  auto generateVisualizationPlotWithPower(
    const std::string & json_data_file, const std::string & output_dir,
    const std::string & output_prefix, size_t event_idx) -> void;
};

}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__BALL_CALIBRATION_DATA_EXTRACTOR_HPP_
