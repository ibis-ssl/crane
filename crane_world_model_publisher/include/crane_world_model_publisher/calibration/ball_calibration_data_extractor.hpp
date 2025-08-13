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
    double max_trajectory_gap = 0.1;          // 軌道データの最大時間間隔[s]
    double min_trajectory_duration = 0.5;     // 最小軌道継続時間[s]
    size_t min_trajectory_points = 10;        // 最小軌道点数
    size_t min_consistency_frames = 3;        // 速度変化の一貫性チェックフレーム数
    bool extract_straight_kicks_only = true;  // ストレートキックのみ抽出
    std::vector<uint8_t> target_robot_ids;    // 対象ロボットID（空の場合は全ロボット）
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
    double avg_trajectory_duration = 0.0;
    double avg_trajectory_points = 0.0;
    std::vector<uint8_t> active_robot_ids;
  };

  auto getLastExtractionStats() const -> const ExtractionStats &;

  /**
   * @brief キックイベント前後のボール軌道を可視化
   * @param ball_data 全ボールデータ
   * @param kick_events 検出されたキックイベント
   * @param output_prefix 出力ファイル名のプレフィックス
   * @param rosbag_path 使用したROSBAGのパス（出力先ディレクトリ決定用）
   */
  auto visualizeKickEvents(
    const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data,
    const std::vector<std::pair<rclcpp::Time, Point>> & kick_events,
    const std::string & output_prefix = "kick_event",
    const std::string & rosbag_path = "") -> void;

  /**
   * @brief テレポート（瞬間移動）イベントの検出と除外
   * @param ball_data 全ボールデータ
   * @param kick_events キックイベント候補
   * @return テレポートではない正当なキックイベント
   */
  auto filterTeleportEvents(
    const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data,
    const std::vector<std::pair<rclcpp::Time, Point>> & kick_events)
    -> std::vector<std::pair<rclcpp::Time, Point>>;

private:
  ExtractorConfig config_;
  ExtractionStats last_stats_;
  
  // 可視化のために保存される最後のキックイベント
  std::vector<std::pair<rclcpp::Time, Point>> last_detected_kick_events_;
  std::vector<std::pair<rclcpp::Time, Ball>> last_ball_data_;

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
   * @brief 軌道データの品質チェック
   */
  auto validateTrajectoryQuality(const std::vector<Ball> & trajectory) -> bool;

  /**
   * @brief ボール速度の物理的妥当性チェック
   */
  auto validateBallPhysics(
    const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data, size_t index) -> bool;

  /**
   * @brief 速度変化の一貫性チェック
   */
  auto validateSpeedConsistency(
    const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data, size_t kick_index) -> bool;

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
};

}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__BALL_CALIBRATION_DATA_EXTRACTOR_HPP_
