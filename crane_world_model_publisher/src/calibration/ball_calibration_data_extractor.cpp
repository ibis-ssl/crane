// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/calibration/ball_calibration_data_extractor.hpp"
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <crane_msgs/msg/ball_info.hpp>
#include <algorithm>
#include <numeric>
#include <set>
#include <limits>

namespace crane
{

BallCalibrationDataExtractor::BallCalibrationDataExtractor()
{
  // デフォルト設定
}

auto BallCalibrationDataExtractor::setConfig(const ExtractorConfig & config) -> void
{
  config_ = config;
}

auto BallCalibrationDataExtractor::getLastExtractionStats() const -> const ExtractionStats &
{
  return last_stats_;
}

auto BallCalibrationDataExtractor::extractKickDataFromBag(const std::string & bag_path)
  -> std::vector<KickDataPoint>
{
  std::vector<KickDataPoint> kick_data_points;
  
  try {
    // ROSBAGリーダーの初期化
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    
    reader.open(storage_options);
    
    // データ格納用コンテナ
    std::vector<std::pair<rclcpp::Time, Ball>> ball_data;
    std::vector<std::pair<rclcpp::Time, crane_msgs::msg::RobotCommand>> command_data;
    
    // ROSBAGからメッセージを読み込み
    while (reader.has_next()) {
      auto bag_message = reader.read_next();
      
      // ボール情報の処理
      if (bag_message->topic_name == "/ball_info") {
        auto ball_msg = std::make_shared<crane_msgs::msg::BallInfo>();
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        rclcpp::Serialization<crane_msgs::msg::BallInfo> serialization;
        serialization.deserialize_message(&serialized_msg, ball_msg.get());
        
        Ball ball;
        ball.fromMsg(*ball_msg);
        ball_data.emplace_back(rclcpp::Time(bag_message->time_stamp), ball);
      }
      
      // ロボットコマンドの処理
      else if (bag_message->topic_name.find("/robot_command") != std::string::npos) {
        auto cmd_msg = std::make_shared<crane_msgs::msg::RobotCommand>();
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        rclcpp::Serialization<crane_msgs::msg::RobotCommand> serialization;
        serialization.deserialize_message(&serialized_msg, cmd_msg.get());
        
        // キック関連コマンドのみ記録
        if (cmd_msg->kick_power > 0.0) {
          command_data.emplace_back(rclcpp::Time(bag_message->time_stamp), *cmd_msg);
        }
      }
    }
    
    reader.close();
    
    // データの時系列ソート
    std::sort(ball_data.begin(), ball_data.end(),
              [](const auto & a, const auto & b) { return a.first < b.first; });
    std::sort(command_data.begin(), command_data.end(),
              [](const auto & a, const auto & b) { return a.first < b.first; });
    
    // キックデータとボール軌道のマッチング
    kick_data_points = matchBallTrajectoryWithKicks(ball_data, command_data);
    
    // 統計情報の更新
    updateExtractionStats(kick_data_points);
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("BallCalibrationDataExtractor"),
                 "ROSBAGからのデータ抽出に失敗: %s", e.what());
  }
  
  return kick_data_points;
}

auto BallCalibrationDataExtractor::matchBallTrajectoryWithKicks(
  const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data,
  const std::vector<std::pair<rclcpp::Time, crane_msgs::msg::RobotCommand>> & command_data
) -> std::vector<KickDataPoint>
{
  std::vector<KickDataPoint> kick_points;
  
  // キックイベントの検出
  auto kick_events = detectKickEvents(ball_data);
  
  for (const auto & [kick_time, kick_pos] : kick_events) {
    // 最も近い時刻のロボットコマンドを検索
    auto cmd_it = std::min_element(
      command_data.begin(), command_data.end(),
      [&kick_time](const auto & a, const auto & b) {
        auto diff_a = std::abs((a.first - kick_time).seconds());
        auto diff_b = std::abs((b.first - kick_time).seconds());
        return diff_a < diff_b;
      });
    
    if (cmd_it != command_data.end()) {
      const auto & [cmd_time, cmd_msg] = *cmd_it;
      
      // 時間差が許容範囲内かチェック
      double time_diff = std::abs((cmd_time - kick_time).seconds());
      if (time_diff <= 0.5) { // 500ms以内
        
        // 対象ロボットIDフィルタ
        if (!config_.target_robot_ids.empty() && 
            std::find(config_.target_robot_ids.begin(), config_.target_robot_ids.end(), 
                      cmd_msg.robot_id) == config_.target_robot_ids.end()) {
          continue;
        }
        
        // ストレートキックのみフィルタ
        if (config_.extract_straight_kicks_only && cmd_msg.chip_enable) {
          continue;
        }
        
        // キックデータポイントの作成
        KickDataPoint kick_point;
        kick_point.timestamp = kick_time;
        kick_point.kick_position = kick_pos;
        kick_point.kicker_id = cmd_msg.robot_id;
        kick_point.is_our_robot = true; // 自チームのコマンドのみ処理
        kick_point.kick_power = cmd_msg.kick_power;
        kick_point.is_chip_kick = cmd_msg.chip_enable;
        
        // キック後の軌道データを抽出
        auto trajectory_start = std::upper_bound(
          ball_data.begin(), ball_data.end(), 
          std::make_pair(kick_time, Ball{}),
          [](const auto & a, const auto & b) { return a.first < b.first; });
        
        // 軌道データの収集（最大3秒間または停止まで）
        for (auto it = trajectory_start; it != ball_data.end(); ++it) {
          const auto & [time, ball] = *it;
          double elapsed = (time - kick_time).seconds();
          
          if (elapsed > 3.0) break; // 3秒でタイムアウト
          if (ball.vel.norm() < 0.1 && elapsed > 0.5) break; // 停止判定
          
          kick_point.trajectory.push_back(ball);
        }
        
        // 初期状態の設定
        if (!kick_point.trajectory.empty()) {
          kick_point.initial_ball_state = kick_point.trajectory.front();
        }
        
        // 軌道品質の評価
        kick_point.valid_trajectory_points = kick_point.trajectory.size();
        kick_point.trajectory_duration = kick_point.trajectory.empty() ? 0.0 :
          (kick_time - kick_time).seconds(); // 実際の継続時間を計算
        kick_point.max_speed = 0.0;
        for (const auto & ball : kick_point.trajectory) {
          kick_point.max_speed = std::max(kick_point.max_speed, ball.vel.norm());
        }
        
        // 品質チェック
        if (validateTrajectoryQuality(kick_point.trajectory) &&
            kick_point.max_speed >= config_.min_kick_speed) {
          kick_points.push_back(kick_point);
        }
      }
    }
  }
  
  return kick_points;
}

auto BallCalibrationDataExtractor::detectKickEvents(
  const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data
) -> std::vector<std::pair<rclcpp::Time, Point>>
{
  std::vector<std::pair<rclcpp::Time, Point>> kick_events;
  
  if (ball_data.size() < 3) return kick_events;
  
  // 速度変化によるキックイベント検出
  for (size_t i = 1; i < ball_data.size() - 1; ++i) {
    const auto & [prev_time, prev_ball] = ball_data[i-1];
    const auto & [curr_time, curr_ball] = ball_data[i];
    const auto & [next_time, next_ball] = ball_data[i+1];
    
    double prev_speed = prev_ball.vel.norm();
    double curr_speed = curr_ball.vel.norm();
    double next_speed = next_ball.vel.norm();
    
    // 急激な速度増加を検出
    double speed_increase = curr_speed - prev_speed;
    double speed_ratio = prev_speed > 0.1 ? speed_increase / prev_speed : speed_increase;
    
    if (speed_increase > 1.0 && speed_ratio > 2.0 && curr_speed > config_.min_kick_speed) {
      kick_events.emplace_back(curr_time, curr_ball.pos);
    }
  }
  
  return kick_events;
}

auto BallCalibrationDataExtractor::validateTrajectoryQuality(const std::vector<Ball> & trajectory)
  -> bool
{
  if (trajectory.size() < config_.min_trajectory_points) {
    return false;
  }
  
  // 軌道データの連続性チェック
  double total_duration = 0.0;
  for (size_t i = 1; i < trajectory.size(); ++i) {
    // 時間間隔をチェック（実際の実装では時刻情報が必要）
    // ここでは簡略化
  }
  
  return total_duration >= config_.min_trajectory_duration;
}

auto BallCalibrationDataExtractor::extractKickPower(
  const rclcpp::Time & kick_time,
  const std::vector<std::pair<rclcpp::Time, crane_msgs::msg::RobotCommand>> & command_data,
  uint8_t robot_id
) -> std::pair<double, bool>
{
  // 指定されたロボットIDとキック時刻に最も近いコマンドを検索
  auto best_it = command_data.end();
  double min_time_diff = std::numeric_limits<double>::max();
  
  for (auto it = command_data.begin(); it != command_data.end(); ++it) {
    if (it->second.robot_id == robot_id) {
      double time_diff = std::abs((it->first - kick_time).seconds());
      if (time_diff < min_time_diff && time_diff <= 0.5) { // 500ms以内
        min_time_diff = time_diff;
        best_it = it;
      }
    }
  }
  
  if (best_it != command_data.end()) {
    return {best_it->second.kick_power, best_it->second.chip_enable};
  }
  
  return {0.0, false};
}

auto BallCalibrationDataExtractor::updateExtractionStats(const std::vector<KickDataPoint> & kick_points)
  -> void
{
  last_stats_ = ExtractionStats{};
  last_stats_.valid_kick_events = kick_points.size();
  
  if (!kick_points.empty()) {
    // 統計計算
    double total_duration = 0.0;
    double total_points = 0.0;
    std::set<uint8_t> robot_ids;
    
    for (const auto & kick_point : kick_points) {
      total_duration += kick_point.trajectory_duration;
      total_points += kick_point.valid_trajectory_points;
      robot_ids.insert(kick_point.kicker_id);
      
      if (kick_point.is_chip_kick) {
        last_stats_.chip_kick_count++;
      } else {
        last_stats_.straight_kick_count++;
      }
    }
    
    last_stats_.avg_trajectory_duration = total_duration / kick_points.size();
    last_stats_.avg_trajectory_points = total_points / kick_points.size();
    last_stats_.active_robot_ids.assign(robot_ids.begin(), robot_ids.end());
  }
}


} // namespace crane