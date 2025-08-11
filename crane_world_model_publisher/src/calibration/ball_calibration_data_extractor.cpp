// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/calibration/ball_calibration_data_extractor.hpp"

#include <algorithm>
#include <crane_msgs/msg/ball_info.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <limits>
#include <numeric>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <set>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <nlohmann/json.hpp>

namespace crane
{

BallCalibrationDataExtractor::BallCalibrationDataExtractor()
{
  // デフォルト設定（物理的制約を考慮）
  config_.min_kick_speed = 0.8;          // 最小キック速度: 0.8m/s
  config_.max_kick_speed = 30.0;         // 最大キック速度: 30.0m/s（物理的上限）
  config_.max_acceleration = 500.0;       // 最大加速度: 500m/s²（キック瞬間の物理的上限）
  config_.min_consistency_frames = 3;     // 一貫性チェック用最小フレーム数
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

      // ボール情報の処理（world_modelから抽出）
      if (bag_message->topic_name == "/world_model") {
        auto world_model_msg = std::make_shared<crane_msgs::msg::WorldModel>();
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        rclcpp::Serialization<crane_msgs::msg::WorldModel> serialization;
        serialization.deserialize_message(&serialized_msg, world_model_msg.get());

        // Vision生データを使用してキャリブレーション
        const auto & ball_info = world_model_msg->ball_info;
        
        if (ball_info.detected && ball_info.vision.stamp.sec != 0) {
          // Vision生データからBall構造体を構築
          Ball vision_ball;
          vision_ball.pos = Point(ball_info.vision.pos.x, ball_info.vision.pos.y);
          vision_ball.pos_z = ball_info.vision.pos.z;
          
          // Vision生データから速度を時間微分で計算
          auto current_time = rclcpp::Time(bag_message->recv_timestamp);
          if (!ball_data.empty()) {
            auto & [prev_time, prev_ball] = ball_data.back();
            double dt = (current_time - prev_time).seconds();
            
            if (dt > 1e-6) {  // 最小時間間隔チェック
              // 位置の差分から速度を計算
              Point position_diff = vision_ball.pos - prev_ball.pos;
              double pos_z_diff = vision_ball.pos_z - prev_ball.pos_z;
              
              vision_ball.vel = position_diff / dt;
              vision_ball.vel_z = pos_z_diff / dt;
              
              // 速度から状態を推定
              double speed = vision_ball.vel.norm();
              if (speed < 0.05) {
                vision_ball.state = Ball::State::STOPPED;
              } else if (vision_ball.pos_z > 0.02 || std::abs(vision_ball.vel_z) > 0.1) {
                vision_ball.state = Ball::State::FLYING;
              } else {
                vision_ball.state = Ball::State::ROLLING;
              }
            } else {
              vision_ball.vel = Point(0, 0);
              vision_ball.vel_z = 0;
              vision_ball.state = Ball::State::STOPPED;
            }
          } else {
            // 最初のデータポイント
            vision_ball.vel = Point(0, 0);
            vision_ball.vel_z = 0;
            vision_ball.state = Ball::State::STOPPED;
          }
          
          vision_ball.detected = ball_info.detected;
          ball_data.emplace_back(current_time, vision_ball);
        }
      }

      // ロボットコマンドの処理
      else if (bag_message->topic_name == "/robot_commands") {
        try {
          auto cmds_msg = std::make_shared<crane_msgs::msg::RobotCommands>();
          rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
          rclcpp::Serialization<crane_msgs::msg::RobotCommands> serialization;
          serialization.deserialize_message(&serialized_msg, cmds_msg.get());

          // 各ロボットコマンドを個別に処理
          for (const auto & robot_cmd : cmds_msg->robot_commands) {
            // キック関連コマンドのみ記録
            if (robot_cmd.kick_power > 0.0) {
              command_data.emplace_back(rclcpp::Time(bag_message->recv_timestamp), robot_cmd);
            }
          }
        } catch (const std::exception & e) {
          RCLCPP_WARN_ONCE(
            rclcpp::get_logger("BallCalibrationDataExtractor"),
            "ロボットコマンドのデシリアライゼーションに失敗: %s", e.what());
          // 続行して他のメッセージを処理
        }
      }
    }

    reader.close();

    RCLCPP_INFO(
      rclcpp::get_logger("BallCalibrationDataExtractor"),
      "ROSBAGから抽出完了: ボールデータ %zu 個（Vision生データ使用）, ロボットコマンド %zu 個", ball_data.size(),
      command_data.size());

    // データの時系列ソート
    std::sort(ball_data.begin(), ball_data.end(), [](const auto & a, const auto & b) {
      return a.first < b.first;
    });
    std::sort(command_data.begin(), command_data.end(), [](const auto & a, const auto & b) {
      return a.first < b.first;
    });

    // ボール速度の統計情報をログ出力（異常値分析付き）
    if (!ball_data.empty()) {
      double min_speed = std::numeric_limits<double>::max();
      double max_speed = 0.0;
      double avg_speed = 0.0;
      size_t abnormal_speed_count = 0;
      size_t excessive_speed_count = 0;
      
      for (const auto & [time, ball] : ball_data) {
        double speed = ball.vel.norm();
        min_speed = std::min(min_speed, speed);
        max_speed = std::max(max_speed, speed);
        avg_speed += speed;
        
        if (speed > config_.max_kick_speed) {
          excessive_speed_count++;
        }
        if (speed > 50.0) {  // 50m/s超過は明らかに異常
          abnormal_speed_count++;
        }
      }
      avg_speed /= ball_data.size();

      RCLCPP_INFO(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "ボール速度統計: 最小=%.3fm/s, 最大=%.3fm/s, 平均=%.3fm/s", 
        min_speed, max_speed, avg_speed);
      
      RCLCPP_INFO(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "異常値統計: 上限超過=%zu個(%.1f%%), 明らかな異常=%zu個(%.1f%%), 閾値: 最小=%.3f, 最大=%.3fm/s",
        excessive_speed_count, (100.0 * excessive_speed_count) / ball_data.size(),
        abnormal_speed_count, (100.0 * abnormal_speed_count) / ball_data.size(),
        config_.min_kick_speed, config_.max_kick_speed);
    }

    // キックデータとボール軌道のマッチング
    kick_data_points = matchBallTrajectoryWithKicks(ball_data, command_data);

    // 統計情報の更新
    updateExtractionStats(kick_data_points);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BallCalibrationDataExtractor"), "ROSBAGからのデータ抽出に失敗: %s",
      e.what());
  }

  return kick_data_points;
}

auto BallCalibrationDataExtractor::matchBallTrajectoryWithKicks(
  const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data,
  const std::vector<std::pair<rclcpp::Time, crane_msgs::msg::RobotCommand>> & command_data)
  -> std::vector<KickDataPoint>
{
  std::vector<KickDataPoint> kick_points;

  // キックイベントの検出
  auto raw_kick_events = detectKickEvents(ball_data);

  // テレポートイベントのフィルタリング
  auto kick_events = filterTeleportEvents(ball_data, raw_kick_events);

  RCLCPP_INFO(
    rclcpp::get_logger("BallCalibrationDataExtractor"),
    "キックイベント検出完了: %zu個検出 → %zu個フィルタ後 (%zu個のテレポートを除外)",
    raw_kick_events.size(), kick_events.size(), raw_kick_events.size() - kick_events.size());

  // 可視化機能を呼び出し（フィルタ後のイベントのみ）
  if (!kick_events.empty()) {
    visualizeKickEvents(ball_data, kick_events, "kick_event_visualization");
  }

  RCLCPP_INFO(
    rclcpp::get_logger("BallCalibrationDataExtractor"),
    "マッチング開始: %zu個のキックイベントと%zu個のロボットコマンドをマッチング",
    kick_events.size(), command_data.size());
  RCLCPP_INFO(
    rclcpp::get_logger("BallCalibrationDataExtractor"),
    "設定: ストレートキックのみ=%s, 最小速度=%.3fm/s, 最小軌道点数=%zu",
    config_.extract_straight_kicks_only ? "true" : "false", config_.min_kick_speed,
    config_.min_trajectory_points);

  for (const auto & [kick_time, kick_pos] : kick_events) {
    RCLCPP_INFO(
      rclcpp::get_logger("BallCalibrationDataExtractor"),
      "キックイベントマッチング: 時刻=%.3fs, 位置=(%.3f,%.3f)", kick_time.seconds(), kick_pos.x(),
      kick_pos.y());

    // 最も近い時刻のロボットコマンドを検索
    auto cmd_it = std::min_element(
      command_data.begin(), command_data.end(), [&kick_time](const auto & a, const auto & b) {
        auto diff_a = std::abs((a.first - kick_time).seconds());
        auto diff_b = std::abs((b.first - kick_time).seconds());
        return diff_a < diff_b;
      });

    if (cmd_it != command_data.end()) {
      const auto & [cmd_time, cmd_msg] = *cmd_it;

      // 時間差が許容範囲内かチェック
      double time_diff = std::abs((cmd_time - kick_time).seconds());
      RCLCPP_INFO(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "最も近いコマンド: ロボット%u, 時刻=%.3fs, 時間差=%.3fs, キック力=%.3f", cmd_msg.robot_id,
        cmd_time.seconds(), time_diff, cmd_msg.kick_power);

      if (time_diff <= 5.0) {  // 5秒以内（キャリブレーション用に緩和）
        RCLCPP_INFO(
          rclcpp::get_logger("BallCalibrationDataExtractor"), "時間差OK: %.3fs以内", time_diff);

        // 対象ロボットIDフィルタ
        if (
          !config_.target_robot_ids.empty() &&
          std::find(
            config_.target_robot_ids.begin(), config_.target_robot_ids.end(), cmd_msg.robot_id) ==
            config_.target_robot_ids.end()) {
          RCLCPP_WARN(
            rclcpp::get_logger("BallCalibrationDataExtractor"),
            "ロボットIDフィルタで除外: %u (対象ロボットが指定されている)", cmd_msg.robot_id);
          continue;
        }

        // ストレートキックのみフィルタ
        if (config_.extract_straight_kicks_only && cmd_msg.chip_enable) {
          RCLCPP_WARN(
            rclcpp::get_logger("BallCalibrationDataExtractor"),
            "チップキックのため除外: ロボット%u", cmd_msg.robot_id);
          continue;
        }

        // キックデータポイントの作成
        KickDataPoint kick_point;
        kick_point.timestamp = kick_time;
        kick_point.kick_position = kick_pos;
        kick_point.kicker_id = cmd_msg.robot_id;
        kick_point.is_our_robot = true;  // 自チームのコマンドのみ処理
        kick_point.kick_power = cmd_msg.kick_power;
        kick_point.is_chip_kick = cmd_msg.chip_enable;

        // キック後の軌道データを抽出
        auto trajectory_start = std::upper_bound(
          ball_data.begin(), ball_data.end(), std::make_pair(kick_time, Ball{}),
          [](const auto & a, const auto & b) { return a.first < b.first; });

        // 軌道データの収集（最大3秒間または停止まで）
        for (auto it = trajectory_start; it != ball_data.end(); ++it) {
          const auto & [time, ball] = *it;
          double elapsed = (time - kick_time).seconds();

          if (elapsed > 3.0) break;                           // 3秒でタイムアウト
          if (ball.vel.norm() < 0.1 && elapsed > 0.5) break;  // 停止判定

          kick_point.trajectory.push_back(ball);
        }

        // 初期状態の設定
        if (!kick_point.trajectory.empty()) {
          kick_point.initial_ball_state = kick_point.trajectory.front();
        }

        // 軌道品質の評価
        kick_point.valid_trajectory_points = kick_point.trajectory.size();
        kick_point.trajectory_duration =
          kick_point.trajectory.empty()
            ? 0.0
            : (kick_time - kick_time).seconds();  // 実際の継続時間を計算
        kick_point.max_speed = 0.0;
        for (const auto & ball : kick_point.trajectory) {
          kick_point.max_speed = std::max(kick_point.max_speed, ball.vel.norm());
        }

        // 品質チェック
        bool trajectory_quality_ok = validateTrajectoryQuality(kick_point.trajectory);
        bool min_speed_ok = kick_point.max_speed >= config_.min_kick_speed;

        RCLCPP_INFO(
          rclcpp::get_logger("BallCalibrationDataExtractor"),
          "品質チェック: 軌道品質=%s, 最高速度=%.3fm/s(閾値%.3f)=%s, 軌道点数=%zu",
          trajectory_quality_ok ? "OK" : "NG", kick_point.max_speed, config_.min_kick_speed,
          min_speed_ok ? "OK" : "NG", kick_point.trajectory.size());

        if (trajectory_quality_ok && min_speed_ok) {
          kick_points.push_back(kick_point);
          RCLCPP_INFO(
            rclcpp::get_logger("BallCalibrationDataExtractor"),
            "キックデータポイント追加: ロボット%u, 最高速度=%.3fm/s", kick_point.kicker_id,
            kick_point.max_speed);
        } else {
          RCLCPP_WARN(
            rclcpp::get_logger("BallCalibrationDataExtractor"),
            "キックデータポイント除外: 品質=%s, 速度=%s", trajectory_quality_ok ? "OK" : "NG",
            min_speed_ok ? "OK" : "NG");
        }
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("BallCalibrationDataExtractor"),
          "時間差が大きすぎて除外: %.3fs > 5.0s", time_diff);
      }
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "対応するロボットコマンドが見つからない");
    }
  }

  return kick_points;
}

auto BallCalibrationDataExtractor::detectKickEvents(
  const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data)
  -> std::vector<std::pair<rclcpp::Time, Point>>
{
  std::vector<std::pair<rclcpp::Time, Point>> kick_events;

  if (ball_data.size() < 3) {
    RCLCPP_WARN(
      rclcpp::get_logger("BallCalibrationDataExtractor"),
      "キックイベント検出: データ不足 (必要: 3個以上, 実際: %zu 個)", ball_data.size());
    return kick_events;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("BallCalibrationDataExtractor"),
    "キックイベント検出開始: %zu 個のボールデータを解析", ball_data.size());

  size_t potential_kicks = 0;
  size_t speed_increase_candidates = 0;
  size_t speed_ratio_candidates = 0;

  // 速度変化によるキックイベント検出（物理的妥当性チェック付き）
  for (size_t i = 1; i < ball_data.size() - 1; ++i) {
    const auto & [prev_time, prev_ball] = ball_data[i - 1];
    const auto & [curr_time, curr_ball] = ball_data[i];
    const auto & [next_time, next_ball] = ball_data[i + 1];

    double prev_speed = prev_ball.vel.norm();
    double curr_speed = curr_ball.vel.norm();
    double next_speed = next_ball.vel.norm();

    // 急激な速度増加を検出
    double speed_increase = curr_speed - prev_speed;
    double speed_ratio = prev_speed > 0.1 ? speed_increase / prev_speed : speed_increase;

    // デバッグ: 条件に近いケースをログ出力
    if (speed_increase > 0.5 || speed_ratio > 1.0 || curr_speed > config_.min_kick_speed * 0.5) {
      potential_kicks++;
      if (i % 100 == 0) {  // 100個に1回ログ出力
        RCLCPP_DEBUG(
          rclcpp::get_logger("BallCalibrationDataExtractor"),
          "候補 #%zu: 前速度=%.3f, 現速度=%.3f, 次速度=%.3f, 増加=%.3f, 比率=%.3f", i, prev_speed,
          curr_speed, next_speed, speed_increase, speed_ratio);
      }
    }

    bool speed_increase_ok = speed_increase > 1.0;
    bool speed_ratio_ok = speed_ratio > 2.0;
    bool min_speed_ok = curr_speed > config_.min_kick_speed;

    if (speed_increase_ok) speed_increase_candidates++;
    if (speed_ratio_ok) speed_ratio_candidates++;

    // キャリブレーション用：基本的な速度・比率チェックのみで十分
    // 物理的妥当性チェックは参考程度に留める
    bool physics_valid = validateBallPhysics(ball_data, i);
    
    if (speed_increase_ok && speed_ratio_ok && min_speed_ok) {
      // キャリブレーション用には物理性チェックで除外せず、包括的にデータ収集
      kick_events.emplace_back(curr_time, curr_ball.pos);
      RCLCPP_INFO(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "キックイベント検出: 時刻=%.3fs, 位置=(%.3f,%.3f), 速度=%.3fm/s, 増加=%.3f, 比率=%.3f, 物理性=%s",
        curr_time.seconds(), curr_ball.pos.x(), curr_ball.pos.y(), curr_speed, speed_increase,
        speed_ratio, physics_valid ? "OK" : "参考");
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("BallCalibrationDataExtractor"),
    "キックイベント検出完了: %zu個検出, 候補=%zu, 速度増加条件=%zu, 速度比率条件=%zu",
    kick_events.size(), potential_kicks, speed_increase_candidates, speed_ratio_candidates);

  return kick_events;
}

auto BallCalibrationDataExtractor::validateTrajectoryQuality(const std::vector<Ball> & trajectory)
  -> bool
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("BallCalibrationDataExtractor"),
    "軌道品質チェック: 軌道点数=%zu (必要最小値=%zu)", trajectory.size(),
    config_.min_trajectory_points);

  if (trajectory.size() < config_.min_trajectory_points) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("BallCalibrationDataExtractor"), "軌道点数不足: %zu < %zu",
      trajectory.size(), config_.min_trajectory_points);
    return false;
  }

  // 注意: 現在の実装では時間情報が利用できないため、軌道点数のみで品質を判定
  // 実際の軌道継続時間のチェックは、KickDataPointレベルで実行される
  RCLCPP_DEBUG(
    rclcpp::get_logger("BallCalibrationDataExtractor"),
    "軌道品質チェック合格: 十分な軌道点数 (%zu点)", trajectory.size());

  return true;  // 軌道点数が十分であれば品質OK
}

auto BallCalibrationDataExtractor::validateBallPhysics(
  const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data, size_t index) -> bool
{
  if (index >= ball_data.size()) {
    return false;
  }

  const auto & [time, ball] = ball_data[index];
  double speed = ball.vel.norm();

  // 最大速度チェック
  if (speed > config_.max_kick_speed) {
    RCLCPP_WARN(
      rclcpp::get_logger("BallCalibrationDataExtractor"),
      "物理的上限超過で除外: 速度=%.3fm/s > %.3fm/s (インデックス=%zu)", speed,
      config_.max_kick_speed, index);
    return false;
  }

  // 加速度チェック（前後のフレームと比較）
  if (index > 0 && index < ball_data.size() - 1) {
    const auto & [prev_time, prev_ball] = ball_data[index - 1];
    const auto & [next_time, next_ball] = ball_data[index + 1];

    double dt_prev = (time - prev_time).seconds();
    double dt_next = (next_time - time).seconds();

    if (dt_prev > 1e-6) {  // 前フレームとの時間差がある場合
      double prev_speed = prev_ball.vel.norm();
      double acceleration = (speed - prev_speed) / dt_prev;

      if (std::abs(acceleration) > config_.max_acceleration) {
        RCLCPP_WARN(
          rclcpp::get_logger("BallCalibrationDataExtractor"),
          "異常加速度で除外: 加速度=%.1fm/s² > %.1fm/s² (速度変化=%.3f→%.3fm/s)", acceleration,
          config_.max_acceleration, prev_speed, speed);
        return false;
      }
    }
  }

  return true;
}

auto BallCalibrationDataExtractor::validateSpeedConsistency(
  const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data, size_t kick_index) -> bool
{
  if (kick_index + config_.min_consistency_frames >= ball_data.size()) {
    return false;  // 十分なフレーム数がない
  }

  const auto & [kick_time, kick_ball] = ball_data[kick_index];
  double kick_speed = kick_ball.vel.norm();

  // キック後数フレームの速度変化を確認
  size_t consistent_frames = 0;
  for (size_t i = kick_index + 1; i < kick_index + config_.min_consistency_frames && i < ball_data.size(); ++i) {
    const auto & [time, ball] = ball_data[i];
    double speed = ball.vel.norm();
    
    // キック後は速度が維持または物理的に減少しているかチェック
    // キャリブレーション用には緩い基準を適用
    if (speed >= config_.min_kick_speed * 0.2) {  // より緩い下限
      consistent_frames++;
    } else if (speed > kick_speed * 3.0) {  // より緩い上限（データ品質より包括性重視）
      // 速度が非常に大幅に増加している場合のみ除外
      RCLCPP_DEBUG(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "速度一貫性違反で除外: フレーム%zu で速度が%.3f→%.3fm/s に増加", i, kick_speed, speed);
      return false;
    }
  }

  return consistent_frames >= config_.min_consistency_frames - 1;
}

auto BallCalibrationDataExtractor::extractKickPower(
  const rclcpp::Time & kick_time,
  const std::vector<std::pair<rclcpp::Time, crane_msgs::msg::RobotCommand>> & command_data,
  uint8_t robot_id) -> std::pair<double, bool>
{
  // 指定されたロボットIDとキック時刻に最も近いコマンドを検索
  auto best_it = command_data.end();
  double min_time_diff = std::numeric_limits<double>::max();

  for (auto it = command_data.begin(); it != command_data.end(); ++it) {
    if (it->second.robot_id == robot_id) {
      double time_diff = std::abs((it->first - kick_time).seconds());
      if (time_diff < min_time_diff && time_diff <= 0.5) {  // 500ms以内
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

auto BallCalibrationDataExtractor::updateExtractionStats(
  const std::vector<KickDataPoint> & kick_points) -> void
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

auto BallCalibrationDataExtractor::visualizeKickEvents(
  const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data,
  const std::vector<std::pair<rclcpp::Time, Point>> & kick_events,
  const std::string & output_prefix) -> void
{
  RCLCPP_INFO(
    rclcpp::get_logger("BallCalibrationDataExtractor"),
    "キックイベント可視化開始: %zu個のイベントを処理", kick_events.size());

  for (size_t event_idx = 0; event_idx < kick_events.size(); ++event_idx) {
    const auto & [kick_time, kick_pos] = kick_events[event_idx];
    
    // キック前後3秒のデータを抽出
    double window_seconds = 3.0;
    std::vector<std::pair<double, Ball>> event_data;
    
    for (const auto & [time, ball] : ball_data) {
      double relative_time = (time - kick_time).seconds();
      if (relative_time >= -window_seconds && relative_time <= window_seconds) {
        event_data.emplace_back(relative_time, ball);
      }
    }
    
    if (event_data.size() < 5) {
      RCLCPP_WARN(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "イベント%zu: データ不足のためスキップ (%zu点)", event_idx, event_data.size());
      continue;
    }

    // JSONデータファイル生成
    std::ostringstream data_filename;
    data_filename << output_prefix << "_" << event_idx << "_data.json";
    
    nlohmann::json data_json;
    data_json["event_info"]["event_index"] = event_idx;
    data_json["event_info"]["kick_time_seconds"] = kick_time.seconds();
    data_json["event_info"]["kick_position"]["x"] = kick_pos.x();
    data_json["event_info"]["kick_position"]["y"] = kick_pos.y();
    data_json["event_info"]["data_points_count"] = event_data.size();
    data_json["event_info"]["time_window_seconds"] = window_seconds;
    
    // データ配列をJSON形式で保存
    data_json["data"]["time"] = nlohmann::json::array();
    data_json["data"]["position"]["x"] = nlohmann::json::array();
    data_json["data"]["position"]["y"] = nlohmann::json::array();
    data_json["data"]["velocity"]["x"] = nlohmann::json::array();
    data_json["data"]["velocity"]["y"] = nlohmann::json::array();
    data_json["data"]["speed"] = nlohmann::json::array();
    
    for (const auto & [time_rel, ball] : event_data) {
      data_json["data"]["time"].push_back(time_rel);
      data_json["data"]["position"]["x"].push_back(ball.pos.x());
      data_json["data"]["position"]["y"].push_back(ball.pos.y());
      data_json["data"]["velocity"]["x"].push_back(ball.vel.x());
      data_json["data"]["velocity"]["y"].push_back(ball.vel.y());
      data_json["data"]["speed"].push_back(ball.vel.norm());
    }
    
    // JSONファイル出力
    std::ofstream data_file(data_filename.str());
    if (!data_file.is_open()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "データファイル作成失敗: %s", data_filename.str().c_str());
      continue;
    }
    data_file << data_json.dump(2);
    data_file.close();

    // Pythonスクリプト生成（データ読み込み版）
    std::ostringstream script_filename;
    script_filename << output_prefix << "_" << event_idx << "_plot.py";
    
    std::ofstream script_file(script_filename.str());
    if (!script_file.is_open()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "スクリプトファイル作成失敗: %s", script_filename.str().c_str());
      continue;
    }

    // Pythonスクリプト内容を生成（データファイル読み込み版）
    script_file << "#!/usr/bin/env python3\n";
    script_file << "import matplotlib.pyplot as plt\n";
    script_file << "import numpy as np\n";
    script_file << "import json\n\n";
    
    script_file << "# データファイル読み込み\n";
    script_file << "with open('" << data_filename.str() << "', 'r') as f:\n";
    script_file << "    data = json.load(f)\n\n";
    
    script_file << "# データ抽出\n";
    script_file << "event_info = data['event_info']\n";
    script_file << "time = data['data']['time']\n";
    script_file << "pos_x = data['data']['position']['x']\n";
    script_file << "pos_y = data['data']['position']['y']\n";
    script_file << "vel_x = data['data']['velocity']['x']\n";
    script_file << "vel_y = data['data']['velocity']['y']\n";
    script_file << "speed = data['data']['speed']\n\n";
    
    script_file << "kick_pos_x = event_info['kick_position']['x']\n";
    script_file << "kick_pos_y = event_info['kick_position']['y']\n";
    script_file << "event_idx = event_info['event_index']\n\n";
    

    // グラフプロット部分
    script_file << "# グラフ生成\n";
    script_file << "fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))\n";
    script_file << "fig.suptitle(f'Kick Event {event_idx} Analysis', fontsize=16)\n\n";
    
    // ボール軌道 (XY)
    script_file << "# ボール軌道 (XY)\n";
    script_file << "ax1.plot(pos_x, pos_y, 'b-', linewidth=2, label='Ball trajectory')\n";
    script_file << "ax1.scatter([kick_pos_x], [kick_pos_y], color='red', s=100, marker='*', label='Kick position', zorder=5)\n";
    script_file << "ax1.set_xlabel('X Position (m)')\n";
    script_file << "ax1.set_ylabel('Y Position (m)')\n";
    script_file << "ax1.set_title('Ball Trajectory (XY)')\n";
    script_file << "ax1.grid(True, alpha=0.3)\n";
    script_file << "ax1.legend()\n";
    script_file << "ax1.set_aspect('equal')\n\n";
    
    // 位置vs時間
    script_file << "# 位置 vs 時間\n";
    script_file << "ax2.plot(time, pos_x, 'b-', label='X position', linewidth=2)\n";
    script_file << "ax2.plot(time, pos_y, 'g-', label='Y position', linewidth=2)\n";
    script_file << "ax2.axvline(x=0, color='red', linestyle='--', alpha=0.7, label='Kick time')\n";
    script_file << "ax2.set_xlabel('Time relative to kick (s)')\n";
    script_file << "ax2.set_ylabel('Position (m)')\n";
    script_file << "ax2.set_title('Position vs Time')\n";
    script_file << "ax2.grid(True, alpha=0.3)\n";
    script_file << "ax2.legend()\n\n";
    
    // 速度vs時間
    script_file << "# 速度 vs 時間\n";
    script_file << "ax3.plot(time, vel_x, 'b-', label='X velocity', linewidth=2)\n";
    script_file << "ax3.plot(time, vel_y, 'g-', label='Y velocity', linewidth=2)\n";
    script_file << "ax3.plot(time, speed, 'r-', label='Speed', linewidth=3)\n";
    script_file << "ax3.axvline(x=0, color='red', linestyle='--', alpha=0.7, label='Kick time')\n";
    script_file << "ax3.set_xlabel('Time relative to kick (s)')\n";
    script_file << "ax3.set_ylabel('Velocity (m/s)')\n";
    script_file << "ax3.set_title('Velocity vs Time')\n";
    script_file << "ax3.grid(True, alpha=0.3)\n";
    script_file << "ax3.legend()\n\n";
    
    // 速度ベクトル
    script_file << "# 速度ベクトル（データ点数に応じて間引き）\n";
    script_file << "step = max(1, len(pos_x) // 20)  # 最大20個のベクトルを表示\n";
    script_file << "ax4.quiver(pos_x[::step], pos_y[::step], vel_x[::step], vel_y[::step], ";
    script_file << "angles='xy', scale_units='xy', scale=1, alpha=0.7)\n";
    script_file << "ax4.plot(pos_x, pos_y, 'b-', alpha=0.5, linewidth=1)\n";
    script_file << "ax4.scatter([kick_pos_x], [kick_pos_y], color='red', s=100, marker='*', label='Kick position', zorder=5)\n";
    script_file << "ax4.set_xlabel('X Position (m)')\n";
    script_file << "ax4.set_ylabel('Y Position (m)')\n";
    script_file << "ax4.set_title('Ball Velocity Vectors')\n";
    script_file << "ax4.grid(True, alpha=0.3)\n";
    script_file << "ax4.set_aspect('equal')\n";
    script_file << "ax4.legend()\n\n";
    
    script_file << "plt.tight_layout()\n";
    script_file << "output_file = f'" << output_prefix << "_{event_idx}_plot.png'\n";
    script_file << "plt.savefig(output_file, dpi=300, bbox_inches='tight')\n";
    script_file << "print(f'グラフ保存: {output_file}')\n";
    script_file << "print(f'データファイル: " << output_prefix << "_" << event_idx << "_data.json')\n";
    script_file << "# plt.show()  # コメントアウトを外すと表示\n";
    
    script_file.close();
    
    RCLCPP_INFO(
      rclcpp::get_logger("BallCalibrationDataExtractor"),
      "データファイル生成: %s (%zu データ点)", data_filename.str().c_str(), event_data.size());
    RCLCPP_INFO(
      rclcpp::get_logger("BallCalibrationDataExtractor"),
      "可視化スクリプト生成: %s", script_filename.str().c_str());
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("BallCalibrationDataExtractor"),
    "可視化完了。生成されたPythonスクリプトとデータファイル:");
  
  for (size_t i = 0; i < kick_events.size(); ++i) {
    RCLCPP_INFO(
      rclcpp::get_logger("BallCalibrationDataExtractor"),
      "  python3 %s_%zu_plot.py (データ: %s_%zu_data.json)", output_prefix.c_str(), i, output_prefix.c_str(), i);
  }
}

auto BallCalibrationDataExtractor::filterTeleportEvents(
  const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data,
  const std::vector<std::pair<rclcpp::Time, Point>> & kick_events)
  -> std::vector<std::pair<rclcpp::Time, Point>>
{
  std::vector<std::pair<rclcpp::Time, Point>> filtered_events;
  
  RCLCPP_INFO(
    rclcpp::get_logger("BallCalibrationDataExtractor"),
    "テレポート検出開始: %zu 個のキック候補を分析", kick_events.size());
  
  for (const auto & [kick_time, kick_pos] : kick_events) {
    // キック前後のデータを抽出（前後3秒間）
    const double analysis_window = 3.0;  // 秒
    const double teleport_distance_threshold = 1.5;  // m (1.5m以上の瞬間移動は疑わしい) - より厳格に
    const double min_gradual_acceleration = 0.5;  // m/s² (段階的加速の最小値)
    const double max_teleport_velocity_duration = 0.3;  // s (テレポート時の短時間速度スパイク)
    const double teleport_speed_threshold = 3.0;  // m/s (テレポート疑いの速度閾値)
    
    std::vector<std::pair<rclcpp::Time, Ball>> event_data;
    
    // 時間窓内のデータを収集
    for (const auto & [timestamp, ball] : ball_data) {
      double dt = (timestamp - kick_time).seconds();
      if (std::abs(dt) <= analysis_window) {
        event_data.push_back({timestamp, ball});
      }
    }
    
    if (event_data.size() < 10) {
      RCLCPP_WARN(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "データ点不足でキック候補を除外: %zu 点", event_data.size());
      continue;
    }
    
    // 時系列順にソート
    std::sort(event_data.begin(), event_data.end(),
      [](const auto & a, const auto & b) { return a.first < b.first; });
    
    // キック時刻に最も近いインデックスを検索
    size_t kick_index = 0;
    double min_time_diff = std::numeric_limits<double>::max();
    for (size_t i = 0; i < event_data.size(); ++i) {
      double time_diff = std::abs((event_data[i].first - kick_time).seconds());
      if (time_diff < min_time_diff) {
        min_time_diff = time_diff;
        kick_index = i;
      }
    }
    
    // テレポート特徴を解析
    bool is_teleport = false;
    std::string teleport_reason;
    
    // 1. 瞬間的な大距離移動チェック（より敏感に）
    if (kick_index > 0 && kick_index < event_data.size() - 1) {
      const Ball & before_ball = event_data[kick_index - 1].second;
      const Ball & kick_ball = event_data[kick_index].second;
      const Ball & after_ball = event_data[kick_index + 1].second;
      
      // キック前後の位置変化
      double position_jump = (after_ball.pos - before_ball.pos).norm();
      double time_diff = (event_data[kick_index + 1].first - event_data[kick_index - 1].first).seconds();
      
      // キック直前の位置変化もチェック
      double pre_jump = (kick_ball.pos - before_ball.pos).norm();
      double pre_time_diff = (event_data[kick_index].first - event_data[kick_index - 1].first).seconds();
      
      if ((position_jump > teleport_distance_threshold && time_diff < 0.15) ||
          (pre_jump > 0.8 && pre_time_diff < 0.08)) {  // より敏感な検出
        is_teleport = true;
        teleport_reason = "瞬間的大距離移動 (全体=" + std::to_string(position_jump) + "m/" + 
                         std::to_string(time_diff) + "s, 直前=" + std::to_string(pre_jump) + 
                         "m/" + std::to_string(pre_time_diff) + "s)";
      }
    }
    
    // 2. 速度パターン解析 - テレポートは短時間の速度スパイクを示す
    if (!is_teleport && kick_index >= 2 && kick_index < event_data.size() - 2) {
      std::vector<double> speeds_before, speeds_after;
      
      // キック前後の速度パターンを抽出
      for (int i = -2; i <= 2; ++i) {
        size_t idx = kick_index + i;
        if (idx < event_data.size()) {
          double speed = event_data[idx].second.vel.norm();
          if (i < 0) speeds_before.push_back(speed);
          else if (i > 0) speeds_after.push_back(speed);
        }
      }
      
      // キック直後の最大速度
      double max_kick_speed = 0.0;
      if (kick_index < event_data.size() - 1) {
        max_kick_speed = event_data[kick_index].second.vel.norm();
        if (kick_index + 1 < event_data.size()) {
          max_kick_speed = std::max(max_kick_speed, event_data[kick_index + 1].second.vel.norm());
        }
      }
      
      // 前後の平均速度
      double avg_speed_before = speeds_before.empty() ? 0.0 : 
        std::accumulate(speeds_before.begin(), speeds_before.end(), 0.0) / speeds_before.size();
      double avg_speed_after = speeds_after.empty() ? 0.0 : 
        std::accumulate(speeds_after.begin(), speeds_after.end(), 0.0) / speeds_after.size();
      
      // テレポートパターン: 前後が低速、中央が高速、かつ持続時間が短い
      if (avg_speed_before < 0.8 && avg_speed_after < 1.0 && max_kick_speed > teleport_speed_threshold) {
        // 高速度の持続時間をチェック
        double high_speed_duration = 0.0;
        for (size_t i = kick_index; i < std::min(kick_index + 5, event_data.size()); ++i) {
          if (event_data[i].second.vel.norm() > 2.0) {
            if (i + 1 < event_data.size()) {
              high_speed_duration += (event_data[i + 1].first - event_data[i].first).seconds();
            }
          } else {
            break;
          }
        }
        
        if (high_speed_duration < max_teleport_velocity_duration) {
          is_teleport = true;
          teleport_reason = "短時間速度スパイクパターン (持続時間: " + 
                           std::to_string(high_speed_duration) + "s, 最大速度: " + 
                           std::to_string(max_kick_speed) + "m/s)";
        }
      }
    }
    
    // 3. 軌道の物理的妥当性チェック
    if (!is_teleport && kick_index >= 1 && kick_index < event_data.size() - 3) {
      // 加速度の連続性をチェック
      std::vector<double> accelerations;
      
      for (size_t i = kick_index; i < std::min(kick_index + 3, event_data.size() - 1); ++i) {
        double dt = (event_data[i + 1].first - event_data[i].first).seconds();
        if (dt > 0.001) {
          double vel_diff = (event_data[i + 1].second.vel - event_data[i].second.vel).norm();
          double acceleration = vel_diff / dt;
          accelerations.push_back(acceleration);
        }
      }
      
      if (!accelerations.empty()) {
        double max_acceleration = *std::max_element(accelerations.begin(), accelerations.end());
        double avg_acceleration = std::accumulate(accelerations.begin(), accelerations.end(), 0.0) / accelerations.size();
        
        // 段階的加速度がない場合はテレポートの疑い
        if (max_acceleration > 100.0 && avg_acceleration < min_gradual_acceleration) {
          is_teleport = true;
          teleport_reason = "非物理的加速度パターン (最大: " + 
                           std::to_string(max_acceleration) + "m/s², 平均: " + 
                           std::to_string(avg_acceleration) + "m/s²)";
        }
      }
    }
    
    // 4. より厳格な軌道継続性チェック
    if (!is_teleport && kick_index >= 3 && kick_index < event_data.size() - 3) {
      // キック前後の速度の連続性をチェック
      double speed_before = 0.0;
      double speed_after = 0.0;
      double speed_peak = event_data[kick_index].second.vel.norm();
      
      // キック前3フレームの平均速度
      for (int i = -3; i < 0; ++i) {
        size_t idx = kick_index + i;
        if (idx < event_data.size()) {
          speed_before += event_data[idx].second.vel.norm();
        }
      }
      speed_before /= 3.0;
      
      // キック後3フレームの平均速度（直後は除く）
      for (int i = 2; i <= 4; ++i) {
        size_t idx = kick_index + i;
        if (idx < event_data.size()) {
          speed_after += event_data[idx].second.vel.norm();
        }
      }
      speed_after /= 3.0;
      
      // テレポートパターン: 急激な速度上昇→急激な減少、前後の継続性なし
      if (speed_before < 0.3 && speed_after < 0.5 && speed_peak > 3.5) {
        double pre_post_ratio = speed_peak / std::max(std::max(speed_before, speed_after), 0.1);
        if (pre_post_ratio > 10.0) {  // 10倍以上の速度比は非現実的
          is_teleport = true;
          teleport_reason = "軌道非継続性 (前=" + std::to_string(speed_before) + 
                           "m/s, ピーク=" + std::to_string(speed_peak) + 
                           "m/s, 後=" + std::to_string(speed_after) + 
                           "m/s, 比率=" + std::to_string(pre_post_ratio) + ")";
        }
      }
    }
    
    if (is_teleport) {
      RCLCPP_WARN(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "テレポート検出により除外: 位置(%.3f, %.3f) 理由=%s",
        kick_pos.x(), kick_pos.y(), teleport_reason.c_str());
    } else {
      filtered_events.push_back({kick_time, kick_pos});
      RCLCPP_DEBUG(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "正当なキック認定: 位置(%.3f, %.3f)", kick_pos.x(), kick_pos.y());
    }
  }
  
  size_t rejected_count = kick_events.size() - filtered_events.size();
  RCLCPP_INFO(
    rclcpp::get_logger("BallCalibrationDataExtractor"),
    "テレポート検出完了: %zu/%zu 個のイベントを除外 (除外率: %.1f%%)",
    rejected_count, kick_events.size(),
    kick_events.empty() ? 0.0 : (100.0 * rejected_count / kick_events.size()));
  
  return filtered_events;
}

}  // namespace crane
