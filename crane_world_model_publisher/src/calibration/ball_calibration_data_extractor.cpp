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
#include <fstream>
#include <iomanip>
#include <limits>
#include <nlohmann/json.hpp>
#include <numeric>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <set>
#include <sstream>

namespace crane
{

BallCalibrationDataExtractor::BallCalibrationDataExtractor()
{
  // デフォルト設定（物理的制約を考慮）
  config_.min_kick_speed = 0.8;        // 最小キック速度: 0.8m/s
  config_.max_kick_speed = 30.0;       // 最大キック速度: 30.0m/s（物理的上限）
  config_.max_acceleration = 500.0;    // 最大加速度: 500m/s²（キック瞬間の物理的上限）
  config_.min_consistency_frames = 3;  // 一貫性チェック用最小フレーム数
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

          // Vision生データから速度を時間微分で計算（スパイク対策強化版）
          auto current_time = rclcpp::Time(bag_message->recv_timestamp);
          if (!ball_data.empty()) {
            // 速度計算のため位置データをフィルタリング
            Point smoothed_pos = applySmoothingFilter(ball_data, vision_ball.pos);
            double smoothed_pos_z = applySmoothingFilterScalar(ball_data, vision_ball.pos_z);

            auto & [prev_time, prev_ball] = ball_data.back();
            double dt = (current_time - prev_time).seconds();

            // 適切な時間間隔チェック（動的調整）
            const double min_dt = 1e-5;  // より厳格な最小時間間隔
            const double max_dt = 0.2;   // 最大時間間隔（200ms）

            if (dt >= min_dt && dt <= max_dt) {
              // 平滑化された位置から速度を計算
              Point position_diff = smoothed_pos - prev_ball.pos;
              double pos_z_diff = smoothed_pos_z - prev_ball.pos_z;

              Point raw_velocity = position_diff / dt;
              double raw_velocity_z = pos_z_diff / dt;

              // 速度妥当性チェックと外れ値除去
              auto validated_velocity = validateAndFilterVelocity(ball_data, raw_velocity, dt);
              auto validated_velocity_z = validateAndFilterVelocityScalar(ball_data, raw_velocity_z, dt);

              vision_ball.vel = validated_velocity.first ? validated_velocity.second : Point(0, 0);
              vision_ball.vel_z = validated_velocity_z.first ? validated_velocity_z.second : 0.0;

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
              // 不適切な時間間隔の場合は前回の速度を維持または停止状態
              if (dt > max_dt) {
                // 長時間のギャップ後は停止状態とする
                vision_ball.vel = Point(0, 0);
                vision_ball.vel_z = 0;
                vision_ball.state = Ball::State::STOPPED;
              } else {
                // 短すぎる時間間隔の場合は前回速度を維持
                vision_ball.vel = prev_ball.vel;
                vision_ball.vel_z = prev_ball.vel_z;
                vision_ball.state = prev_ball.state;
              }
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
      } else if (bag_message->topic_name == "/robot_commands") {
        // ロボットコマンドの処理
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
      "ROSBAGから抽出完了: ボールデータ %zu 個（Vision生データ使用）, ロボットコマンド %zu 個",
      ball_data.size(), command_data.size());

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
        "ボール速度統計: 最小=%.3fm/s, 最大=%.3fm/s, 平均=%.3fm/s", min_speed, max_speed,
        avg_speed);

      RCLCPP_INFO(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "異常値統計: 上限超過=%zu個(%.1f%%), 明らかな異常=%zu個(%.1f%%), 閾値: 最小=%.3f, "
        "最大=%.3fm/s",
        excessive_speed_count, (100.0 * excessive_speed_count) / ball_data.size(),
        abnormal_speed_count, (100.0 * abnormal_speed_count) / ball_data.size(),
        config_.min_kick_speed, config_.max_kick_speed);
    }

    // キックデータとボール軌道のマッチング
    kick_data_points = matchBallTrajectoryWithKicks(ball_data, command_data);

    // 統計情報の更新
    updateExtractionStats(kick_data_points);

    // 可視化機能を呼び出し（キックイベントが検出された場合）
    if (!kick_data_points.empty() && !last_ball_data_.empty()) {
      visualizeKickEventsWithPower(
        last_ball_data_, kick_data_points, "kick_event_visualization", bag_path);
    }
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

  // 可視化のためにキックイベントと球データを保存
  last_detected_kick_events_ = kick_events;
  last_ball_data_ = ball_data;

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

        // 軌道データの収集（位置変化による停止判定まで）
        double stationary_start_time = -1.0;  // 静止開始時刻
        Point last_position = Point::Zero();   // 前回の位置
        bool has_movement = false;              // 位置変化が検出されたかのフラグ
        const double position_threshold = 0.05; // 位置変化の閾値 [m] (5cm)
        std::vector<Ball> full_trajectory;      // 停止判定前の完全な軌道データ
        
        for (auto it = trajectory_start; it != ball_data.end(); ++it) {
          const auto & [time, ball] = *it;
          double elapsed = (time - kick_time).seconds();
          Point current_position = ball.pos;
          
          // 全データを一時保存
          full_trajectory.push_back(ball);
          
          // 初回位置設定
          if (full_trajectory.size() == 1) {
            last_position = current_position;
          }
          
          // 位置変化の検出
          double position_change = (current_position - last_position).norm();
          if (position_change > position_threshold) {
            has_movement = true;
            stationary_start_time = -1.0;  // 静止カウンターをリセット
            last_position = current_position;
          }
          
          // 位置による停止判定：0.3秒間位置が変わらない場合
          if (position_change <= position_threshold && elapsed > 0.5) {
            if (stationary_start_time < 0.0) {
              stationary_start_time = elapsed;  // 静止開始時刻を記録
            } else if (elapsed - stationary_start_time > 0.3) {
              // 0.3秒間連続して位置が変わらない場合のみ停止
              // ただし、位置変化が一度も検出されていない場合は継続
              if (has_movement) {
                break;
              }
            }
          } else if (position_change > position_threshold) {
            stationary_start_time = -1.0;  // 位置が変わったらリセット
          }
        }
        
        // 停止判定後の処理：停止点から5cm離れる最後の点を特定
        if (has_movement && stationary_start_time > 0.0) {
          // 停止点（最後のデータ点）の位置を取得
          Point stop_position = full_trajectory.back().pos;
          size_t cutoff_index = full_trajectory.size(); // デフォルトは全データ使用
          const double distance_threshold = 0.05; // 5cm
          
          // 停止点から遡って、初めて5cm離れる点を探す
          for (int i = static_cast<int>(full_trajectory.size()) - 1; i >= 0; --i) {
            Point current_pos = full_trajectory[i].pos;
            double distance_from_stop = (current_pos - stop_position).norm();
            
            if (distance_from_stop > distance_threshold) {
              cutoff_index = i + 1; // この点の次の点（5cm以内に入る最初の点）
              break;
            }
          }
          
          // 少なくとも3点は残す（物理計算のため）
          cutoff_index = std::max(cutoff_index, static_cast<size_t>(3));
          cutoff_index = std::min(cutoff_index, full_trajectory.size());
          
          // 5cm閾値で切り取ったデータを使用
          kick_point.trajectory.assign(full_trajectory.begin(), full_trajectory.begin() + cutoff_index);
          
          RCLCPP_INFO(
            rclcpp::get_logger("BallCalibrationDataExtractor"),
            "停止判定: %.2f秒で静止開始、停止点から5cm離れる点で切り取り、%zu点中%zu点を使用", 
            stationary_start_time, full_trajectory.size(), kick_point.trajectory.size());
        } else {
          // 停止判定されなかった場合は全データを使用
          kick_point.trajectory = full_trajectory;
        }

        // 初期状態の設定
        if (!kick_point.trajectory.empty()) {
          kick_point.initial_ball_state = kick_point.trajectory.front();
        }

        // 軌道品質の評価
        kick_point.valid_trajectory_points = kick_point.trajectory.size();
        
        // 実際の継続時間を計算（軌道の最初から最後まで）
        if (!kick_point.trajectory.empty()) {
          // 軌道データから最初と最後のタイムスタンプを取得
          auto first_time = trajectory_start != ball_data.end() ? trajectory_start->first : kick_time;
          auto last_it = trajectory_start;
          for (auto it = trajectory_start; it != ball_data.end() && 
               std::distance(trajectory_start, it) < static_cast<long>(kick_point.trajectory.size()); ++it) {
            last_it = it;
          }
          kick_point.trajectory_duration = (last_it->first - first_time).seconds();
        } else {
          kick_point.trajectory_duration = 0.0;
        }
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
  size_t min_speed_candidates = 0;

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

    bool min_speed_ok = curr_speed > config_.min_kick_speed;
    
    // キック前の拡張静止状態チェック
    bool extended_stationary_ok = true;
    
    if (i >= config_.required_stationary_frames) {
      for (size_t j = 0; j < config_.required_stationary_frames; ++j) {
        size_t check_index = i - j - 1;  // 現在フレームの1つ前から10フレーム分チェック
        double check_speed = ball_data[check_index].second.vel.norm();
        if (check_speed >= config_.max_pre_kick_speed) {
          extended_stationary_ok = false;
          break;
        }
      }
    } else {
      // データが必要フレーム数未満の場合は除外
      extended_stationary_ok = false;
    }

    if (min_speed_ok) min_speed_candidates++;

    // キャリブレーション用：シンプルな速度チェックと拡張静止状態チェック
    // 物理的妥当性チェックは参考程度に留める
    bool physics_valid = validateBallPhysics(ball_data, i);

    if (min_speed_ok && extended_stationary_ok) {
      // キャリブレーション用には物理性チェックで除外せず、包括的にデータ収集
      kick_events.emplace_back(curr_time, curr_ball.pos);
      RCLCPP_INFO(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "キックイベント検出: 時刻=%.3fs, 位置=(%.3f,%.3f), 現速度=%.3fm/s, "
        "%zuフレーム静止状態=OK, 物理性=%s",
        curr_time.seconds(), curr_ball.pos.x(), curr_ball.pos.y(), curr_speed,
        config_.required_stationary_frames, physics_valid ? "OK" : "参考");
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("BallCalibrationDataExtractor"),
    "キックイベント検出完了: %zu個検出, 候補=%zu, 最小速度条件=%zu",
    kick_events.size(), potential_kicks, min_speed_candidates);

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
    // double dt_next = (next_time - time).seconds();  // 未使用のためコメントアウト

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
  for (size_t i = kick_index + 1;
       i < kick_index + config_.min_consistency_frames && i < ball_data.size(); ++i) {
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

auto BallCalibrationDataExtractor::visualizeKickEventsWithPower(
  const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data,
  const std::vector<KickDataPoint> & kick_data_points,
  const std::string & output_prefix, const std::string & rosbag_path) -> void
{
  RCLCPP_INFO(
    rclcpp::get_logger("BallCalibrationDataExtractor"),
    "キックイベント可視化開始: %zu個のイベントを処理（キック力情報付き）", kick_data_points.size());

  // 出力ディレクトリの決定
  std::string output_dir;
  if (!rosbag_path.empty() && std::filesystem::exists(rosbag_path)) {
    // rosbagパスが指定されている場合は、そのフォルダ内にサブディレクトリを作成
    std::filesystem::path bag_path_obj(rosbag_path);
    std::filesystem::path analysis_dir = bag_path_obj / "ball_calibration_analysis";

    try {
      std::filesystem::create_directories(analysis_dir);
      output_dir = analysis_dir.string();
      RCLCPP_INFO(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "キャリブレーション分析ディレクトリ作成: %s", output_dir.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "分析ディレクトリ作成失敗: %s, 現在のディレクトリを使用", e.what());
      output_dir = ".";
    }
  } else {
    // rosbagパスが指定されていない場合は現在のディレクトリ
    output_dir = ".";
    RCLCPP_WARN(
      rclcpp::get_logger("BallCalibrationDataExtractor"),
      "ROSBAGパスが無効です。現在のディレクトリに出力: %s", rosbag_path.c_str());
  }

  for (size_t event_idx = 0; event_idx < kick_data_points.size(); ++event_idx) {
    const auto & kick_point = kick_data_points[event_idx];
    const auto & kick_time = kick_point.timestamp;
    const auto & kick_pos = kick_point.kick_position;

    // キック前1秒、キック後は実際の軌道の最後まで抽出
    double pre_kick_seconds = 1.0;
    
    // 実際の軌道データから最後の時刻を見つける
    double post_kick_seconds = 3.0;  // デフォルト最小値
    if (!kick_point.trajectory.empty()) {
      // 軌道データに対応するball_dataの最後の時刻を見つける
      auto trajectory_start = std::upper_bound(
        ball_data.begin(), ball_data.end(), std::make_pair(kick_time, Ball{}),
        [](const auto & a, const auto & b) { return a.first < b.first; });
      
      auto trajectory_end = trajectory_start;
      std::advance(trajectory_end, std::min(kick_point.trajectory.size(), 
                                          static_cast<size_t>(std::distance(trajectory_start, ball_data.end()))));
      
      if (trajectory_end != ball_data.begin()) {
        --trajectory_end;  // 最後の有効な要素を指す
        double actual_end_time = (trajectory_end->first - kick_time).seconds();
        post_kick_seconds = std::max(post_kick_seconds, actual_end_time + 0.5);  // 軌道終了後0.5秒余裕
      }
    }
    
    RCLCPP_INFO(
      rclcpp::get_logger("BallCalibrationDataExtractor"),
      "イベント%zu: 時間範囲 %.1f秒 〜 +%.1f秒 (軌道点数: %zu)", 
      event_idx, -pre_kick_seconds, post_kick_seconds, kick_point.trajectory.size());
    
    std::vector<std::pair<rclcpp::Time, Ball>> event_data;

    for (const auto & [time, ball] : ball_data) {
      double relative_time = (time - kick_time).seconds();
      if (relative_time >= -pre_kick_seconds && relative_time <= post_kick_seconds) {
        event_data.emplace_back(time, ball);  // 絶対時刻を保存
      }
    }

    if (event_data.size() < 5) {
      RCLCPP_WARN(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "イベント%zu: データ不足のためスキップ (%zu点)", event_idx, event_data.size());
      continue;
    }

    // JSONデータファイル生成（キック力情報付き）
    std::ostringstream data_filename;
    data_filename << output_dir << "/" << output_prefix << "_" << event_idx << "_data.json";

    nlohmann::json data_json;
    data_json["event_info"]["event_index"] = event_idx;
    data_json["event_info"]["kick_time_seconds"] = kick_time.seconds();
    data_json["event_info"]["kick_position"]["x"] = kick_pos.x();
    data_json["event_info"]["kick_position"]["y"] = kick_pos.y();
    data_json["event_info"]["data_points_count"] = event_data.size();
    data_json["event_info"]["time_window_pre_kick"] = pre_kick_seconds;
    data_json["event_info"]["time_window_post_kick"] = post_kick_seconds;
    
    // キック情報を追加（詳細情報付き）
    data_json["kick_info"]["robot_id"] = kick_point.kicker_id;
    data_json["kick_info"]["is_our_robot"] = kick_point.is_our_robot;
    data_json["kick_info"]["kick_power"] = kick_point.kick_power;
    data_json["kick_info"]["is_chip_kick"] = kick_point.is_chip_kick;
    data_json["kick_info"]["kick_type"] = kick_point.is_chip_kick ? "chip" : "straight";
    
    // 軌道品質情報と分析データ（統合）
    data_json["trajectory_info"]["max_speed"] = kick_point.max_speed;  // フィルタリング後軌道の最大速度（参考用）
    
    // 全データから正確な最大速度を計算
    double actual_max_speed = 0.0;
    for (const auto & [time, ball] : event_data) {
      double speed = std::sqrt(ball.vel.x() * ball.vel.x() + ball.vel.y() * ball.vel.y());
      actual_max_speed = std::max(actual_max_speed, speed);
    }
    data_json["trajectory_info"]["actual_max_speed"] = actual_max_speed;  // 全データからの正確な最大速度
    
    data_json["trajectory_info"]["trajectory_duration"] = kick_point.trajectory_duration;
    data_json["trajectory_info"]["valid_trajectory_points"] = kick_point.valid_trajectory_points;
    data_json["trajectory_info"]["total_trajectory_points"] = kick_point.trajectory.size();
    data_json["trajectory_info"]["trajectory_quality_ratio"] = 
      kick_point.trajectory.size() > 0 ? 
        static_cast<double>(kick_point.valid_trajectory_points) / kick_point.trajectory.size() : 0.0;
    
    // キック前後の状態分析（trajectory_infoに統合）
    if (!event_data.empty()) {
      // キック前の平均速度（最初の1秒間）
      double pre_kick_avg_speed = 0.0;
      size_t pre_kick_samples = 0;
      for (const auto & [time, ball] : event_data) {
        double rel_time = (time - kick_time).seconds();
        if (rel_time >= -1.0 && rel_time < 0.0) {
          pre_kick_avg_speed += std::sqrt(ball.vel.x() * ball.vel.x() + ball.vel.y() * ball.vel.y());
          pre_kick_samples++;
        }
      }
      if (pre_kick_samples > 0) pre_kick_avg_speed /= pre_kick_samples;
      
      // キック直後の最大速度（最初の0.5秒間）
      double post_kick_max_speed = 0.0;
      for (const auto & [time, ball] : event_data) {
        double rel_time = (time - kick_time).seconds();
        if (rel_time >= 0.0 && rel_time <= 0.5) {
          double speed = std::sqrt(ball.vel.x() * ball.vel.x() + ball.vel.y() * ball.vel.y());
          post_kick_max_speed = std::max(post_kick_max_speed, speed);
        }
      }
      
      // analysisデータをtrajectory_infoに統合
      data_json["trajectory_info"]["pre_kick_avg_speed"] = pre_kick_avg_speed;
      data_json["trajectory_info"]["post_kick_max_speed"] = post_kick_max_speed;
      data_json["trajectory_info"]["speed_increase_ratio"] = 
        pre_kick_avg_speed > 0.001 ? post_kick_max_speed / pre_kick_avg_speed : 
          (post_kick_max_speed > 0.1 ? 1000.0 : 1.0);
      data_json["trajectory_info"]["kick_effectiveness"] = kick_point.kick_power > 0.01 ? 
        post_kick_max_speed / kick_point.kick_power : 0.0;
      
      // Eigenを使った線形回帰（キック後の実測データ）
      std::vector<double> post_kick_times, post_kick_speeds;
      
      // 位置の時系列データから速度を計算するため、前の点を記録
      Point prev_position;
      rclcpp::Time prev_time;
      bool has_prev = false;
      
      for (const auto & [time, ball] : event_data) {
        double rel_time = (time - kick_time).seconds();
        
        if (rel_time > 0.0) {  // キック後のデータのみ
          if (has_prev) {
            // 位置差分から速度を計算
            double dt = (time - prev_time).seconds();
            if (dt > 0.001) {  // 最小時間間隔チェック（1ms以上）
              Point pos_diff = ball.pos - prev_position;
              double speed_2d = pos_diff.norm() / dt;
              
              post_kick_times.push_back(rel_time);
              post_kick_speeds.push_back(speed_2d);
            }
          }
          
          // 次の反復のために現在の位置と時刻を保存
          prev_position = ball.pos;
          prev_time = time;
          has_prev = true;
        }
      }
      
      if (post_kick_times.size() >= 2) {
        size_t n = post_kick_times.size();
        
        // Eigenマトリックス構築
        Eigen::MatrixXd A(n, 2);  // [1, t] の設計行列
        Eigen::VectorXd b(n);     // 速度ベクトル
        
        for (size_t i = 0; i < n; ++i) {
          A(i, 0) = 1.0;                    // 切片項
          A(i, 1) = post_kick_times[i];     // 時間項
          b(i) = post_kick_speeds[i];       // 速度
        }
        
        // 最小二乗法による線形回帰: x = (A^T A)^(-1) A^T b
        Eigen::Vector2d coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * b);
        double intercept = coeffs(0);
        double slope = coeffs(1);
        
        // R²計算
        Eigen::VectorXd predicted = A * coeffs;
        double mean_y = b.mean();
        double ss_res = (b - predicted).squaredNorm();
        double ss_tot = (b.array() - mean_y).square().sum();
        double r_squared = ss_tot > 0.0 ? (1.0 - ss_res / ss_tot) : 0.0;
        
        data_json["trajectory_info"]["linear_fit"]["slope"] = slope;
        data_json["trajectory_info"]["linear_fit"]["intercept"] = intercept;
        data_json["trajectory_info"]["linear_fit"]["r_squared"] = r_squared;
        data_json["trajectory_info"]["linear_fit"]["data_points"] = static_cast<int>(n);
      }
    }

    // データ配列をJSON形式で保存（位置、高さ、状態、タイムスタンプ）
    data_json["data"]["timestamp_ns"] = nlohmann::json::array();
    data_json["data"]["position"]["x"] = nlohmann::json::array();
    data_json["data"]["position"]["y"] = nlohmann::json::array();
    data_json["data"]["position"]["z"] = nlohmann::json::array();
    data_json["data"]["ball_state"] = nlohmann::json::array();

    for (const auto & [time, ball] : event_data) {
      // キック時刻を基準としたナノ秒タイムスタンプ
      int64_t relative_ns = (time - kick_time).nanoseconds();
      data_json["data"]["timestamp_ns"].push_back(relative_ns);
      data_json["data"]["position"]["x"].push_back(ball.pos.x());
      data_json["data"]["position"]["y"].push_back(ball.pos.y());
      data_json["data"]["position"]["z"].push_back(ball.pos_z);
      
      // ボール状態情報（STOPPED=0, ROLLING=1, FLYING=2）
      int state_value = 1;  // デフォルトはROLLING
      if (ball.state == Ball::State::STOPPED) state_value = 0;
      else if (ball.state == Ball::State::FLYING) state_value = 2;
      data_json["data"]["ball_state"].push_back(state_value);
    }

    // 物理モデルによる予測軌道生成
    double default_deceleration = 0.35;  // デフォルト減速度パラメータ（キャリブレーション前）
    std::vector<double> prediction_time_points;
    
    // 実測データと同じ時間範囲で予測データを生成
    for (const auto & [time, ball] : event_data) {
      double rel_time = (time - kick_time).seconds();
      prediction_time_points.push_back(rel_time);
    }
    
    auto predicted_trajectory = generatePredictedTrajectory(kick_point, prediction_time_points, default_deceleration);
    
    // 予測データをJSONに追加
    data_json["predicted_trajectory"]["timestamp_ns"] = nlohmann::json::array();
    data_json["predicted_trajectory"]["position"]["x"] = nlohmann::json::array();
    data_json["predicted_trajectory"]["position"]["y"] = nlohmann::json::array();
    data_json["predicted_trajectory"]["deceleration_used"] = default_deceleration;
    data_json["predicted_trajectory"]["model_version"] = "rolling_ball_v1";
    
    for (const auto & [rel_time, predicted_pos] : predicted_trajectory) {
      int64_t prediction_ns = static_cast<int64_t>(rel_time * 1e9);  // 秒をナノ秒に変換
      data_json["predicted_trajectory"]["timestamp_ns"].push_back(prediction_ns);
      data_json["predicted_trajectory"]["position"]["x"].push_back(predicted_pos.x());
      data_json["predicted_trajectory"]["position"]["y"].push_back(predicted_pos.y());
    }

    // JSONファイル出力
    std::ofstream data_file(data_filename.str());
    if (!data_file.is_open()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("BallCalibrationDataExtractor"), "データファイル作成失敗: %s",
        data_filename.str().c_str());
      continue;
    }
    data_file << data_json.dump(2);
    data_file.close();

    // Python可視化スクリプト生成（キック力情報付き）
    generateVisualizationPlotWithPower(data_filename.str(), output_dir, output_prefix, event_idx);

    RCLCPP_INFO(
      rclcpp::get_logger("BallCalibrationDataExtractor"), 
      "データファイル生成: %s (%zu データ点, キック力=%.3f)", 
      data_filename.str().c_str(), event_data.size(), kick_point.kick_power);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("BallCalibrationDataExtractor"),
    "可視化完了。生成されたファイル (%zu イベント):", kick_data_points.size());
  RCLCPP_INFO(
    rclcpp::get_logger("BallCalibrationDataExtractor"), "出力ディレクトリ: %s", output_dir.c_str());
  RCLCPP_INFO(
    rclcpp::get_logger("BallCalibrationDataExtractor"),
    "ファイル形式: %s_<event_id>_data.json, %s_<event_id>_plot.png", output_prefix.c_str(),
    output_prefix.c_str());
}

auto BallCalibrationDataExtractor::generateVisualizationPlotWithPower(
  const std::string & json_data_file, const std::string & output_dir,
  const std::string & output_prefix, size_t event_idx) -> void
{
  RCLCPP_INFO(
    rclcpp::get_logger("BallCalibrationDataExtractor"),
    "キック力情報付き可視化スクリプト生成中: イベント%zu", event_idx);

  // PythonでPNG画像を直接生成
  std::ostringstream plot_filename;
  plot_filename << output_dir << "/" << output_prefix << "_" << event_idx << "_plot.png";

  // Pythonスクリプトを一時ファイルとして作成（キック力情報付き）
  std::ostringstream temp_script;
  temp_script << "#!/usr/bin/env python3\n";
  temp_script << "# -*- coding: utf-8 -*-\n";
  temp_script << "import matplotlib\n";
  temp_script << "matplotlib.use('Agg')  # GUI不要のバックエンド\n";
  temp_script << "import matplotlib.pyplot as plt\n";
  temp_script << "import numpy as np\n";
  temp_script << "import json\n";
  temp_script << "import sys\n\n";
  temp_script << "# 日本語フォント設定（matplotlib-fontja使用）\n";
  temp_script << "try:\n";
  temp_script << "    import matplotlib_fontja\n";
  temp_script << "    matplotlib_fontja.set_language('ja')\n";
  temp_script << "    print('matplotlib-fontjaで日本語フォント設定完了')\n";
  temp_script << "except ImportError:\n";
  temp_script << "    print('警告: matplotlib-fontjaがインストールされていません')\n";
  temp_script << "    print('pip install matplotlib-fontja でインストールしてください')\n";
  temp_script << "except Exception as e:\n";
  temp_script << "    print(f'matplotlib-fontja設定エラー: {e}')\n";
  temp_script << "\n";

  temp_script << "# データファイル読み込み\n";
  temp_script << "with open(sys.argv[1], 'r') as f:\n";
  temp_script << "    data = json.load(f)\n\n";

  temp_script << "# データ抽出\n";
  temp_script << "event_info = data['event_info']\n";
  temp_script << "kick_info = data.get('kick_info', {})\n";
  temp_script << "trajectory_info = data.get('trajectory_info', {})\n";
  temp_script << "timestamp_ns = np.array(data['data']['timestamp_ns'])\n";
  temp_script << "pos_x = np.array(data['data']['position']['x'])\n";
  temp_script << "pos_y = np.array(data['data']['position']['y'])\n";
  temp_script << "pos_z = np.array(data['data'].get('position', {}).get('z', [0]*len(pos_x)))\n";
  temp_script << "ball_state = np.array(data['data'].get('ball_state', [1]*len(pos_x)))\n\n";
  
  temp_script << "# 予測データ抽出\n";
  temp_script << "predicted_data = data.get('predicted_trajectory', {})\n";
  temp_script << "if predicted_data:\n";
  temp_script << "    pred_timestamp_ns = np.array(predicted_data['timestamp_ns'])\n";
  temp_script << "    pred_pos_x = np.array(predicted_data['position']['x'])\n";
  temp_script << "    pred_pos_y = np.array(predicted_data['position']['y'])\n";
  temp_script << "    pred_time = pred_timestamp_ns / 1e9  # ナノ秒から秒に変換\n";
  temp_script << "    deceleration_used = predicted_data.get('deceleration_used', 0.35)\n";
  temp_script << "    model_version = predicted_data.get('model_version', 'unknown')\n";
  temp_script << "    has_prediction = True\n";
  temp_script << "else:\n";
  temp_script << "    has_prediction = False\n\n";
  
  temp_script << "# タイムスタンプから時間配列を生成（秒単位）\n";
  temp_script << "time = timestamp_ns / 1e9  # ナノ秒から秒へ変換\n\n";
  
  temp_script << "# Python側で数値微分による速度計算\n";
  temp_script << "def calculate_velocity_with_filtering(pos, time):\n";
  temp_script << "    if len(pos) < 3:\n";
  temp_script << "        return np.zeros_like(pos)\n";
  temp_script << "    vel_raw = np.gradient(pos, time)\n";
  temp_script << "    vel_filtered = np.copy(vel_raw)\n";
  temp_script << "    abnormal_mask = np.abs(vel_filtered) > 50.0\n";
  temp_script << "    vel_filtered[abnormal_mask] = 0.0\n";
  temp_script << "    if len(vel_filtered) >= 5:\n";
  temp_script << "        try:\n";
  temp_script << "            from scipy import ndimage\n";
  temp_script << "            vel_filtered = ndimage.uniform_filter1d(vel_filtered, size=5, mode='nearest')\n";
  temp_script << "        except ImportError:\n";
  temp_script << "            window = 5\n";
  temp_script << "            for i in range(window//2, len(vel_filtered) - window//2):\n";
  temp_script << "                vel_filtered[i] = np.mean(vel_filtered[i-window//2:i+window//2+1])\n";
  temp_script << "    return vel_filtered\n\n";
  
  temp_script << "# 速度計算\n";
  temp_script << "vel_x = calculate_velocity_with_filtering(pos_x, time)\n";
  temp_script << "vel_y = calculate_velocity_with_filtering(pos_y, time)\n";
  temp_script << "vel_z = calculate_velocity_with_filtering(pos_z, time)\n";
  temp_script << "speed = np.sqrt(vel_x**2 + vel_y**2)\n";
  temp_script << "speed_3d = np.sqrt(vel_x**2 + vel_y**2 + vel_z**2)\n\n";

  temp_script << "# キック情報とタイトル生成\n";
  temp_script << "event_idx = event_info['event_index']\n";
  temp_script << "kick_power = kick_info.get('kick_power', 0.0)\n";
  temp_script << "kick_type = kick_info.get('kick_type', 'unknown')\n";
  temp_script << "robot_id = kick_info.get('robot_id', 'N/A')\n";
  temp_script << "# 最大速度は線形近似の切片を使用\n";
  temp_script << "linear_fit = trajectory_info.get('linear_fit', {})\n";
  temp_script << "max_speed = linear_fit.get('intercept', 0.0)\n";

  temp_script << "# グラフ生成（1x2レイアウト）\n";
  temp_script << "fig = plt.figure(figsize=(12, 6))\n";
  temp_script << "title = f'キックイベント {event_idx} 分析 - ロボット{robot_id} ({kick_type}キック)'\n";
  temp_script << "subtitle_parts = [f'キック力: {kick_power:.3f}', f'最大速度: {max_speed:.2f}m/s']\n";
  temp_script << "if has_prediction:\n";
  temp_script << "    subtitle_parts.append(f'予測モデル: α={deceleration_used:.2f}m/s²')\n";
  temp_script << "subtitle = ', '.join(subtitle_parts)\n";
  temp_script << "fig.suptitle(f'{title}\\n{subtitle}', fontsize=12, y=0.98)\n\n";

  temp_script << "# 1. 位置 vs 時間\n";
  temp_script << "ax1 = plt.subplot(1, 2, 1)\n";
  temp_script << "# 実測データ\n";
  temp_script << "ax1.plot(time, pos_x, 'b-', label='X位置 (実測)', linewidth=2)\n";
  temp_script << "ax1.plot(time, pos_y, 'g-', label='Y位置 (実測)', linewidth=2)\n";
  temp_script << "ax1.plot(time, pos_z, 'orange', label='Z位置 (実測)', linewidth=1.5)\n";
  temp_script << "# 予測データ\n";
  temp_script << "if has_prediction:\n";
  temp_script << "    ax1.plot(pred_time, pred_pos_x, 'b--', label=f'X位置 (予測, α={deceleration_used:.2f})', linewidth=1.5, alpha=0.8)\n";
  temp_script << "    ax1.plot(pred_time, pred_pos_y, 'g--', label=f'Y位置 (予測, α={deceleration_used:.2f})', linewidth=1.5, alpha=0.8)\n";
  temp_script << "ax1.axvline(x=0, color='red', linestyle='--', alpha=0.7, label='キック時刻')\n";
  temp_script << "if len(time) > 0:\n";
  temp_script << "    last_time = np.max(time)\n";
  temp_script << "    ax1.axvline(x=last_time, color='blue', linestyle='--', alpha=0.7, label=f'データ終了: {last_time:.2f}s')\n";
  temp_script << "ax1.set_xlabel('キックからの相対時間 (s)')\n";
  temp_script << "ax1.set_ylabel('位置 (m)')\n";
  temp_script << "ax1.set_title('位置 vs 時間 (実測 vs 予測)')\n";
  temp_script << "ax1.grid(True, alpha=0.3)\n";
  temp_script << "ax1.legend()\n\n";

  temp_script << "# 2. 速度 vs 時間 (キック力情報付き)\n";
  temp_script << "ax2 = plt.subplot(1, 2, 2)\n";
  temp_script << "# 実測データ\n";
  temp_script << "ax2.plot(time, speed, 'r-', label='2D速度 (実測)', linewidth=3)\n";
  temp_script << "ax2.plot(time, speed_3d, 'm--', label='3D速度 (実測)', linewidth=2)\n";
  temp_script << "# 予測データ（速度計算）\n";
  temp_script << "if has_prediction and len(pred_time) > 1:\n";
  temp_script << "    pred_vel_x = np.gradient(pred_pos_x, pred_time)\n";
  temp_script << "    pred_vel_y = np.gradient(pred_pos_y, pred_time)\n";
  temp_script << "    pred_speed = np.sqrt(pred_vel_x**2 + pred_vel_y**2)\n";
  temp_script << "    ax2.plot(pred_time, pred_speed, 'r:', label=f'2D速度 (予測, α={deceleration_used:.2f})', linewidth=2, alpha=0.8)\n";
  temp_script << "# C++側で計算済みの線形近似結果を使用\n";
  temp_script << "if 'linear_fit' in trajectory_info and trajectory_info['linear_fit'].get('data_points', 0) >= 2:\n";
  temp_script << "    linear_fit = trajectory_info['linear_fit']\n";
  temp_script << "    slope = linear_fit.get('slope', 0.0)\n";
  temp_script << "    intercept = linear_fit.get('intercept', 0.0)\n";
  temp_script << "    r_squared = linear_fit.get('r_squared', 0.0)\n";
  temp_script << "    # 線形近似線を描画\n";
  temp_script << "    time_fit_extended = np.linspace(0, np.max(time), 100)\n";
  temp_script << "    speed_fit_extended = slope * time_fit_extended + intercept\n";
  temp_script << "    ax2.plot(time_fit_extended, speed_fit_extended, 'cyan', linestyle='-', \n";
  temp_script << "             label=f'線形近似 (傾き: {slope:.2f}m/s², R²: {r_squared:.3f})', linewidth=2, alpha=0.7)\n";
  temp_script << "ax2.axvline(x=0, color='red', linestyle='--', alpha=0.7, label='キック時刻')\n";
  temp_script << "if len(time) > 0:\n";
  temp_script << "    last_time = np.max(time)\n";
  temp_script << "    ax2.axvline(x=last_time, color='blue', linestyle='--', alpha=0.7, label=f'データ終了: {last_time:.2f}s')\n";
  temp_script << "ax2.axhline(y=max_speed, color='orange', linestyle=':', alpha=0.7, label=f'最大速度: {max_speed:.2f}m/s')\n";
  temp_script << "ax2.set_xlabel('キックからの相対時間 (s)')\n";
  temp_script << "ax2.set_ylabel('速度 (m/s)')\n";
  temp_script << "ax2.set_title(f'速度 vs 時間 (実測 vs 予測)')\n";
  temp_script << "ax2.grid(True, alpha=0.3)\n";
  temp_script << "ax2.legend()\n\n";


  temp_script << "# グラフのレイアウト調整と保存\n";
  temp_script << "plt.tight_layout()\n";
  temp_script << "plt.subplots_adjust(top=0.85, bottom=0.15, left=0.08, right=0.95, wspace=0.3)  # 1x2レイアウト調整\n";
  temp_script << "plt.savefig(sys.argv[2], dpi=300, bbox_inches='tight')\n";
  temp_script << "plt.close()\n";
  temp_script << "print(f'\u30adック力情報付きプロット保存成功: {sys.argv[2]}')\n\n";

  // 一時Pythonスクリプトファイルを作成
  std::string temp_py_file = "/tmp/ball_plot_power_" + std::to_string(event_idx) + ".py";
  std::ofstream temp_file(temp_py_file);
  if (temp_file.is_open()) {
    temp_file << temp_script.str();
    temp_file.close();

    // Pythonスクリプトを実行してPNG画像を生成
    std::ostringstream python_cmd;
    python_cmd << "python3 " << temp_py_file << " " << json_data_file << " "
               << plot_filename.str();

    int result = std::system(python_cmd.str().c_str());
    if (result == 0) {
      RCLCPP_INFO(
        rclcpp::get_logger("BallCalibrationDataExtractor"), "キック力情報付きプロット生成成功: %s",
        plot_filename.str().c_str());
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "キック力情報付きプロット生成失敗 (コード: %d): %s", result, plot_filename.str().c_str());
    }

    // 一時ファイルを保持（デバッグ用）
    RCLCPP_DEBUG(
      rclcpp::get_logger("BallCalibrationDataExtractor"), "Pythonスクリプト保存: %s",
      temp_py_file.c_str());
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("BallCalibrationDataExtractor"), "一時Pythonファイル作成失敗: %s",
      temp_py_file.c_str());
  }

  RCLCPP_INFO(
    rclcpp::get_logger("BallCalibrationDataExtractor"), "キック力情報付き可視化完了: %s",
    plot_filename.str().c_str());
}

auto BallCalibrationDataExtractor::applySmoothingFilter(
  const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data, const Point & current_pos) const
  -> Point
{
  if (ball_data.size() < 2) {
    return current_pos;  // データ不足時はそのまま返す
  }

  // 5点移動平均（現在位置 + 過去4点）
  constexpr size_t window_size = 5;
  constexpr double weights[] = {0.4, 0.25, 0.15, 0.15, 0.05};  // 新しいデータにより重みを置く
  
  Point weighted_pos = current_pos * weights[0];
  double total_weight = weights[0];

  size_t available_points = std::min(window_size - 1, ball_data.size());
  for (size_t i = 0; i < available_points; ++i) {
    const Point & prev_pos = ball_data[ball_data.size() - 1 - i].second.pos;
    weighted_pos += prev_pos * weights[i + 1];
    total_weight += weights[i + 1];
  }

  return weighted_pos / total_weight;
}

auto BallCalibrationDataExtractor::applySmoothingFilterScalar(
  const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data, double current_value) const
  -> double
{
  if (ball_data.size() < 2) {
    return current_value;  // データ不足時はそのまま返す
  }

  // 5点移動平均（現在値 + 過去4点）
  constexpr size_t window_size = 5;
  constexpr double weights[] = {0.4, 0.25, 0.15, 0.15, 0.05};  // 新しいデータにより重みを置く

  double weighted_value = current_value * weights[0];
  double total_weight = weights[0];

  size_t available_points = std::min(window_size - 1, ball_data.size());
  for (size_t i = 0; i < available_points; ++i) {
    double prev_value = ball_data[ball_data.size() - 1 - i].second.pos_z;
    weighted_value += prev_value * weights[i + 1];
    total_weight += weights[i + 1];
  }

  return weighted_value / total_weight;
}

auto BallCalibrationDataExtractor::validateAndFilterVelocity(
  const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data, const Point & raw_velocity,
  double dt) const -> std::pair<bool, Point>
{
  double speed = raw_velocity.norm();
  
  // 物理的制約チェック
  const double max_reasonable_speed = 50.0;  // m/s（キャリブレーション用上限）
  const double max_reasonable_acceleration = 1000.0;  // m/s²（キック瞬間の制限）

  if (speed > max_reasonable_speed) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("BallCalibrationDataExtractor"),
      "物理的制約により速度除外: %.3fm/s > %.3fm/s", speed, max_reasonable_speed);
    return {false, Point(0, 0)};
  }

  // 加速度チェック（前フレームとの比較）
  if (ball_data.size() >= 1) {
    const Ball & prev_ball = ball_data.back().second;
    double prev_speed = prev_ball.vel.norm();
    double acceleration = std::abs(speed - prev_speed) / dt;
    
    if (acceleration > max_reasonable_acceleration) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "加速度制約により速度除外: 加速度=%.1fm/s² > %.1fm/s²", acceleration, max_reasonable_acceleration);
      return {false, Point(0, 0)};
    }
  }

  // 統計的外れ値検出（過去の速度データとの比較）
  if (ball_data.size() >= 10) {
    std::vector<double> recent_speeds;
    size_t lookback_size = std::min(static_cast<size_t>(20), ball_data.size());
    
    for (size_t i = ball_data.size() - lookback_size; i < ball_data.size(); ++i) {
      recent_speeds.push_back(ball_data[i].second.vel.norm());
    }

    // IQR法による外れ値検出
    std::sort(recent_speeds.begin(), recent_speeds.end());
    size_t n = recent_speeds.size();
    double q1 = recent_speeds[n / 4];
    double q3 = recent_speeds[3 * n / 4];
    double iqr = q3 - q1;
    double outlier_threshold = q3 + 2.5 * iqr;  // より緩い閾値（キャリブレーション用）

    if (speed > outlier_threshold && outlier_threshold > 1.0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "統計的外れ値として除外: 速度=%.3fm/s > 閾値=%.3fm/s", speed, outlier_threshold);
      return {false, Point(0, 0)};
    }
  }

  return {true, raw_velocity};
}

auto BallCalibrationDataExtractor::validateAndFilterVelocityScalar(
  const std::vector<std::pair<rclcpp::Time, Ball>> & ball_data, double raw_velocity, double dt) const
  -> std::pair<bool, double>
{
  // Z方向速度の物理的制約チェック
  const double max_reasonable_z_velocity = 20.0;  // m/s（Z方向上限）
  const double max_reasonable_z_acceleration = 500.0;  // m/s²（Z方向加速度制限）

  if (std::abs(raw_velocity) > max_reasonable_z_velocity) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("BallCalibrationDataExtractor"),
      "Z方向物理的制約により速度除外: %.3fm/s", raw_velocity);
    return {false, 0.0};
  }

  // Z方向加速度チェック
  if (ball_data.size() >= 1) {
    double prev_z_velocity = ball_data.back().second.vel_z;
    double z_acceleration = std::abs(raw_velocity - prev_z_velocity) / dt;
    
    if (z_acceleration > max_reasonable_z_acceleration) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "Z方向加速度制約により速度除外: 加速度=%.1fm/s²", z_acceleration);
      return {false, 0.0};
    }
  }

  return {true, raw_velocity};
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
    const double teleport_distance_threshold =
      1.5;                                        // m (1.5m以上の瞬間移動は疑わしい) - より厳格に
    const double min_gradual_acceleration = 0.5;  // m/s² (段階的加速の最小値)
    const double max_teleport_velocity_duration = 0.3;  // s (テレポート時の短時間速度スパイク)
    const double teleport_speed_threshold = 3.0;        // m/s (テレポート疑いの速度閾値)

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
    std::sort(event_data.begin(), event_data.end(), [](const auto & a, const auto & b) {
      return a.first < b.first;
    });

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
      double time_diff =
        (event_data[kick_index + 1].first - event_data[kick_index - 1].first).seconds();

      // キック直前の位置変化もチェック
      double pre_jump = (kick_ball.pos - before_ball.pos).norm();
      double pre_time_diff =
        (event_data[kick_index].first - event_data[kick_index - 1].first).seconds();

      if (
        (position_jump > teleport_distance_threshold && time_diff < 0.15) ||
        (pre_jump > 0.8 && pre_time_diff < 0.08)) {  // より敏感な検出
        is_teleport = true;
        teleport_reason = "瞬間的大距離移動 (全体=" + std::to_string(position_jump) + "m/" +
                          std::to_string(time_diff) + "s, 直前=" + std::to_string(pre_jump) + "m/" +
                          std::to_string(pre_time_diff) + "s)";
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
          if (i < 0)
            speeds_before.push_back(speed);
          else if (i > 0)
            speeds_after.push_back(speed);
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
      double avg_speed_before =
        speeds_before.empty()
          ? 0.0
          : std::accumulate(speeds_before.begin(), speeds_before.end(), 0.0) / speeds_before.size();
      double avg_speed_after =
        speeds_after.empty()
          ? 0.0
          : std::accumulate(speeds_after.begin(), speeds_after.end(), 0.0) / speeds_after.size();

      // テレポートパターン: 前後が低速、中央が高速、かつ持続時間が短い
      if (
        avg_speed_before < 0.8 && avg_speed_after < 1.0 &&
        max_kick_speed > teleport_speed_threshold) {
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
          teleport_reason =
            "短時間速度スパイクパターン (持続時間: " + std::to_string(high_speed_duration) +
            "s, 最大速度: " + std::to_string(max_kick_speed) + "m/s)";
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
        double avg_acceleration =
          std::accumulate(accelerations.begin(), accelerations.end(), 0.0) / accelerations.size();

        // 段階的加速度がない場合はテレポートの疑い
        if (max_acceleration > 100.0 && avg_acceleration < min_gradual_acceleration) {
          is_teleport = true;
          teleport_reason = "非物理的加速度パターン (最大: " + std::to_string(max_acceleration) +
                            "m/s², 平均: " + std::to_string(avg_acceleration) + "m/s²)";
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
        "テレポート検出により除外: 位置(%.3f, %.3f) 理由=%s", kick_pos.x(), kick_pos.y(),
        teleport_reason.c_str());
    } else {
      filtered_events.push_back({kick_time, kick_pos});
      RCLCPP_DEBUG(
        rclcpp::get_logger("BallCalibrationDataExtractor"), "正当なキック認定: 位置(%.3f, %.3f)",
        kick_pos.x(), kick_pos.y());
    }
  }

  size_t rejected_count = kick_events.size() - filtered_events.size();
  RCLCPP_INFO(
    rclcpp::get_logger("BallCalibrationDataExtractor"),
    "テレポート検出完了: %zu/%zu 個のイベントを除外 (除外率: %.1f%%)", rejected_count,
    kick_events.size(), kick_events.empty() ? 0.0 : (100.0 * rejected_count / kick_events.size()));

  return filtered_events;
}

auto BallCalibrationDataExtractor::generatePredictedTrajectory(
  const KickDataPoint & kick_point,
  const std::vector<double> & time_points,
  double deceleration) -> std::vector<std::pair<double, Point>>
{
  std::vector<std::pair<double, Point>> predicted_trajectory;

  if (kick_point.trajectory.empty() || time_points.empty()) {
    return predicted_trajectory;
  }

  // キック直後の初期状態を取得
  const Ball & initial_ball = kick_point.initial_ball_state;
  Point initial_position = initial_ball.pos;
  Point initial_velocity = initial_ball.vel;

  double initial_speed = initial_velocity.norm();
  if (initial_speed < 1e-6) {
    // 初期速度が0の場合、全て初期位置
    for (double t : time_points) {
      predicted_trajectory.emplace_back(t, initial_position);
    }
    return predicted_trajectory;
  }

  Point direction = initial_velocity.normalized();
  double stop_time = initial_speed / deceleration;

  for (double t : time_points) {
    Point predicted_pos;

    if (t >= stop_time) {
      // 停止後
      double distance_at_stop =
        initial_speed * stop_time - 0.5 * deceleration * stop_time * stop_time;
      predicted_pos = initial_position + direction * distance_at_stop;
    } else {
      // 減速中
      double distance = initial_speed * t - 0.5 * deceleration * t * t;
      predicted_pos = initial_position + direction * distance;
    }

    predicted_trajectory.emplace_back(t, predicted_pos);
  }

  return predicted_trajectory;
}

}  // namespace crane
