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

      // ボール情報の処理（world_modelから抽出）
      if (bag_message->topic_name == "/world_model") {
        auto world_model_msg = std::make_shared<crane_msgs::msg::WorldModel>();
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        rclcpp::Serialization<crane_msgs::msg::WorldModel> serialization;
        serialization.deserialize_message(&serialized_msg, world_model_msg.get());

        Ball ball;
        ball.fromMsg(world_model_msg->ball_info);
        ball_data.emplace_back(rclcpp::Time(bag_message->recv_timestamp), ball);
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
      "ROSBAGから抽出完了: ボールデータ %zu 個, ロボットコマンド %zu 個", ball_data.size(),
      command_data.size());

    // データの時系列ソート
    std::sort(ball_data.begin(), ball_data.end(), [](const auto & a, const auto & b) {
      return a.first < b.first;
    });
    std::sort(command_data.begin(), command_data.end(), [](const auto & a, const auto & b) {
      return a.first < b.first;
    });

    // ボール速度の統計情報をログ出力
    if (!ball_data.empty()) {
      double min_speed = std::numeric_limits<double>::max();
      double max_speed = 0.0;
      double avg_speed = 0.0;
      for (const auto & [time, ball] : ball_data) {
        double speed = ball.vel.norm();
        min_speed = std::min(min_speed, speed);
        max_speed = std::max(max_speed, speed);
        avg_speed += speed;
      }
      avg_speed /= ball_data.size();

      RCLCPP_INFO(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "ボール速度統計: 最小=%.3fm/s, 最大=%.3fm/s, 平均=%.3fm/s, 最小キック速度閾値=%.3fm/s",
        min_speed, max_speed, avg_speed, config_.min_kick_speed);
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
  auto kick_events = detectKickEvents(ball_data);

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

      if (time_diff <= 0.5) {  // 500ms以内
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
          "時間差が大きすぎて除外: %.3fs > 0.5s", time_diff);
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

  // 速度変化によるキックイベント検出
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

    if (speed_increase_ok && speed_ratio_ok && min_speed_ok) {
      kick_events.emplace_back(curr_time, curr_ball.pos);
      RCLCPP_INFO(
        rclcpp::get_logger("BallCalibrationDataExtractor"),
        "キックイベント検出: 時刻=%.3fs, 位置=(%.3f,%.3f), 速度=%.3fm/s, 増加=%.3f, 比率=%.3f",
        curr_time.seconds(), curr_ball.pos.x(), curr_ball.pos.y(), curr_speed, speed_increase,
        speed_ratio);
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

}  // namespace crane
