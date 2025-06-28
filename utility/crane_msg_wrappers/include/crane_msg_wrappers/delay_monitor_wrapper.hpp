// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__DELAY_MONITOR_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__DELAY_MONITOR_WRAPPER_HPP_

#include <algorithm>
#include <chrono>
#include <crane_msgs/msg/delay_checkpoint.hpp>
#include <crane_msgs/msg/delay_checkpoints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace crane
{

/**
 * @brief 遅延監視システム用ユーティリティクラス
 * RobotCommandのstate_factorsパターンを参考に、遅延チェックポイント情報を管理する
 */
class DelayMonitorWrapper
{
public:
  using DelayCheckpointMsg = crane_msgs::msg::DelayCheckpoint;
  using DelayCheckpointArray = std::vector<DelayCheckpointMsg>;
  using DelayCheckpointsMsg = crane_msgs::msg::DelayCheckpoints;

  /**
   * @brief チェックポイントを追加する
   * @param name チェックポイント名（例: "vision_received", "ekf_updated"）
   * @param value 追加情報（オプション、例: "30Hz", "robot_id:3"）
   */
  static void addDelayCheckpoint(
    DelayCheckpointArray & checkpoints, const std::string & name, const std::string & value = "")
  {
    auto now = std::chrono::steady_clock::now();
    auto timestamp_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

    // 同じnameのチェックポイントが存在する場合は更新
    auto existing = std::find_if(
      checkpoints.begin(), checkpoints.end(),
      [name](const auto & checkpoint) { return checkpoint.name == name; });

    // 注意: 旧API - 新しいDelayCheckpointsメッセージを使用することを推奨
    if (existing != checkpoints.end()) {
      existing->relative_time_us = static_cast<int32_t>(timestamp_ns / 1000);
      existing->value = value;
    } else {
      DelayCheckpointMsg checkpoint;
      checkpoint.name = name;
      checkpoint.relative_time_us = static_cast<int32_t>(timestamp_ns / 1000);
      checkpoint.value = value;
      checkpoints.push_back(checkpoint);
    }
  }

  /**
   * @brief 指定したチェックポイント間の遅延を計算する（ミリ秒）
   * @param checkpoints チェックポイント配列
   * @param start_name 開始チェックポイント名
   * @param end_name 終了チェックポイント名
   * @return 遅延時間（ミリ秒）、チェックポイントが見つからない場合は-1
   */
  static double calculateDelayMs(
    const DelayCheckpointArray & checkpoints, const std::string & start_name,
    const std::string & end_name)
  {
    auto start_it = std::find_if(
      checkpoints.begin(), checkpoints.end(),
      [start_name](const auto & cp) { return cp.name == start_name; });

    auto end_it = std::find_if(checkpoints.begin(), checkpoints.end(), [end_name](const auto & cp) {
      return cp.name == end_name;
    });

    if (start_it == checkpoints.end() || end_it == checkpoints.end()) {
      return -1.0;
    }

    auto delay_us = end_it->relative_time_us - start_it->relative_time_us;
    return static_cast<double>(delay_us) / 1000.0;  // マイクロ秒をミリ秒に変換
  }

  /**
   * @brief 最初のチェックポイントからの総遅延を計算する（ミリ秒）
   * @param checkpoints チェックポイント配列
   * @param end_name 終了チェックポイント名
   * @return 総遅延時間（ミリ秒）、チェックポイントが見つからない場合は-1
   */
  static double calculateTotalDelayMs(
    const DelayCheckpointArray & checkpoints, const std::string & end_name)
  {
    if (checkpoints.empty()) {
      return -1.0;
    }

    auto end_it = std::find_if(checkpoints.begin(), checkpoints.end(), [end_name](const auto & cp) {
      return cp.name == end_name;
    });

    if (end_it == checkpoints.end()) {
      return -1.0;
    }

    auto start_time = checkpoints.front().relative_time_us;
    auto delay_us = end_it->relative_time_us - start_time;
    return static_cast<double>(delay_us) / 1000.0;
  }

  /**
   * @brief 直前のチェックポイントからの遅延を計算する（ミリ秒）
   * @param checkpoints チェックポイント配列
   * @param current_name 現在のチェックポイント名
   * @return 直前からの遅延時間（ミリ秒）、適切なチェックポイントが見つからない場合は-1
   */
  static double calculateIncrementalDelayMs(
    const DelayCheckpointArray & checkpoints, const std::string & current_name)
  {
    auto current_it = std::find_if(
      checkpoints.begin(), checkpoints.end(),
      [current_name](const auto & cp) { return cp.name == current_name; });

    if (current_it == checkpoints.end() || current_it == checkpoints.begin()) {
      return -1.0;
    }

    auto prev_it = current_it - 1;
    auto delay_us = current_it->relative_time_us - prev_it->relative_time_us;
    return static_cast<double>(delay_us) / 1000.0;
  }

  /**
   * @brief チェックポイント配列を文字列に変換（デバッグ用）
   * @param checkpoints チェックポイント配列
   * @return フォーマットされた文字列
   */
  static std::string checkpointsToString(const DelayCheckpointArray & checkpoints)
  {
    if (checkpoints.empty()) {
      return "[]";
    }

    std::string result = "[";
    for (size_t i = 0; i < checkpoints.size(); ++i) {
      const auto & cp = checkpoints[i];
      result += cp.name;
      if (!cp.value.empty()) {
        result += "(" + cp.value + ")";
      }

      // 前のチェックポイントからの遅延を表示
      if (i > 0) {
        auto delay_us = cp.relative_time_us - checkpoints[i - 1].relative_time_us;
        auto delay_ms = static_cast<double>(delay_us) / 1000.0;
        result += ":" + std::to_string(delay_ms) + "ms";
      }

      if (i < checkpoints.size() - 1) {
        result += ", ";
      }
    }
    result += "]";
    return result;
  }

  /**
   * @brief チェックポイント配列をクリアする
   */
  static void clearCheckpoints(DelayCheckpointArray & checkpoints) { checkpoints.clear(); }

  // ===== Unix時刻変換用ユーティリティ =====

  /**
   * @brief Vision遅延情報を文字列として記録するヘルパー関数
   * @param t_capture Vision起動からのキャプチャ時刻（秒）
   * @param t_sent Vision起動からの送信時刻（秒）
   * @param ros_receive_time ROS2でのパケット受信時刻
   * @return フォーマットされた遅延情報文字列
   */
  static std::string formatVisionDelayInfo(
    double t_capture, double t_sent, const rclcpp::Time & ros_receive_time)
  {
    // Vision内部処理時間
    double vision_processing_ms = (t_sent - t_capture) * 1000.0;

    // Vision相対時刻の情報を文字列として返す
    return "t_capture:" + std::to_string(t_capture) + "s, t_sent:" + std::to_string(t_sent) +
           "s, vision_proc:" + std::to_string(vision_processing_ms) + "ms";
  }

  // ===== 新しいDelayCheckpointsメッセージ用API =====

  /**
   * @brief DelayCheckpointsメッセージにチェックポイントを追加する
   * @param checkpoints DelayCheckpointsメッセージ
   * @param name チェックポイント名
   * @param value 追加情報（オプション）
   */
  static void addDelayCheckpoint(
    DelayCheckpointsMsg & checkpoints, const std::string & name, const std::string & value = "")
  {
    auto now = std::chrono::steady_clock::now();
    auto timestamp_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

    // 最初のチェックポイントの場合、基準タイムスタンプを設定
    if (checkpoints.checkpoints.empty()) {
      checkpoints.reference_timestamp_ns = timestamp_ns;
    }

    // 同じnameのチェックポイントが存在する場合は更新
    auto existing = std::find_if(
      checkpoints.checkpoints.begin(), checkpoints.checkpoints.end(),
      [name](const auto & checkpoint) { return checkpoint.name == name; });

    // 基準からの相対時間をマイクロ秒で計算
    auto relative_time_us =
      static_cast<int32_t>((timestamp_ns - checkpoints.reference_timestamp_ns) / 1000);

    if (existing != checkpoints.checkpoints.end()) {
      existing->relative_time_us = relative_time_us;
      existing->value = value;
    } else {
      DelayCheckpointMsg checkpoint;
      checkpoint.name = name;
      checkpoint.relative_time_us = relative_time_us;
      checkpoint.value = value;
      checkpoints.checkpoints.push_back(checkpoint);
    }
  }

  /**
   * @brief 指定したチェックポイント間の遅延を計算する（ミリ秒）
   * @param checkpoints DelayCheckpointsメッセージ
   * @param start_name 開始チェックポイント名
   * @param end_name 終了チェックポイント名
   * @return 遅延時間（ミリ秒）、チェックポイントが見つからない場合は-1
   */
  static double calculateDelayMs(
    const DelayCheckpointsMsg & checkpoints, const std::string & start_name,
    const std::string & end_name)
  {
    auto start_it = std::find_if(
      checkpoints.checkpoints.begin(), checkpoints.checkpoints.end(),
      [start_name](const auto & cp) { return cp.name == start_name; });

    auto end_it = std::find_if(
      checkpoints.checkpoints.begin(), checkpoints.checkpoints.end(),
      [end_name](const auto & cp) { return cp.name == end_name; });

    if (start_it == checkpoints.checkpoints.end() || end_it == checkpoints.checkpoints.end()) {
      return -1.0;
    }

    auto delay_us = end_it->relative_time_us - start_it->relative_time_us;
    return static_cast<double>(delay_us) / 1000.0;  // マイクロ秒をミリ秒に変換
  }

  /**
   * @brief 最初のチェックポイントからの総遅延を計算する（ミリ秒）
   * @param checkpoints DelayCheckpointsメッセージ
   * @param end_name 終了チェックポイント名
   * @return 総遅延時間（ミリ秒）、チェックポイントが見つからない場合は-1
   */
  static double calculateTotalDelayMs(
    const DelayCheckpointsMsg & checkpoints, const std::string & end_name)
  {
    if (checkpoints.checkpoints.empty()) {
      return -1.0;
    }

    auto end_it = std::find_if(
      checkpoints.checkpoints.begin(), checkpoints.checkpoints.end(),
      [end_name](const auto & cp) { return cp.name == end_name; });

    if (end_it == checkpoints.checkpoints.end()) {
      return -1.0;
    }

    return static_cast<double>(end_it->relative_time_us) / 1000.0;  // マイクロ秒をミリ秒に変換
  }

  /**
   * @brief 直前のチェックポイントからの遅延を計算する（ミリ秒）
   * @param checkpoints DelayCheckpointsメッセージ
   * @param current_name 現在のチェックポイント名
   * @return 直前からの遅延時間（ミリ秒）、適切なチェックポイントが見つからない場合は-1
   */
  static double calculateIncrementalDelayMs(
    const DelayCheckpointsMsg & checkpoints, const std::string & current_name)
  {
    auto current_it = std::find_if(
      checkpoints.checkpoints.begin(), checkpoints.checkpoints.end(),
      [current_name](const auto & cp) { return cp.name == current_name; });

    if (
      current_it == checkpoints.checkpoints.end() ||
      current_it == checkpoints.checkpoints.begin()) {
      return -1.0;
    }

    auto prev_it = current_it - 1;
    auto delay_us = current_it->relative_time_us - prev_it->relative_time_us;
    return static_cast<double>(delay_us) / 1000.0;  // マイクロ秒をミリ秒に変換
  }

  /**
   * @brief DelayCheckpointsメッセージを文字列に変換（デバッグ用）
   * @param checkpoints DelayCheckpointsメッセージ
   * @return フォーマットされた文字列
   */
  static std::string checkpointsToString(const DelayCheckpointsMsg & checkpoints)
  {
    if (checkpoints.checkpoints.empty()) {
      return "[]";
    }

    std::string result = "[";
    for (size_t i = 0; i < checkpoints.checkpoints.size(); ++i) {
      const auto & cp = checkpoints.checkpoints[i];
      result += cp.name;
      if (!cp.value.empty()) {
        result += "(" + cp.value + ")";
      }

      // 前のチェックポイントからの遅延を表示
      if (i > 0) {
        auto delay_us = cp.relative_time_us - checkpoints.checkpoints[i - 1].relative_time_us;
        auto delay_ms = static_cast<double>(delay_us) / 1000.0;
        result += ":" + std::to_string(delay_ms) + "ms";
      } else {
        // 最初のチェックポイントは基準からの時間を表示
        auto delay_ms = static_cast<double>(cp.relative_time_us) / 1000.0;
        result += ":" + std::to_string(delay_ms) + "ms";
      }

      if (i < checkpoints.checkpoints.size() - 1) {
        result += ", ";
      }
    }
    result += "]";
    return result;
  }

  /**
   * @brief DelayCheckpointsメッセージをクリアする
   */
  static void clearCheckpoints(DelayCheckpointsMsg & checkpoints)
  {
    checkpoints.checkpoints.clear();
    checkpoints.reference_timestamp_ns = 0;
  }

  /**
   * @brief 既存のDelayCheckpointsメッセージに別のメッセージをマージする
   * 基準タイムスタンプが異なる場合は、より古い基準を使用し、時間を調整
   */
  static void mergeCheckpoints(DelayCheckpointsMsg & target, const DelayCheckpointsMsg & source)
  {
    if (source.checkpoints.empty()) {
      return;
    }

    // targetが空の場合、sourceをそのままコピー
    if (target.checkpoints.empty()) {
      target = source;
      return;
    }

    // 基準タイムスタンプの調整
    int64_t target_ref = target.reference_timestamp_ns;
    int64_t source_ref = source.reference_timestamp_ns;
    int64_t common_ref = std::min(target_ref, source_ref);

    // targetの基準を調整
    if (target_ref != common_ref) {
      int64_t offset_us = (target_ref - common_ref) / 1000;
      for (auto & cp : target.checkpoints) {
        cp.relative_time_us += static_cast<int32_t>(offset_us);
      }
      target.reference_timestamp_ns = common_ref;
    }

    // sourceのチェックポイントを追加（基準時刻調整込み）
    int64_t source_offset_us = (source_ref - common_ref) / 1000;
    for (const auto & src_cp : source.checkpoints) {
      auto existing = std::find_if(
        target.checkpoints.begin(), target.checkpoints.end(),
        [src_cp](const auto & cp) { return cp.name == src_cp.name; });

      DelayCheckpointMsg adjusted_cp = src_cp;
      adjusted_cp.relative_time_us += static_cast<int32_t>(source_offset_us);

      if (existing != target.checkpoints.end()) {
        // より新しいタイムスタンプのものを保持
        if (adjusted_cp.relative_time_us > existing->relative_time_us) {
          *existing = adjusted_cp;
        }
      } else {
        target.checkpoints.push_back(adjusted_cp);
      }
    }
  }

  /**
   * @brief 既存のチェックポイント配列に別の配列をマージする
   * 同名のチェックポイントがある場合は、より新しいタイムスタンプのものを保持
   */
  static void mergeCheckpoints(DelayCheckpointArray & target, const DelayCheckpointArray & source)
  {
    for (const auto & src_cp : source) {
      auto existing = std::find_if(
        target.begin(), target.end(), [src_cp](const auto & cp) { return cp.name == src_cp.name; });

      if (existing != target.end()) {
        // より新しいタイムスタンプのものを保持
        if (src_cp.relative_time_us > existing->relative_time_us) {
          *existing = src_cp;
        }
      } else {
        target.push_back(src_cp);
      }
    }
  }
};

}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__DELAY_MONITOR_WRAPPER_HPP_
