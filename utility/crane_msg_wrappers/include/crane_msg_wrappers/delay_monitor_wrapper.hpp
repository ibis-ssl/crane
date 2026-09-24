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
#include <format>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
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
  using DelayCheckpointsMsg = crane_msgs::msg::DelayCheckpoints;

  // ===== Unix時刻変換用ユーティリティ =====

  /**
   * @brief Vision遅延情報を文字列として記録するヘルパー関数
   * @param t_capture Vision起動からのキャプチャ時刻（秒）
   * @param t_sent Vision起動からの送信時刻（秒）
   * @param ros_receive_time ROS 2でのパケット受信時刻
   * @return フォーマットされた遅延情報文字列
   */
  static std::string formatVisionDelayInfo(
    double t_capture, double t_sent, [[maybe_unused]] const rclcpp::Time & ros_receive_time)
  {
    double vision_processing_ms = (t_sent - t_capture) * 1000.0;
    return std::format(
      "t_capture:{}s, t_sent:{}s, vision_proc:{}ms", t_capture, t_sent, vision_processing_ms);
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
    addDelayCheckpointAt(checkpoints, name, timestamp_ns, value);
  }

  /**
   * @brief 指定した時刻でチェックポイントを追加する
   * @param checkpoints DelayCheckpointsメッセージ
   * @param name チェックポイント名
   * @param timestamp_ns steady_clockエポック基準のタイムスタンプ（ナノ秒）
   * @param value 追加情報（オプション）
   *
   * asioスレッドで記録したUDP受信時刻など、呼び出し時点以外の時刻を刻むために使う。
   */
  static void addDelayCheckpointAt(
    DelayCheckpointsMsg & checkpoints, const std::string & name, int64_t timestamp_ns,
    const std::string & value = "")
  {
    // 最初のチェックポイントの場合、基準タイムスタンプを設定
    if (checkpoints.checkpoints.empty()) {
      checkpoints.reference_timestamp_ns = timestamp_ns;
    }

    // 同じnameのチェックポイントが存在する場合は更新
    auto existing = std::find_if(
      checkpoints.checkpoints.begin(), checkpoints.checkpoints.end(),
      [&name](const auto & checkpoint) { return checkpoint.name == name; });

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
      [&start_name](const auto & cp) { return cp.name == start_name; });

    auto end_it = std::find_if(
      checkpoints.checkpoints.begin(), checkpoints.checkpoints.end(),
      [&end_name](const auto & cp) { return cp.name == end_name; });

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
      [&end_name](const auto & cp) { return cp.name == end_name; });

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
      [&current_name](const auto & cp) { return cp.name == current_name; });

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

    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < checkpoints.checkpoints.size(); ++i) {
      const auto & cp = checkpoints.checkpoints[i];
      oss << cp.name;
      if (!cp.value.empty()) {
        oss << "(" << cp.value << ")";
      }
      if (i > 0) {
        auto delay_ms = static_cast<double>(
                          cp.relative_time_us - checkpoints.checkpoints[i - 1].relative_time_us) /
                        1000.0;
        oss << ":" << delay_ms << "ms";
      } else {
        oss << ":" << static_cast<double>(cp.relative_time_us) / 1000.0 << "ms";
      }
      if (i < checkpoints.checkpoints.size() - 1) {
        oss << ", ";
      }
    }
    oss << "]";
    return oss.str();
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
        [&src_cp](const auto & cp) { return cp.name == src_cp.name; });

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
};

/**
 * @brief 遅延監視関連メソッドを提供する CRTP ミックスイン
 * @tparam Derived 派生クラス
 *
 * 派生クラスは以下のメソッドを提供する必要がある（friend宣言推奨）:
 *   crane_msgs::msg::DelayCheckpoints & getDelayCheckpoints()
 *   const crane_msgs::msg::DelayCheckpoints & getDelayCheckpoints() const
 */
template <typename Derived>
class DelayMonitorMixin
{
  auto & checkpoints() { return static_cast<Derived &>(*this).getDelayCheckpoints(); }
  const auto & checkpoints() const
  {
    return static_cast<const Derived &>(*this).getDelayCheckpoints();
  }

public:
  auto addDelayCheckpoint(const std::string & name, const std::string & value = "") -> void
  {
    DelayMonitorWrapper::addDelayCheckpoint(checkpoints(), name, value);
  }

  auto addDelayCheckpointAt(
    const std::string & name, int64_t timestamp_ns, const std::string & value = "") -> void
  {
    DelayMonitorWrapper::addDelayCheckpointAt(checkpoints(), name, timestamp_ns, value);
  }

  auto clearDelayCheckpoints() -> void { DelayMonitorWrapper::clearCheckpoints(checkpoints()); }

  auto calculateDelayMs(const std::string & start_name, const std::string & end_name) const
    -> double
  {
    return DelayMonitorWrapper::calculateDelayMs(checkpoints(), start_name, end_name);
  }

  auto calculateTotalDelayMs(const std::string & end_name) const -> double
  {
    return DelayMonitorWrapper::calculateTotalDelayMs(checkpoints(), end_name);
  }

  auto getDelayCheckpointsString() const -> std::string
  {
    return DelayMonitorWrapper::checkpointsToString(checkpoints());
  }

  auto mergeDelayCheckpoints(const crane_msgs::msg::DelayCheckpoints & source) -> void
  {
    DelayMonitorWrapper::mergeCheckpoints(checkpoints(), source);
  }
};

}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__DELAY_MONITOR_WRAPPER_HPP_
