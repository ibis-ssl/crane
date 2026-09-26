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
#include <string>
#include <vector>

namespace crane
{

/**
 * @brief 遅延チェックポイント情報を管理するユーティリティクラス
 */
class DelayMonitorWrapper
{
public:
  using DelayCheckpointMsg = crane_msgs::msg::DelayCheckpoint;
  using DelayCheckpointsMsg = crane_msgs::msg::DelayCheckpoints;

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
    if (checkpoints.checkpoints.empty()) {
      checkpoints.reference_timestamp_ns = timestamp_ns;
    }

    // 同じnameのチェックポイントが存在する場合は更新
    auto existing = std::find_if(
      checkpoints.checkpoints.begin(), checkpoints.checkpoints.end(),
      [&name](const auto & checkpoint) { return checkpoint.name == name; });

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
};

/**
 * @brief 遅延監視関連メソッドを提供する CRTP ミックスイン
 * @tparam Derived 派生クラス
 *
 * 派生クラスは以下のメソッドを提供する必要がある（friend宣言推奨）:
 *   crane_msgs::msg::DelayCheckpoints & getDelayCheckpoints()
 */
template <typename Derived>
class DelayMonitorMixin
{
  auto & checkpoints() { return static_cast<Derived &>(*this).getDelayCheckpoints(); }

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
};

}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__DELAY_MONITOR_WRAPPER_HPP_
