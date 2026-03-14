// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <chrono>
#include <crane_msgs/msg/game_analysis.hpp>
#include <crane_msgs/msg/play_situation.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/robot_select_results.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <limits>
#include <map>
#include <optional>
#include <rcl_interfaces/msg/log.hpp>
#include <string>
#include <utility>
#include <vector>

namespace crane::bag
{

template <typename MsgT>
struct TimestampedMsg
{
  int64_t timestamp_ns;
  MsgT msg;

  double t(int64_t bag_start_ns) const
  {
    return static_cast<double>(timestamp_ns - bag_start_ns) / 1e9;
  }
};

struct BagInfo
{
  std::string path;
  double duration_sec;
  int64_t start_time_ns;
  int64_t end_time_ns;
  std::map<std::string, size_t> topic_counts;
  std::map<std::string, std::string> topic_types;
};

struct BagData
{
  BagInfo info;
  std::vector<TimestampedMsg<crane_msgs::msg::PlaySituation>> play_situations;
  std::vector<TimestampedMsg<crane_msgs::msg::WorldModel>> world_models;
  std::vector<TimestampedMsg<crane_msgs::msg::RobotCommands>> control_targets;
  std::vector<TimestampedMsg<crane_msgs::msg::RobotCommands>> robot_commands;
  std::vector<TimestampedMsg<crane_msgs::msg::GameAnalysis>> game_analyses;
  std::vector<TimestampedMsg<crane_msgs::msg::RobotSelectResults>> robot_select_results;
  std::vector<TimestampedMsg<rcl_interfaces::msg::Log>> rosout;

  /// 指定間隔でサンプリングしたポインタ列を返す
  template <typename MsgT>
  static std::vector<const TimestampedMsg<MsgT> *> sample(
    const std::vector<TimestampedMsg<MsgT>> & msgs, double interval_sec)
  {
    std::vector<const TimestampedMsg<MsgT> *> result;
    int64_t interval_ns = static_cast<int64_t>(interval_sec * 1e9);
    int64_t last_ns = 0;  // タイムスタンプはエポックからのns（>>0）なので最初は必ず通過
    for (const auto & m : msgs) {
      if (m.timestamp_ns - last_ns >= interval_ns) {
        result.push_back(&m);
        last_ns = m.timestamp_ns;
      }
    }
    return result;
  }
};

/// time_range（相対秒）を絶対ナノ秒フィルタ範囲に変換するヘルパー
inline std::pair<int64_t, int64_t> make_ns_range(
  int64_t bag_start_ns, const std::optional<std::pair<double, double>> & time_range)
{
  if (!time_range) {
    return {std::numeric_limits<int64_t>::min(), std::numeric_limits<int64_t>::max()};
  }
  return {
    bag_start_ns + static_cast<int64_t>(time_range->first * 1e9),
    bag_start_ns + static_cast<int64_t>(time_range->second * 1e9)};
}

class BagReader
{
public:
  /// メタデータのみ取得（メッセージ読み込みなし）
  static BagInfo read_info(const std::string & bag_path);

  /// 全トピックをワンパスで読み込む
  static BagData read(
    const std::string & bag_path,
    std::optional<std::pair<double, double>> time_range = std::nullopt);
};

}  // namespace crane::bag
