// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "bag_reader.hpp"

#include <filesystem>
#include <mcap/reader.hpp>
#include <rosx_introspection/deserializer.hpp>
#include <rosx_introspection/ros_parser.hpp>
#include <rosx_introspection/ros_type.hpp>
#include <stdexcept>
#include <unordered_set>

#include "msg_extractors.hpp"

namespace crane::bag
{

namespace
{

/// bag_path がディレクトリの場合、その中の .mcap ファイルを探して返す。
std::string resolve_mcap_path(const std::string & bag_path)
{
  std::filesystem::path p(bag_path);
  if (std::filesystem::is_directory(p)) {
    for (const auto & entry : std::filesystem::directory_iterator(p)) {
      if (entry.path().extension() == ".mcap") {
        return entry.path().string();
      }
    }
    throw std::runtime_error("ディレクトリ内に .mcap ファイルが見つかりません: " + bag_path);
  }
  return bag_path;
}

/// mcap::Schema の data（ByteArray）を文字列に変換
std::string schema_data_to_string(const mcap::Schema & schema)
{
  return std::string(reinterpret_cast<const char *>(schema.data.data()), schema.data.size());
}

/// 対象トピックのセット
const std::unordered_set<std::string> & target_topics()
{
  static const std::unordered_set<std::string> s = {
    "/play_situation", "/world_model",          "/control_targets", "/robot_commands",
    "/game_analysis",  "/robot_select_results", "/rosout",          "/referee",
  };
  return s;
}

}  // namespace

BagInfo BagReader::read_info(const std::string & bag_path)
{
  const std::string mcap_path = resolve_mcap_path(bag_path);

  mcap::McapReader reader;
  auto status = reader.open(mcap_path);
  if (!status.ok()) {
    throw std::runtime_error(
      "mcapファイルを開けません: " + mcap_path + " (" + status.message + ")");
  }

  (void)reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);

  BagInfo info;
  info.path = bag_path;

  // 統計情報からタイムスタンプを取得
  const auto & stats = reader.statistics();
  if (stats.has_value()) {
    info.start_time_ns = static_cast<int64_t>(stats->messageStartTime);
    int64_t end_ns = static_cast<int64_t>(stats->messageEndTime);
    info.end_time_ns = end_ns;
    info.duration_sec = static_cast<double>(end_ns - info.start_time_ns) / 1e9;
  }

  // トピックごとのメッセージ数と型名を収集
  const auto & channels = reader.channels();
  const auto & schemas = reader.schemas();

  for (const auto & [ch_id, ch] : channels) {
    info.topic_counts[ch->topic] = 0;  // 詳細カウントは summary から
    auto it = schemas.find(ch->schemaId);
    if (it != schemas.end()) {
      info.topic_types[ch->topic] = it->second->name;
    }
  }

  // メッセージ数はsummaryのチャンネル統計から取得
  if (stats.has_value()) {
    for (const auto & [ch_id, count] : stats->channelMessageCounts) {
      auto ch_it = channels.find(ch_id);
      if (ch_it != channels.end()) {
        info.topic_counts[ch_it->second->topic] += count;
      }
    }
  }

  reader.close();
  return info;
}

BagData BagReader::read(
  const std::string & bag_path, std::optional<std::pair<double, double>> time_range)
{
  const std::string mcap_path = resolve_mcap_path(bag_path);

  mcap::McapReader reader;
  auto open_status = reader.open(mcap_path);
  if (!open_status.ok()) {
    throw std::runtime_error(
      "mcapファイルを開けません: " + mcap_path + " (" + open_status.message + ")");
  }

  (void)reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);

  BagData data;

  // --- BagInfo を読み込み ---
  const auto & stats = reader.statistics();
  if (stats.has_value()) {
    data.info.start_time_ns = static_cast<int64_t>(stats->messageStartTime);
    int64_t end_ns = static_cast<int64_t>(stats->messageEndTime);
    data.info.end_time_ns = end_ns;
    data.info.duration_sec = static_cast<double>(end_ns - data.info.start_time_ns) / 1e9;
    for (const auto & [ch_id, count] : stats->channelMessageCounts) {
      auto ch_it = reader.channels().find(ch_id);
      if (ch_it != reader.channels().end()) {
        data.info.topic_counts[ch_it->second->topic] += count;
      }
    }
  }
  data.info.path = bag_path;

  // トピック型名を収集
  for (const auto & [ch_id, ch] : reader.channels()) {
    auto it = reader.schemas().find(ch->schemaId);
    if (it != reader.schemas().end()) {
      data.info.topic_types[ch->topic] = it->second->name;
    }
  }

  // --- rosx_introspection パーサを構築 ---
  // 対象トピックのチャンネルに対してのみパーサを登録
  RosMsgParser::ParsersCollection<RosMsgParser::FastCDR_Deserializer> parsers;

  for (const auto & [ch_id, ch] : reader.channels()) {
    if (target_topics().find(ch->topic) == target_topics().end()) continue;

    auto schema_it = reader.schemas().find(ch->schemaId);
    if (schema_it == reader.schemas().end()) continue;

    const auto & schema = *schema_it->second;
    if (schema.encoding != "ros2msg") continue;

    std::string definition = schema_data_to_string(schema);
    parsers.registerParser(ch->topic, RosMsgParser::ROSType(schema.name), definition);
  }

  // --- 時間範囲フィルタを設定 ---
  mcap::Timestamp start_ts = 0;
  mcap::Timestamp end_ts = mcap::MaxTime;
  if (time_range && stats.has_value()) {
    int64_t bag_start = data.info.start_time_ns;
    start_ts =
      static_cast<mcap::Timestamp>(bag_start + static_cast<int64_t>(time_range->first * 1e9));
    end_ts =
      static_cast<mcap::Timestamp>(bag_start + static_cast<int64_t>(time_range->second * 1e9));
  }

  // --- メッセージをワンパスで読み込み ---
  const auto & targets = target_topics();
  for (const auto & view : reader.readMessages(start_ts, end_ts)) {
    const std::string & topic = view.channel->topic;
    if (targets.find(topic) == targets.end()) continue;

    const uint8_t * raw = reinterpret_cast<const uint8_t *>(view.message.data);
    size_t raw_size = view.message.dataSize;

    // ROS2 CDR encapsulation header（4バイト: 00 01 00 00）をスキップ
    if (raw_size < 4) continue;
    RosMsgParser::Span<const uint8_t> buf(raw + 4, raw_size - 4);

    const RosMsgParser::FlatMessage * flat = parsers.deserialize(topic, buf);
    if (!flat) continue;

    int64_t ts = static_cast<int64_t>(view.message.logTime);

    try {
      if (topic == "/play_situation") {
        data.play_situations.push_back({ts, extract_play_situation(*flat)});
      } else if (topic == "/world_model") {
        data.world_models.push_back({ts, extract_world_model(*flat)});
      } else if (topic == "/control_targets") {
        data.control_targets.push_back({ts, extract_robot_commands(*flat)});
      } else if (topic == "/robot_commands") {
        data.robot_commands.push_back({ts, extract_robot_commands(*flat)});
      } else if (topic == "/game_analysis") {
        data.game_analyses.push_back({ts, extract_game_analysis(*flat)});
      } else if (topic == "/robot_select_results") {
        data.robot_select_results.push_back({ts, extract_robot_select_results(*flat)});
      } else if (topic == "/rosout") {
        data.rosout.push_back({ts, extract_log_message(*flat)});
      } else if (topic == "/referee") {
        data.referees.push_back({ts, extract_referee(*flat)});
      }
    } catch (const std::exception &) {
      // デシリアライズ失敗は静かにスキップ（古いbagでのフィールド不足等）
    }
  }

  reader.close();
  return data;
}

}  // namespace crane::bag
