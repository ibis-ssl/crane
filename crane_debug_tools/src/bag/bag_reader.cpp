// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "bag_reader.hpp"

#include <filesystem>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <unordered_set>

namespace crane::bag
{

namespace
{

/// bag_path がディレクトリの場合、その中の .mcap ファイルを探して返す。
std::string resolve_bag_uri(const std::string & bag_path)
{
  std::filesystem::path p(bag_path);
  if (std::filesystem::is_directory(p)) {
    for (const auto & entry : std::filesystem::directory_iterator(p)) {
      if (entry.path().extension() == ".mcap") {
        return p.string();
      }
    }
  }
  return bag_path;
}

rosbag2_storage::StorageOptions make_storage_options(const std::string & uri)
{
  rosbag2_storage::StorageOptions opts;
  opts.uri = uri;
  return opts;
}

template <typename MsgT>
void deserialize_and_push(
  const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & bag_message,
  std::vector<TimestampedMsg<MsgT>> & out)
{
  rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
  rclcpp::Serialization<MsgT> ser;
  MsgT msg;
  ser.deserialize_message(&serialized, &msg);
  out.push_back({bag_message->recv_timestamp, std::move(msg)});
}

/// 対象トピックのセット（bag_reader.cpp 内で共有）
const std::unordered_set<std::string> & target_topics()
{
  static const std::unordered_set<std::string> s = {
    "/play_situation", "/world_model",          "/control_targets", "/robot_commands",
    "/game_analysis",  "/robot_select_results", "/rosout",          "/referee",
  };
  return s;
}

/// BagMetadata からメタ情報を収集する共通処理
BagInfo collect_bag_info(
  const std::string & bag_path, const rosbag2_storage::BagMetadata & meta,
  const std::vector<rosbag2_storage::TopicMetadata> & topic_types_vec)
{
  std::map<std::string, std::string> topic_types;
  for (const auto & t : topic_types_vec) {
    topic_types[t.name] = t.type;
  }

  std::map<std::string, size_t> topic_counts;
  for (const auto & t : meta.topics_with_message_count) {
    topic_counts[t.topic_metadata.name] = t.message_count;
  }

  int64_t start_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(meta.starting_time.time_since_epoch())
      .count();
  int64_t duration_ns = meta.duration.count();

  return BagInfo{
    bag_path,
    static_cast<double>(duration_ns) / 1e9,
    start_ns,
    start_ns + duration_ns,
    std::move(topic_counts),
    std::move(topic_types),
  };
}

}  // namespace

BagInfo BagReader::read_info(const std::string & bag_path)
{
  std::string uri = resolve_bag_uri(bag_path);
  rosbag2_cpp::Reader reader;
  reader.open(make_storage_options(uri));
  // get_metadata() はメッセージを読まずにメタデータYAMLから取得
  return collect_bag_info(bag_path, reader.get_metadata(), reader.get_all_topics_and_types());
}

BagData BagReader::read(
  const std::string & bag_path, std::optional<std::pair<double, double>> time_range)
{
  std::string uri = resolve_bag_uri(bag_path);

  // メタデータからBagInfoを取得（メッセージ読み込みなし）
  rosbag2_cpp::Reader meta_reader;
  meta_reader.open(make_storage_options(uri));
  BagInfo bag_info =
    collect_bag_info(bag_path, meta_reader.get_metadata(), meta_reader.get_all_topics_and_types());
  // meta_reader はここでスコープを抜けてclose

  // 時間範囲フィルタをストレージレベルで指定（読み込みループ外で制限）
  auto storage_opts = make_storage_options(uri);
  if (time_range) {
    auto [ns_start, ns_end] = make_ns_range(bag_info.start_time_ns, time_range);
    storage_opts.start_time_ns = ns_start;
    storage_opts.end_time_ns = ns_end;
  }

  rosbag2_cpp::Reader reader;
  reader.open(storage_opts);

  // トピックフィルタ（7トピック以外はストレージレベルでスキップ）
  rosbag2_storage::StorageFilter filter;
  filter.topics = std::vector<std::string>(target_topics().begin(), target_topics().end());
  reader.set_filter(filter);

  BagData data;
  data.info = std::move(bag_info);

  // ワンパスで直接デシリアライズ（RawMsg バッファなし）
  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    const auto & topic = bag_msg->topic_name;
    if (topic == "/play_situation") {
      deserialize_and_push(bag_msg, data.play_situations);
    } else if (topic == "/world_model") {
      deserialize_and_push(bag_msg, data.world_models);
    } else if (topic == "/control_targets") {
      deserialize_and_push(bag_msg, data.control_targets);
    } else if (topic == "/robot_commands") {
      deserialize_and_push(bag_msg, data.robot_commands);
    } else if (topic == "/game_analysis") {
      deserialize_and_push(bag_msg, data.game_analyses);
    } else if (topic == "/robot_select_results") {
      deserialize_and_push(bag_msg, data.robot_select_results);
    } else if (topic == "/rosout") {
      deserialize_and_push(bag_msg, data.rosout);
    } else if (topic == "/referee") {
      deserialize_and_push(bag_msg, data.referees);
    }
  }

  return data;
}

}  // namespace crane::bag
