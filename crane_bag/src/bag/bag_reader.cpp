// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "bag_reader.hpp"

#include <filesystem>
#include <mcap/reader.hpp>
#include <optional>
#include <regex>
#include <rosx_introspection/deserializer.hpp>
#include <rosx_introspection/ros_parser.hpp>
#include <rosx_introspection/ros_type.hpp>
#include <stdexcept>
#include <unordered_map>
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
    "/play_situation",       "/world_model",   "/control_targets",
    "/robot_commands",       "/game_analysis", "/rosout",
    "/robot_select_results", "/referee",       "/kick_prediction_traces",
  };
  return s;
}

/// readSummary 済みの McapReader から BagInfo を構築する共通ロジック
BagInfo populate_bag_info(const std::string & bag_path, const mcap::McapReader & reader)
{
  BagInfo info;
  info.path = bag_path;

  const auto & stats = reader.statistics();
  if (stats.has_value()) {
    info.start_time_ns = static_cast<int64_t>(stats->messageStartTime);
    int64_t end_ns = static_cast<int64_t>(stats->messageEndTime);
    info.end_time_ns = end_ns;
    info.duration_sec = static_cast<double>(end_ns - info.start_time_ns) / 1e9;
    for (const auto & [ch_id, count] : stats->channelMessageCounts) {
      auto ch_it = reader.channels().find(ch_id);
      if (ch_it != reader.channels().end()) {
        info.topic_counts[ch_it->second->topic] += count;
      }
    }
  }

  for (const auto & [ch_id, ch] : reader.channels()) {
    auto it = reader.schemas().find(ch->schemaId);
    if (it != reader.schemas().end()) {
      info.topic_types[ch->topic] = it->second->name;
    }
  }

  return info;
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
  BagInfo info = populate_bag_info(bag_path, reader);
  reader.close();
  return info;
}

BagData BagReader::read(const std::string & bag_path, const ReadOptions & opts)
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
  data.info = populate_bag_info(bag_path, reader);

  // --- rosx_introspection パーサを構築 ---
  // readSummary() のスキーマID対応は bagによっては不正確なため、
  // メッセージイテレーション中に view.schema から直接登録する。
  RosMsgParser::ParsersCollection<RosMsgParser::ROS2_Deserializer> parsers;
  std::unordered_set<std::string> registered_topics;

  // /world_model の ball-only 用パーサ（ロボット配列を破棄する MaxArrayPolicy を適用）。
  // ParsersCollection は既定ポリシーで生成されるため、専用 Parser を別管理する。
  std::optional<RosMsgParser::Parser> wm_ball_only_parser;
  RosMsgParser::FlatMessage wm_ball_only_flat;
  RosMsgParser::ROS2_Deserializer wm_ball_only_deser;
  bool wm_ball_only_registered = false;

  // --- 読み込み対象トピック（空指定なら全デフォルトターゲット）---
  const std::unordered_set<std::string> & targets =
    opts.topics.empty() ? target_topics() : opts.topics;

  // --- トピックごとの読み込み時ダウンサンプル状態 ---
  std::unordered_map<std::string, int64_t> last_kept_ns;

  // --- 時間範囲フィルタを設定 ---
  mcap::Timestamp start_ts = 0;
  mcap::Timestamp end_ts = mcap::MaxTime;
  if (opts.time_range && data.info.start_time_ns != 0) {
    int64_t bag_start = data.info.start_time_ns;
    start_ts =
      static_cast<mcap::Timestamp>(bag_start + static_cast<int64_t>(opts.time_range->first * 1e9));
    end_ts =
      static_cast<mcap::Timestamp>(bag_start + static_cast<int64_t>(opts.time_range->second * 1e9));
  }

  // --- メッセージをワンパスで読み込み ---
  for (const auto & view : reader.readMessages(start_ts, end_ts)) {
    const std::string & topic = view.channel->topic;
    if (targets.find(topic) == targets.end()) continue;

    const int64_t ts = static_cast<int64_t>(view.message.logTime);

    // 読み込み時ダウンサンプル（BagData::sample と同一の貪欲規則: last 初期値0）。
    // last_kept は「デシリアライズに成功して格納したフレーム」でのみ進める（下のpush後）。
    // 旧実装はサンプリングを格納済みフレームに対して行うため、デシリアライズに失敗する
    // フレームがあっても採用フレームが一致するようにする（生ストリーム基準で進めない）。
    bool ds_candidate = false;
    {
      auto di = opts.downsample_interval_sec.find(topic);
      if (di != opts.downsample_interval_sec.end() && di->second > 0.0) {
        const int64_t interval_ns = static_cast<int64_t>(di->second * 1e9);
        auto lk = last_kept_ns.find(topic);
        const int64_t last = (lk != last_kept_ns.end()) ? lk->second : 0;
        if (ts - last < interval_ns) continue;  // 間隔未満はデシリアライズせずスキップ
        ds_candidate = true;                    // 採用候補。格納成功後に last_kept を進める
      }
    }

    const bool ball_only_wm = opts.world_model_ball_only && topic == "/world_model";

    // 初回出現時にパーサを登録（view.schema はチャンネルに紐付く正確なスキーマ）
    auto build_definition = [&]() {
      std::string definition = schema_data_to_string(*view.schema);
      // rosx_introspection は bounded sequence 記法 [<=N] を isArray=false に誤パースする。
      // [] (unbounded) に正規化して可変長シーケンスとして正しく扱わせる。
      return std::regex_replace(definition, std::regex(R"(\[<=\d+\])"), "[]");
    };
    if (ball_only_wm) {
      if (!wm_ball_only_registered && view.schema && view.schema->encoding == "ros2msg") {
        wm_ball_only_parser.emplace(
          topic, RosMsgParser::ROSType(view.schema->name), build_definition());
        // ロボット配列（要素数 > 1）を破棄し、ball_info/field_info 等のスカラのみ抽出する。
        wm_ball_only_parser->setMaxArrayPolicy(RosMsgParser::Parser::DISCARD_LARGE_ARRAYS, 1);
        wm_ball_only_registered = true;
      }
    } else if (registered_topics.find(topic) == registered_topics.end()) {
      if (view.schema && view.schema->encoding == "ros2msg") {
        parsers.registerParser(topic, RosMsgParser::ROSType(view.schema->name), build_definition());
        registered_topics.insert(topic);
      }
    }

    const uint8_t * raw = reinterpret_cast<const uint8_t *>(view.message.data);
    size_t raw_size = view.message.dataSize;
    if (raw_size == 0) continue;

    // ROS2_Deserializer (CDR) が encapsulation header（00 01 00 00）を自前で読み込む。
    // スキップせずそのまま渡す。
    RosMsgParser::Span<const uint8_t> buf(raw, raw_size);

    try {
      const RosMsgParser::FlatMessage * flat = nullptr;
      if (ball_only_wm) {
        if (!wm_ball_only_parser) continue;
        // 配列破棄により false が返るが、ball_info/field_info は抽出済み。
        wm_ball_only_parser->deserialize(buf, &wm_ball_only_flat, &wm_ball_only_deser);
        flat = &wm_ball_only_flat;
      } else {
        flat = parsers.deserialize(topic, buf);
      }
      if (!flat) continue;

      // opts.topics に含めるトピックは必ず下のいずれかの分岐で push すること。
      // 分岐の無いトピックを追加すると、格納せずに ds_candidate のカーソルだけ進み
      // ダウンサンプルの整合性が崩れる。
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
      } else if (topic == "/kick_prediction_traces") {
        data.kick_prediction_traces.push_back({ts, extract_kick_prediction_trace(*flat)});
      }

      // 格納に成功したフレームでのみダウンサンプルのカーソルを進める。
      if (ds_candidate) last_kept_ns[topic] = ts;
    } catch (const std::exception &) {
      // デシリアライズ失敗は静かにスキップ（古いbagでのフィールド不足等）。
      // last_kept は進めないため、次フレームが同じ間隔基準で再評価される。
    }
  }

  reader.close();
  return data;
}

}  // namespace crane::bag
