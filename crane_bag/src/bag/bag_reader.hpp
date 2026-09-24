// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <limits>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "bag_types.hpp"

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
  double duration_sec = 0.0;
  int64_t start_time_ns = 0;
  int64_t end_time_ns = 0;
  std::map<std::string, size_t> topic_counts;
  std::map<std::string, std::string> topic_types;
};

struct BagData
{
  BagInfo info;
  std::vector<TimestampedMsg<PlaySituation>> play_situations;
  std::vector<TimestampedMsg<WorldModel>> world_models;
  std::vector<TimestampedMsg<RobotCommands>> control_targets;
  std::vector<TimestampedMsg<RobotCommands>> robot_commands;
  std::vector<TimestampedMsg<GameAnalysis>> game_analyses;
  std::vector<TimestampedMsg<RobotSelectResults>> robot_select_results;
  std::vector<TimestampedMsg<LogMessage>> rosout;
  std::vector<TimestampedMsg<Referee>> referees;
  std::vector<TimestampedMsg<KickPredictionTraceData>> kick_prediction_traces;

  /// 指定間隔でサンプリングしたポインタ列を返す
  template <typename MsgT>
  static std::vector<const TimestampedMsg<MsgT> *> sample(
    const std::vector<TimestampedMsg<MsgT>> & msgs, double interval_sec)
  {
    std::vector<const TimestampedMsg<MsgT> *> result;
    int64_t interval_ns = static_cast<int64_t>(interval_sec * 1e9);
    int64_t last_ns = 0;
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

/// read() の読み込み挙動を制御するオプション。
/// 各コマンドが必要とするトピック/サンプリングだけをデシリアライズすることで、
/// 巨大な /world_model 等の無駄な展開を避ける。
struct ReadOptions
{
  /// デシリアライズ対象トピック。空 = 既存の全デフォルトターゲット（後方互換）。
  std::unordered_set<std::string> topics;

  /// 相対秒の時間範囲フィルタ。
  std::optional<std::pair<double, double>> time_range;

  /// トピックごとの最小サンプリング間隔[秒]。
  /// ts - last_kept_ns >= interval*1e9 を満たすメッセージのみデシリアライズする
  /// （last_kept 初期値 0、貪欲規則は BagData::sample と同一）。
  /// 未登録/0 以下 = 全件。track/survey の読み込み時ダウンサンプル用。
  std::unordered_map<std::string, double> downsample_interval_sec;

  /// /world_model を ball_info/field_info のみ抽出し、ロボット配列を破棄する高速モード。
  /// goal/ball_speed イベント検出など、ロボット情報が不要な経路で使う。
  bool world_model_ball_only = false;
};

class BagReader
{
public:
  /// メタデータのみ取得（メッセージ読み込みなし）
  static BagInfo read_info(const std::string & bag_path);

  /// 必要なトピック/サンプリングをワンパスで読み込む（既定 = 全ターゲット全件）
  static BagData read(const std::string & bag_path, const ReadOptions & opts = {});
};

}  // namespace crane::bag
