// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cstdio>
#include <nlohmann/json.hpp>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "bag_control.hpp"
#include "bag_events.hpp"
#include "bag_json.hpp"
#include "bag_reader.hpp"
#include "bag_referee.hpp"
#include "bag_survey.hpp"
#include "bag_tracking.hpp"

using crane::bag::analyze_control;
using crane::bag::BagReader;
using crane::bag::detect_events;
using crane::bag::detect_factor_transitions;
using crane::bag::extract_referee_transitions;
using crane::bag::format_factor_pairs;
using crane::bag::ReadOptions;
using crane::bag::RefereeSnapshot;
using crane::bag::run_survey;
using crane::bag::sample_referee;
using crane::bag::topics_for_event_types;
using crane::bag::track_ball;
using crane::bag::track_robot;

// ─── 軽量 argparse ──────────────────────────────────────────────────────────

struct Args
{
  std::string command;
  std::string bag_path;
  int robot_id = -1;
  bool ball = false;
  bool enemy = false;
  std::optional<std::pair<double, double>> time_range;
  double interval = 0.5;
  bool changes_only = false;
  std::vector<std::string> event_types;
  std::string format = "text";  // "text" or "json"
};

static std::optional<std::pair<double, double>> parse_time_range(const std::string & s)
{
  auto pos = s.find(':');
  if (pos == std::string::npos) {
    throw std::invalid_argument("時間範囲は 'start:end' 形式で指定してください: " + s);
  }
  double start = std::stod(s.substr(0, pos));
  double end = std::stod(s.substr(pos + 1));
  return std::make_pair(start, end);
}

static Args parse_args(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(
      stderr,
      "使い方: crane_bag <command> <bag_path> [options]\n"
      "コマンド: info, survey, track, events, control, referee\n");
    std::exit(1);
  }

  Args args;
  args.command = argv[1];
  args.bag_path = argv[2];

  for (int i = 3; i < argc; ++i) {
    std::string key = argv[i];
    if (key == "--enemy") {
      args.enemy = true;
    } else if (key == "--ball") {
      args.ball = true;
    } else if (key == "--changes-only") {
      args.changes_only = true;
    } else if (key == "--robot" && i + 1 < argc) {
      args.robot_id = std::stoi(argv[++i]);
    } else if (key == "--interval" && i + 1 < argc) {
      args.interval = std::stod(argv[++i]);
    } else if (key == "--time" && i + 1 < argc) {
      args.time_range = parse_time_range(argv[++i]);
    } else if (key == "--type") {
      while (i + 1 < argc && argv[i + 1][0] != '-') {
        args.event_types.push_back(argv[++i]);
      }
    } else if (key == "--format" && i + 1 < argc) {
      args.format = argv[++i];
    }
  }
  return args;
}

// ─── ユーティリティ ─────────────────────────────────────────────────────────

template <typename T>
static bool print_json(const Args & args, const T & data)
{
  if (args.format != "json") return false;
  nlohmann::json j = data;
  std::printf("%s\n", j.dump(2).c_str());
  return true;
}

static std::string format_designated_position(const RefereeSnapshot & s)
{
  if (!s.has_designated_position) return "";
  char buf[32];
  std::snprintf(buf, sizeof(buf), " dp=(%.3f,%.3f)", s.designated_x, s.designated_y);
  return buf;
}

static std::string format_next_command(const RefereeSnapshot & s)
{
  if (!s.has_next_command) return "";
  return " next=" + s.next_command_name;
}

// ─── コマンド実装 ──────────────────────────────────────────────────────────

static void cmd_info(const Args & args)
{
  auto info = BagReader::read_info(args.bag_path);

  if (print_json(args, info)) return;

  std::printf("Path: %s\n", info.path.c_str());
  std::printf("Duration: %.2fs\n", info.duration_sec);
  std::printf("Topics (%zu):\n", info.topic_counts.size());
  for (const auto & [topic, count] : info.topic_counts) {
    auto it = info.topic_types.find(topic);
    std::string type_name = (it != info.topic_types.end()) ? it->second : "unknown";
    std::printf("  %-50s %6zu msgs  [%s]\n", topic.c_str(), count, type_name.c_str());
  }
}

static void cmd_survey(const Args & args)
{
  std::fprintf(stderr, "Reading %s ...\n", args.bag_path.c_str());
  ReadOptions opts;
  // survey が参照するトピックのみ（referee は不要）。
  opts.topics = {"/play_situation", "/robot_select_results", "/world_model", "/control_targets",
                 "/robot_commands", "/game_analysis",        "/rosout"};
  // 各セクションのサンプリング間隔と一致させ、出力を変えずに展開件数を削減する。
  // robot_select_results(.back()) と rosout(全件dedup) はダウンサンプルしない。
  opts.downsample_interval_sec = {
    {"/world_model", crane::bag::kSurveySampleInterval},
    {"/robot_commands", crane::bag::kSurveyVelocityInterval},
    {"/game_analysis", crane::bag::kSurveySampleInterval}};
  auto data = BagReader::read(args.bag_path, opts);
  std::printf("%s\n", run_survey(data).c_str());
}

static void cmd_track(const Args & args)
{
  if (!args.ball && args.robot_id < 0) {
    std::fprintf(stderr, "エラー: --robot <id> または --ball が必要です\n");
    std::exit(1);
  }
  std::fprintf(stderr, "Reading %s ...\n", args.bag_path.c_str());
  ReadOptions opts;
  opts.topics = {"/world_model"};
  opts.time_range = args.time_range;
  // 読み込み時に interval 間隔へ間引く。track 側は再サンプルせず全件を通す（interval=0）。
  // read のダウンサンプルは track_ball/track_robot と同一の貪欲規則のため、
  // 「windowed ストリームへの単一の貪欲パス」となり、従来の track 単独サンプルと一致する。
  opts.downsample_interval_sec = {{"/world_model", args.interval}};
  auto data = BagReader::read(args.bag_path, opts);

  if (args.ball) {
    auto states = track_ball(data, 0.0);
    if (states.empty()) {
      std::printf("ボールのデータが見つかりません\n");
      return;
    }
    if (print_json(args, states)) return;
    std::printf("=== BALL TRACKING ===\n");
    std::printf("%8s  %8s  %8s  %8s  %8s  %8s\n", "t", "x", "y", "vx", "vy", "speed");
    for (const auto & s : states) {
      std::printf("%8.2f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f\n", s.t, s.x, s.y, s.vx, s.vy, s.speed);
    }
    return;
  }

  bool is_ours = !args.enemy;
  const char * side = is_ours ? "ours" : "enemy";

  auto states = track_robot(data, args.robot_id, is_ours, 0.0);
  if (states.empty()) {
    std::printf("robot=%d (%s) のデータが見つかりません\n", args.robot_id, side);
    return;
  }

  if (print_json(args, states)) return;

  std::printf("=== ROBOT TRACKING: robot=%d (%s) ===\n", args.robot_id, side);
  std::printf("%8s  %8s  %8s  %8s  %8s  %10s  det\n", "t", "x", "y", "theta", "speed", "dist_ball");
  for (const auto & s : states) {
    std::printf(
      "%8.2f  %8.3f  %8.3f  %8.3f  %8.3f  %10.3f  %s\n", s.t, s.x, s.y, s.theta, s.speed,
      s.dist_to_ball, s.detected ? "Y" : "N");
  }
}

static void cmd_events(const Args & args)
{
  std::fprintf(stderr, "Reading %s ...\n", args.bag_path.c_str());
  ReadOptions opts;
  // 要求された event type の検出に必要なトピックのみ読む（空 = 全タイプ）。
  opts.topics = topics_for_event_types(args.event_types);
  // goal/ball_speed は world_model を全件走査するが ball/field しか使わないため、
  // ロボット配列を破棄する高速デシリアライズを有効化する。
  opts.world_model_ball_only = opts.topics.count("/world_model") > 0;
  auto data = BagReader::read(args.bag_path, opts);
  auto events = detect_events(data, args.event_types);

  if (print_json(args, events)) return;

  if (events.empty()) {
    std::printf("イベントが検出されませんでした\n");
    return;
  }

  std::printf("=== EVENTS (%zu) ===\n", events.size());
  for (const auto & e : events) {
    std::printf("  t=%8.2f [%-15s] %s\n", e.t, e.event_type.c_str(), e.description.c_str());
  }
}

static void cmd_control(const Args & args)
{
  if (args.robot_id < 0) {
    std::fprintf(stderr, "エラー: --robot <id> が必要です\n");
    std::exit(1);
  }
  std::fprintf(stderr, "Reading %s ...\n", args.bag_path.c_str());
  ReadOptions opts;
  opts.topics = {"/control_targets"};
  opts.time_range = args.time_range;
  auto data = BagReader::read(args.bag_path, opts);

  if (args.changes_only) {
    auto transitions = detect_factor_transitions(data, args.robot_id);

    if (print_json(args, transitions)) return;

    if (transitions.empty()) {
      std::printf("robot=%d の planning_factors 変化が検出されませんでした\n", args.robot_id);
      return;
    }
    std::printf("=== PLANNING_FACTORS TRANSITIONS: robot=%d ===\n", args.robot_id);
    for (const auto & tr : transitions) {
      std::printf(
        "  t=%.2f  robot=%d: [%s] -> [%s]\n", tr.t, tr.robot_id,
        format_factor_pairs(tr.old_factors).c_str(), format_factor_pairs(tr.new_factors).c_str());
    }
  } else {
    auto snapshots = analyze_control(data, args.robot_id);

    if (print_json(args, snapshots)) return;

    if (snapshots.empty()) {
      std::printf("robot=%d のcontrol_targetデータが見つかりません\n", args.robot_id);
      return;
    }
    std::printf("=== CONTROL_TARGET: robot=%d ===\n", args.robot_id);
    for (const auto & s : snapshots) {
      std::string factors_str = format_factor_pairs(s.planning_factors);

      std::string pos_str = "N/A";
      if (s.target_x.has_value()) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "(%.3f,%.3f)", *s.target_x, *s.target_y);
        pos_str = buf;
      }
      std::string vr_str = "N/A";
      if (s.velocity_r.has_value()) {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%.3f", *s.velocity_r);
        vr_str = buf;
      }
      std::printf(
        "  t=%.2f  kick=%.2f  drb=%.2f  pos=%s  vr=%s  [%s]\n", s.t, s.kick_power, s.dribble_power,
        pos_str.c_str(), vr_str.c_str(), factors_str.c_str());
    }
  }
}

static void cmd_referee(const Args & args)
{
  std::fprintf(stderr, "Reading %s ...\n", args.bag_path.c_str());
  ReadOptions opts;
  opts.topics = {"/referee"};
  opts.time_range = args.time_range;
  auto data = BagReader::read(args.bag_path, opts);

  if (data.referees.empty()) {
    std::printf("/referee トピックが見つかりません\n");
    return;
  }

  if (args.changes_only) {
    auto transitions = extract_referee_transitions(data);

    if (print_json(args, transitions)) return;

    if (transitions.empty()) {
      std::printf("Referee コマンド遷移が検出されませんでした\n");
      return;
    }
    std::printf("=== REFEREE TRANSITIONS (%zu) ===\n", transitions.size());
    for (const auto & s : transitions) {
      std::printf(
        "  t=%7.2f  %-28s  %-26s  yellow:%s(%u) yc=%u rc=%u  blue:%s(%u) yc=%u rc=%u%s%s\n", s.t,
        s.command_name.c_str(), s.stage_name.c_str(), s.yellow.name.c_str(), s.yellow.score,
        s.yellow.yellow_cards, s.yellow.red_cards, s.blue.name.c_str(), s.blue.score,
        s.blue.yellow_cards, s.blue.red_cards, format_designated_position(s).c_str(),
        format_next_command(s).c_str());
    }
  } else {
    auto snaps = sample_referee(data, args.interval);

    if (print_json(args, snaps)) return;

    std::printf("=== REFEREE (sampled every %.1fs) ===\n", args.interval);
    for (const auto & s : snaps) {
      std::printf(
        "  t=%7.2f  %-28s  %-26s  yellow:%s(%u)  blue:%s(%u)%s%s\n", s.t, s.command_name.c_str(),
        s.stage_name.c_str(), s.yellow.name.c_str(), s.yellow.score, s.blue.name.c_str(),
        s.blue.score, format_designated_position(s).c_str(), format_next_command(s).c_str());
    }
  }
}

// ─── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(
      stderr,
      "使い方: crane_bag <command> <bag_path> [options]\n"
      "コマンド:\n"
      "  info     <path> [--format json|text]                       Bag情報を表示\n"
      "  survey   <path>                                             概要サーベイを実行\n"
      "  track    <path> --robot <id>|--ball [--enemy]              ロボット/ボール追跡\n"
      "                  [--time start:end] [--interval 0.5] [--format json|text]\n"
      "  events   <path> [--type goal kick play role ball_speed foul] [--format json|text]\n"
      "  control  <path> --robot <id> [--time start:end] [--changes-only] [--format json|text]\n"
      "  referee  <path> [--time start:end] [--changes-only] [--format json|text]\n"
      "           デフォルト: 全メッセージをサンプリング (--interval で間隔指定)\n"
      "           --changes-only: コマンド遷移のみ表示\n");
    return 1;
  }

  try {
    Args args = parse_args(argc, argv);

    if (args.command == "info") {
      cmd_info(args);
    } else if (args.command == "survey") {
      cmd_survey(args);
    } else if (args.command == "track") {
      cmd_track(args);
    } else if (args.command == "events") {
      cmd_events(args);
    } else if (args.command == "control") {
      cmd_control(args);
    } else if (args.command == "referee") {
      cmd_referee(args);
    } else {
      std::fprintf(stderr, "不明なコマンド: %s\n", args.command.c_str());
      return 1;
    }
  } catch (const std::exception & e) {
    std::fprintf(stderr, "エラー: %s\n", e.what());
    return 1;
  }

  return 0;
}
