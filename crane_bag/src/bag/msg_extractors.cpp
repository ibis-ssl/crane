// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "msg_extractors.hpp"

#include <cmath>
#include <optional>
#include <string_view>
#include <unordered_map>

namespace crane::bag
{

namespace
{

// ─── FlatMessage アクセサユーティリティ ────────────────────────────────────────

/// FlatMessage の数値フィールド（value）を suffix でルックアップするキャッシュ。
/// 構築コストを避けるため、呼び出し元が毎回再構築せずに再利用できるよう設計。
struct FlatValueMap
{
  std::unordered_map<std::string, double> numeric;    // path -> double
  std::unordered_map<std::string, std::string> text;  // path -> string

  void build(const RosMsgParser::FlatMessage & flat)
  {
    numeric.clear();
    text.clear();
    for (const auto & [fv, var] : flat.value) {
      std::string key = fv.toStdString();
      numeric[key] = var.convert<double>();
    }
    for (const auto & [fv, s] : flat.name) {
      std::string key = fv.toStdString();
      text[key] = s;
    }
  }

  // 完全パスで各型の値を返す
  double get_d_exact(const std::string & path, double def = 0.0) const
  {
    auto it = numeric.find(path);
    return it != numeric.end() ? it->second : def;
  }
  int32_t get_i32(const std::string & path) const
  {
    return static_cast<int32_t>(get_d_exact(path));
  }
  uint32_t get_u32(const std::string & path) const
  {
    return static_cast<uint32_t>(get_d_exact(path));
  }
  float get_f(const std::string & path) const { return static_cast<float>(get_d_exact(path)); }
  uint8_t get_u8(const std::string & path) const { return static_cast<uint8_t>(get_d_exact(path)); }
  bool get_b(const std::string & path) const { return get_d_exact(path) != 0.0; }

  std::string get_s_exact(const std::string & path, const std::string & def = "") const
  {
    auto it = text.find(path);
    return it != text.end() ? it->second : def;
  }

  // keys を順に検索し最初に見つかった非ゼロ値を返す
  std::optional<double> find_first_nonzero(std::initializer_list<std::string> keys) const
  {
    for (const auto & k : keys) {
      auto it = numeric.find(k);
      if (it != numeric.end() && it->second != 0.0) return it->second;
    }
    return std::nullopt;
  }

  // keys を順に検索し最初に見つかった値を返す（ゼロ含む）
  std::optional<double> find_first(std::initializer_list<std::string> keys) const
  {
    for (const auto & k : keys) {
      auto it = numeric.find(k);
      if (it != numeric.end()) return it->second;
    }
    return std::nullopt;
  }

  // 配列の要素数をカウント（プレフィックス "/<field>." を持つキーの最大インデックス+1）
  size_t count_array(const std::string & prefix) const
  {
    size_t max_idx = 0;
    bool found = false;

    auto scan_key = [&](const std::string & k) {
      if (k.size() <= prefix.size() || k.compare(0, prefix.size(), prefix) != 0) return;
      size_t start = prefix.size();
      if (k[start] != '.') return;
      size_t end = k.find('/', start + 1);
      if (end == std::string::npos) end = k.size();
      try {
        size_t idx = std::stoull(k.substr(start + 1, end - start - 1));
        if (idx >= max_idx) max_idx = idx + 1;
        found = true;
      } catch (...) {
      }
    };
    for (const auto & [k, v] : numeric) scan_key(k);
    for (const auto & [k, v] : text) scan_key(k);
    return found ? max_idx : 0;
  }

  // 配列要素の完全パスを生成するヘルパー
  static std::string arr_path(const std::string & prefix, size_t idx, const std::string & field)
  {
    return prefix + "." + std::to_string(idx) + "/" + field;
  }
};

// ─── トピック名からプレフィックスを生成 ────────────────────────────────────────
// FlatMessage のフィールドパスは "<topic_name>/<field>" 形式
// topic_name が "/world_model" の場合 prefix は "/world_model"

std::string topic_prefix(const RosMsgParser::FlatMessage & flat)
{
  if (flat.value.empty() && flat.name.empty()) return "";
  if (!flat.value.empty()) {
    const std::string path = flat.value[0].first.toStdString();
    auto pos = path.find('/', 1);  // 先頭の / をスキップして次の / を探す
    return pos != std::string::npos ? path.substr(0, pos) : path;
  }
  const std::string path = flat.name[0].first.toStdString();
  auto pos = path.find('/', 1);
  return pos != std::string::npos ? path.substr(0, pos) : path;
}

}  // namespace

// ─── WorldModel ───────────────────────────────────────────────────────────────

WorldModel extract_world_model(const RosMsgParser::FlatMessage & flat)
{
  FlatValueMap m;
  m.build(flat);
  const std::string p = topic_prefix(flat);

  WorldModel wm;

  // ball_info
  wm.ball_info.position.x = m.get_d_exact(p + "/ball_info/position/x");
  wm.ball_info.position.y = m.get_d_exact(p + "/ball_info/position/y");
  wm.ball_info.position.z = m.get_d_exact(p + "/ball_info/position/z");
  wm.ball_info.velocity.x = m.get_d_exact(p + "/ball_info/velocity/x");
  wm.ball_info.velocity.y = m.get_d_exact(p + "/ball_info/velocity/y");

  // field_info
  wm.field_info.x = m.get_d_exact(p + "/field_info/x");
  wm.field_info.y = m.get_d_exact(p + "/field_info/y");

  // is_yellow
  wm.is_yellow = m.get_d_exact(p + "/is_yellow") != 0.0;

  auto fill_robots = [&](const std::string & prefix, std::vector<RobotInfo> & out) {
    size_t n = m.count_array(prefix);
    out.resize(n);
    for (size_t i = 0; i < n; ++i) {
      auto & r = out[i];
      r.id = static_cast<uint8_t>(m.get_d_exact(FlatValueMap::arr_path(prefix, i, "id")));
      r.pose.x = m.get_d_exact(FlatValueMap::arr_path(prefix, i, "pose/x"));
      r.pose.y = m.get_d_exact(FlatValueMap::arr_path(prefix, i, "pose/y"));
      r.pose.theta = m.get_d_exact(FlatValueMap::arr_path(prefix, i, "pose/theta"));
      r.velocity.x = m.get_d_exact(FlatValueMap::arr_path(prefix, i, "velocity/x"));
      r.velocity.y = m.get_d_exact(FlatValueMap::arr_path(prefix, i, "velocity/y"));
      r.available_vision =
        m.get_d_exact(FlatValueMap::arr_path(prefix, i, "available_vision")) != 0.0;
    }
  };
  fill_robots(p + "/robot_info_ours", wm.robot_info_ours);
  fill_robots(p + "/robot_info_theirs", wm.robot_info_theirs);

  return wm;
}

// ─── PlaySituation ────────────────────────────────────────────────────────────

PlaySituation extract_play_situation(const RosMsgParser::FlatMessage & flat)
{
  FlatValueMap m;
  m.build(flat);
  const std::string p = topic_prefix(flat);

  PlaySituation ps;
  ps.command_name = m.get_s_exact(p + "/command/name");
  ps.reason_text = m.get_s_exact(p + "/reason_text");
  return ps;
}

// ─── RobotCommands ────────────────────────────────────────────────────────────

RobotCommands extract_robot_commands(const RosMsgParser::FlatMessage & flat)
{
  FlatValueMap m;
  m.build(flat);
  const std::string p = topic_prefix(flat);

  RobotCommands rc;
  const std::string cmds_prefix = p + "/robot_commands";
  size_t n = m.count_array(cmds_prefix);
  rc.robot_commands.resize(n);

  for (size_t i = 0; i < n; ++i) {
    auto & cmd = rc.robot_commands[i];
    auto ap = [&](const std::string & f) { return FlatValueMap::arr_path(cmds_prefix, i, f); };
    cmd.robot_id = m.get_u8(ap("robot_id"));
    cmd.kick_power = m.get_f(ap("kick_power"));
    cmd.dribble_power = m.get_f(ap("dribble_power"));
    cmd.stop_flag = m.get_b(ap("stop_flag"));
    cmd.chip_enable = m.get_b(ap("chip_enable"));
    cmd.planner_name = m.get_s_exact(ap("planner_name"));

    const std::string pf_prefix = ap("planning_factors");
    cmd.planning_factors.resize(m.count_array(pf_prefix));
    for (size_t j = 0; j < cmd.planning_factors.size(); ++j) {
      cmd.planning_factors[j].name = m.get_s_exact(FlatValueMap::arr_path(pf_prefix, j, "name"));
      cmd.planning_factors[j].value = m.get_s_exact(FlatValueMap::arr_path(pf_prefix, j, "value"));
    }

    const std::string pos_prefix = ap("position_target_mode");
    cmd.position_target_mode.resize(m.count_array(pos_prefix));
    for (size_t j = 0; j < cmd.position_target_mode.size(); ++j) {
      cmd.position_target_mode[j].target_x =
        m.get_f(FlatValueMap::arr_path(pos_prefix, j, "target_x"));
      cmd.position_target_mode[j].target_y =
        m.get_f(FlatValueMap::arr_path(pos_prefix, j, "target_y"));
    }

    const std::string vel_prefix = ap("polar_velocity_target_mode");
    cmd.polar_velocity_target_mode.resize(m.count_array(vel_prefix));
    for (size_t j = 0; j < cmd.polar_velocity_target_mode.size(); ++j) {
      cmd.polar_velocity_target_mode[j].target_velocity_r =
        m.get_f(FlatValueMap::arr_path(vel_prefix, j, "target_velocity_r"));
      cmd.polar_velocity_target_mode[j].target_velocity_theta =
        m.get_f(FlatValueMap::arr_path(vel_prefix, j, "target_velocity_theta"));
    }
  }

  return rc;
}

// ─── GameAnalysis ─────────────────────────────────────────────────────────────

GameAnalysis extract_game_analysis(const RosMsgParser::FlatMessage & flat)
{
  FlatValueMap m;
  m.build(flat);
  const std::string p = topic_prefix(flat);

  GameAnalysis ga;
  ga.recommended_attacker_id = m.get_i32(p + "/recommended_attacker_id");
  ga.attacker_suitability_score = m.get_f(p + "/attacker_suitability_score");
  ga.pass_target_id = m.get_i32(p + "/pass_target_id");
  return ga;
}

// ─── RobotSelectResults ───────────────────────────────────────────────────────

RobotSelectResults extract_robot_select_results(const RosMsgParser::FlatMessage & flat)
{
  FlatValueMap m;
  m.build(flat);
  const std::string p = topic_prefix(flat);

  RobotSelectResults rsr;
  const std::string results_prefix = p + "/results";
  size_t n = m.count_array(results_prefix);
  rsr.results.resize(n);

  for (size_t i = 0; i < n; ++i) {
    auto & r = rsr.results[i];
    r.name = m.get_s_exact(FlatValueMap::arr_path(results_prefix, i, "name"));

    const std::string robots_prefix = FlatValueMap::arr_path(results_prefix, i, "selected_robots");
    size_t n_robots = m.count_array(robots_prefix);
    r.selected_robots.resize(n_robots);
    for (size_t j = 0; j < n_robots; ++j) {
      // selected_robots の要素はプリミティブ (uint8) → "selected_robots.j" にサブフィールドなし
      // rosx_introspectionはプリミティブ配列要素を "array.j" として格納する
      const std::string elem_key = robots_prefix + "." + std::to_string(j);
      auto it = m.numeric.find(elem_key);
      r.selected_robots[j] = (it != m.numeric.end()) ? static_cast<uint8_t>(it->second) : 0;
    }
  }

  return rsr;
}

// ─── LogMessage (rosout) ─────────────────────────────────────────────────────

LogMessage extract_log_message(const RosMsgParser::FlatMessage & flat)
{
  FlatValueMap m;
  m.build(flat);
  const std::string p = topic_prefix(flat);

  LogMessage log;
  log.level = m.get_u8(p + "/level");
  log.name = m.get_s_exact(p + "/name");
  log.msg = m.get_s_exact(p + "/msg");
  return log;
}

// ─── Referee ─────────────────────────────────────────────────────────────────

Referee extract_referee(const RosMsgParser::FlatMessage & flat)
{
  FlatValueMap m;
  m.build(flat);
  const std::string p = topic_prefix(flat);

  auto fill_team = [&](TeamInfo & ti, const std::string & cp) {
    ti.name = m.get_s_exact(cp + "/name");
    ti.score = m.get_u32(cp + "/score");
    ti.yellow_cards = m.get_u32(cp + "/yellow_cards");
    ti.red_cards = m.get_u32(cp + "/red_cards");
    ti.foul_counter = m.get_u32(cp + "/foul_counter");
    ti.goalkeeper = m.get_u32(cp + "/goalkeeper");
  };

  Referee ref;
  ref.command_value = m.get_i32(p + "/command/value");
  ref.command_counter = m.get_u32(p + "/command_counter");
  ref.stage_value = m.get_i32(p + "/stage/value");
  ref.has_field = m.get_u32(p + "/has_field");
  ref.current_action_time_remaining = m.get_i32(p + "/current_action_time_remaining");
  fill_team(ref.yellow, p + "/yellow");
  fill_team(ref.blue, p + "/blue");
  // designated_position（has_field にかかわらず値を読む。表示側でフラグチェック）
  ref.designated_position_x = m.get_f(p + "/designated_position/x");
  ref.designated_position_y = m.get_f(p + "/designated_position/y");
  ref.next_command_value = m.get_i32(p + "/next_command/value");

  // game_events
  const std::string ge_prefix = p + "/game_events";
  size_t n_ge = m.count_array(ge_prefix);
  ref.game_events.resize(n_ge);

  for (size_t i = 0; i < n_ge; ++i) {
    auto & ge = ref.game_events[i];
    ge.type_value = m.get_i32(FlatValueMap::arr_path(ge_prefix, i, "type/value"));

    // 全サブフィールドが存在するため、各フィールドは複数候補パスから最初の有効値を採用
    const std::string ev = FlatValueMap::arr_path(ge_prefix, i, "event");

    if (
      auto v = m.find_first_nonzero({
        ev + "/bot_crash_unique/by_team/value",
        ev + "/bot_crash_unique_skipped/by_team/value",
        ev + "/bot_too_fast_in_stop/by_team/value",
        ev + "/bot_pushed_bot/by_team/value",
        ev + "/bot_pushed_bot_skipped/by_team/value",
        ev + "/bot_dribbled_ball_too_far/by_team/value",
        ev + "/keeper_held_ball/by_team/value",
        ev + "/defender_in_defense_area/by_team/value",
        ev + "/attacker_too_close_to_defense_area/by_team/value",
        ev + "/attacker_touched_ball_in_defense_area/by_team/value",
        ev + "/bot_held_ball_deliberately/by_team/value",
        ev + "/bot_interfered_placement/by_team/value",
        ev + "/attacker_double_touched_ball/by_team/value",
        ev + "/bot_kicked_ball_too_fast/by_team/value",
        ev + "/aimless_kick/by_team/value",
      }))
      ge.by_team_value = static_cast<int32_t>(*v);

    if (
      auto v = m.find_first(
        {ev + "/bot_crash_unique/violator", ev + "/bot_crash_unique_skipped/violator"}))
      ge.violator = static_cast<uint32_t>(*v);

    if (
      auto v =
        m.find_first({ev + "/bot_crash_unique/victim", ev + "/bot_crash_unique_skipped/victim"}))
      ge.victim = static_cast<uint32_t>(*v);
    if (ge.victim == 0)
      if (
        auto v =
          m.find_first({ev + "/bot_pushed_bot/victim", ev + "/bot_pushed_bot_skipped/victim"}))
        ge.victim = static_cast<uint32_t>(*v);

    if (
      auto v = m.find_first_nonzero({
        ev + "/bot_too_fast_in_stop/by_bot",
        ev + "/bot_dribbled_ball_too_far/by_bot",
        ev + "/defender_in_defense_area/by_bot",
        ev + "/attacker_too_close_to_defense_area/by_bot",
        ev + "/attacker_touched_ball_in_defense_area/by_bot",
        ev + "/bot_held_ball_deliberately/by_bot",
        ev + "/bot_interfered_placement/by_bot",
        ev + "/attacker_double_touched_ball/by_bot",
        ev + "/bot_kicked_ball_too_fast/by_bot",
        ev + "/aimless_kick/by_bot",
      }))
      ge.by_bot = static_cast<uint32_t>(*v);

    // location（bot_crash_unique 優先、次いで各種イベント）
    for (const auto & sub : {
           std::string("bot_crash_unique"),
           std::string("bot_too_fast_in_stop"),
           std::string("bot_pushed_bot"),
           std::string("bot_pushed_bot_skipped"),
           std::string("keeper_held_ball"),
           std::string("defender_in_defense_area"),
           std::string("attacker_too_close_to_defense_area"),
           std::string("attacker_touched_ball_in_defense_area"),
           std::string("bot_held_ball_deliberately"),
           std::string("attacker_double_touched_ball"),
           std::string("aimless_kick"),
         }) {
      auto xi = m.numeric.find(ev + "/" + sub + "/location/x");
      auto yi = m.numeric.find(ev + "/" + sub + "/location/y");
      if (
        xi != m.numeric.end() &&
        (xi->second != 0.0 || (yi != m.numeric.end() && yi->second != 0.0))) {
        ge.location_x = static_cast<float>(xi->second);
        ge.location_y = yi != m.numeric.end() ? static_cast<float>(yi->second) : 0.0f;
        break;
      }
    }

    if (
      auto v = m.find_first(
        {ev + "/bot_crash_unique/crash_speed", ev + "/bot_crash_unique_skipped/crash_speed"}))
      ge.crash_speed = static_cast<float>(*v);
    if (auto v = m.find_first({ev + "/bot_too_fast_in_stop/speed"}))
      ge.speed = static_cast<float>(*v);
    if (auto v = m.find_first({ev + "/keeper_held_ball/duration"}))
      ge.duration = static_cast<float>(*v);
  }

  return ref;
}

}  // namespace crane::bag
