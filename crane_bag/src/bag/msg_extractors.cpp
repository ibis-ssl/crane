// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "msg_extractors.hpp"

#include <cmath>
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

  // サフィックスが一致する最初の数値を返す（トピック名プレフィックスを無視）
  double get_d(const std::string & suffix, double def = 0.0) const
  {
    for (const auto & [k, v] : numeric) {
      if (
        k.size() >= suffix.size() &&
        k.compare(k.size() - suffix.size(), suffix.size(), suffix) == 0) {
        return v;
      }
    }
    return def;
  }

  // 完全パス（先頭の /topic/ から始まる）で数値を返す
  double get_d_exact(const std::string & path, double def = 0.0) const
  {
    auto it = numeric.find(path);
    return it != numeric.end() ? it->second : def;
  }

  // 完全パスで文字列を返す
  std::string get_s_exact(const std::string & path, const std::string & def = "") const
  {
    auto it = text.find(path);
    return it != text.end() ? it->second : def;
  }

  // サフィックスが一致する最初の文字列を返す
  std::string get_s(const std::string & suffix, const std::string & def = "") const
  {
    for (const auto & [k, v] : text) {
      if (
        k.size() >= suffix.size() &&
        k.compare(k.size() - suffix.size(), suffix.size(), suffix) == 0) {
        return v;
      }
    }
    return def;
  }

  // 配列の要素数をカウント（プレフィックス "/<field>." を持つキーの最大インデックス+1）
  size_t count_array(const std::string & prefix) const
  {
    size_t max_idx = 0;
    bool found = false;
    for (const auto & [k, v] : numeric) {
      if (k.size() > prefix.size() && k.compare(0, prefix.size(), prefix) == 0) {
        // prefix の次の文字から数字を読む
        size_t dot_pos = k.find('.', prefix.size());
        if (dot_pos == std::string::npos) continue;
        // prefix は "<topic>/array_field" 形式 → 次は ".<idx>/<subfield>"
        // 実際のパスは "<topic>/array_field.N/<subfield>"
        size_t start = prefix.size();
        if (k[start] != '.') continue;
        size_t end = k.find('/', start + 1);
        if (end == std::string::npos) end = k.size();
        size_t idx = 0;
        try {
          idx = std::stoull(k.substr(start + 1, end - start - 1));
        } catch (...) {
          continue;
        }
        if (idx >= max_idx) max_idx = idx + 1;
        found = true;
      }
    }
    // textにも配列要素があるかチェック
    for (const auto & [k, v] : text) {
      if (k.size() > prefix.size() && k.compare(0, prefix.size(), prefix) == 0) {
        size_t start = prefix.size();
        if (k[start] != '.') continue;
        size_t end = k.find('/', start + 1);
        if (end == std::string::npos) end = k.size();
        size_t idx = 0;
        try {
          idx = std::stoull(k.substr(start + 1, end - start - 1));
        } catch (...) {
          continue;
        }
        if (idx >= max_idx) max_idx = idx + 1;
        found = true;
      }
    }
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

  // robot_info_ours
  const std::string ours_prefix = p + "/robot_info_ours";
  size_t n_ours = m.count_array(ours_prefix);
  wm.robot_info_ours.resize(n_ours);
  for (size_t i = 0; i < n_ours; ++i) {
    auto & r = wm.robot_info_ours[i];
    r.id = static_cast<uint8_t>(m.get_d_exact(FlatValueMap::arr_path(ours_prefix, i, "id")));
    r.pose.x = m.get_d_exact(FlatValueMap::arr_path(ours_prefix, i, "pose/x"));
    r.pose.y = m.get_d_exact(FlatValueMap::arr_path(ours_prefix, i, "pose/y"));
    r.pose.theta = m.get_d_exact(FlatValueMap::arr_path(ours_prefix, i, "pose/theta"));
    r.velocity.x = m.get_d_exact(FlatValueMap::arr_path(ours_prefix, i, "velocity/x"));
    r.velocity.y = m.get_d_exact(FlatValueMap::arr_path(ours_prefix, i, "velocity/y"));
    r.available_vision =
      m.get_d_exact(FlatValueMap::arr_path(ours_prefix, i, "available_vision")) != 0.0;
  }

  // robot_info_theirs
  const std::string theirs_prefix = p + "/robot_info_theirs";
  size_t n_theirs = m.count_array(theirs_prefix);
  wm.robot_info_theirs.resize(n_theirs);
  for (size_t i = 0; i < n_theirs; ++i) {
    auto & r = wm.robot_info_theirs[i];
    r.id = static_cast<uint8_t>(m.get_d_exact(FlatValueMap::arr_path(theirs_prefix, i, "id")));
    r.pose.x = m.get_d_exact(FlatValueMap::arr_path(theirs_prefix, i, "pose/x"));
    r.pose.y = m.get_d_exact(FlatValueMap::arr_path(theirs_prefix, i, "pose/y"));
    r.pose.theta = m.get_d_exact(FlatValueMap::arr_path(theirs_prefix, i, "pose/theta"));
    r.velocity.x = m.get_d_exact(FlatValueMap::arr_path(theirs_prefix, i, "velocity/x"));
    r.velocity.y = m.get_d_exact(FlatValueMap::arr_path(theirs_prefix, i, "velocity/y"));
    r.available_vision =
      m.get_d_exact(FlatValueMap::arr_path(theirs_prefix, i, "available_vision")) != 0.0;
  }

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
    cmd.robot_id =
      static_cast<uint8_t>(m.get_d_exact(FlatValueMap::arr_path(cmds_prefix, i, "robot_id")));
    cmd.kick_power =
      static_cast<float>(m.get_d_exact(FlatValueMap::arr_path(cmds_prefix, i, "kick_power")));
    cmd.dribble_power =
      static_cast<float>(m.get_d_exact(FlatValueMap::arr_path(cmds_prefix, i, "dribble_power")));
    cmd.stop_flag = m.get_d_exact(FlatValueMap::arr_path(cmds_prefix, i, "stop_flag")) != 0.0;
    cmd.chip_enable = m.get_d_exact(FlatValueMap::arr_path(cmds_prefix, i, "chip_enable")) != 0.0;
    cmd.planner_name = m.get_s_exact(FlatValueMap::arr_path(cmds_prefix, i, "planner_name"));

    // planning_factors
    const std::string pf_prefix = FlatValueMap::arr_path(cmds_prefix, i, "planning_factors");
    size_t n_pf = m.count_array(pf_prefix);
    cmd.planning_factors.resize(n_pf);
    for (size_t j = 0; j < n_pf; ++j) {
      cmd.planning_factors[j].name = m.get_s_exact(FlatValueMap::arr_path(pf_prefix, j, "name"));
      cmd.planning_factors[j].value = m.get_s_exact(FlatValueMap::arr_path(pf_prefix, j, "value"));
    }

    // position_target_mode
    const std::string pos_prefix = FlatValueMap::arr_path(cmds_prefix, i, "position_target_mode");
    size_t n_pos = m.count_array(pos_prefix);
    cmd.position_target_mode.resize(n_pos);
    for (size_t j = 0; j < n_pos; ++j) {
      cmd.position_target_mode[j].target_x =
        static_cast<float>(m.get_d_exact(FlatValueMap::arr_path(pos_prefix, j, "target_x")));
      cmd.position_target_mode[j].target_y =
        static_cast<float>(m.get_d_exact(FlatValueMap::arr_path(pos_prefix, j, "target_y")));
    }

    // polar_velocity_target_mode
    const std::string vel_prefix =
      FlatValueMap::arr_path(cmds_prefix, i, "polar_velocity_target_mode");
    size_t n_vel = m.count_array(vel_prefix);
    cmd.polar_velocity_target_mode.resize(n_vel);
    for (size_t j = 0; j < n_vel; ++j) {
      cmd.polar_velocity_target_mode[j].target_velocity_r = static_cast<float>(
        m.get_d_exact(FlatValueMap::arr_path(vel_prefix, j, "target_velocity_r")));
      cmd.polar_velocity_target_mode[j].target_velocity_theta = static_cast<float>(
        m.get_d_exact(FlatValueMap::arr_path(vel_prefix, j, "target_velocity_theta")));
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
  ga.recommended_attacker_id = static_cast<int32_t>(m.get_d_exact(p + "/recommended_attacker_id"));
  ga.attacker_suitability_score =
    static_cast<float>(m.get_d_exact(p + "/attacker_suitability_score"));
  ga.pass_target_id = static_cast<int32_t>(m.get_d_exact(p + "/pass_target_id"));
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
  log.level = static_cast<uint8_t>(m.get_d_exact(p + "/level"));
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

  Referee ref;
  ref.command_value = static_cast<int32_t>(m.get_d_exact(p + "/command/value"));
  ref.command_counter = static_cast<uint32_t>(m.get_d_exact(p + "/command_counter"));
  ref.stage_value = static_cast<int32_t>(m.get_d_exact(p + "/stage/value"));
  ref.has_field = static_cast<uint32_t>(m.get_d_exact(p + "/has_field"));
  ref.current_action_time_remaining =
    static_cast<int32_t>(m.get_d_exact(p + "/current_action_time_remaining"));

  // yellow
  ref.yellow.name = m.get_s_exact(p + "/yellow/name");
  ref.yellow.score = static_cast<uint32_t>(m.get_d_exact(p + "/yellow/score"));
  ref.yellow.yellow_cards = static_cast<uint32_t>(m.get_d_exact(p + "/yellow/yellow_cards"));
  ref.yellow.red_cards = static_cast<uint32_t>(m.get_d_exact(p + "/yellow/red_cards"));
  ref.yellow.foul_counter = static_cast<uint32_t>(m.get_d_exact(p + "/yellow/foul_counter"));
  ref.yellow.goalkeeper = static_cast<uint32_t>(m.get_d_exact(p + "/yellow/goalkeeper"));

  // blue
  ref.blue.name = m.get_s_exact(p + "/blue/name");
  ref.blue.score = static_cast<uint32_t>(m.get_d_exact(p + "/blue/score"));
  ref.blue.yellow_cards = static_cast<uint32_t>(m.get_d_exact(p + "/blue/yellow_cards"));
  ref.blue.red_cards = static_cast<uint32_t>(m.get_d_exact(p + "/blue/red_cards"));
  ref.blue.foul_counter = static_cast<uint32_t>(m.get_d_exact(p + "/blue/foul_counter"));
  ref.blue.goalkeeper = static_cast<uint32_t>(m.get_d_exact(p + "/blue/goalkeeper"));

  // designated_position（has_field にかかわらず値を読む。表示側でフラグチェック）
  ref.designated_position_x = static_cast<float>(m.get_d_exact(p + "/designated_position/x"));
  ref.designated_position_y = static_cast<float>(m.get_d_exact(p + "/designated_position/y"));

  // next_command
  ref.next_command_value = static_cast<int32_t>(m.get_d_exact(p + "/next_command/value"));

  // game_events
  const std::string ge_prefix = p + "/game_events";
  size_t n_ge = m.count_array(ge_prefix);
  ref.game_events.resize(n_ge);

  for (size_t i = 0; i < n_ge; ++i) {
    auto & ge = ref.game_events[i];
    ge.type_value =
      static_cast<int32_t>(m.get_d_exact(FlatValueMap::arr_path(ge_prefix, i, "type/value")));

    // event サブフィールドを共通フィールドにマッピング
    // 全サブフィールドが存在するため type_value で選択的に読み出す
    const std::string ev = FlatValueMap::arr_path(ge_prefix, i, "event");

    // by_team（複数のイベント種別で共通）
    // いずれかのサブメッセージの by_team/value を探す（最初に見つかったものを使用）
    for (const auto & sub : {
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
         }) {
      auto it = m.numeric.find(sub);
      if (it != m.numeric.end() && it->second != 0.0) {
        ge.by_team_value = static_cast<int32_t>(it->second);
        break;
      }
    }

    // violator（bot_crash_unique 系）
    {
      auto it = m.numeric.find(ev + "/bot_crash_unique/violator");
      if (it == m.numeric.end()) it = m.numeric.find(ev + "/bot_crash_unique_skipped/violator");
      if (it != m.numeric.end()) ge.violator = static_cast<uint32_t>(it->second);
    }

    // victim（bot_crash_unique 系）
    {
      auto it = m.numeric.find(ev + "/bot_crash_unique/victim");
      if (it == m.numeric.end()) it = m.numeric.find(ev + "/bot_crash_unique_skipped/victim");
      if (it != m.numeric.end()) ge.victim = static_cast<uint32_t>(it->second);
    }

    // victim for bot_pushed_bot
    {
      auto it = m.numeric.find(ev + "/bot_pushed_bot/victim");
      if (it == m.numeric.end()) it = m.numeric.find(ev + "/bot_pushed_bot_skipped/victim");
      if (it != m.numeric.end() && ge.victim == 0) ge.victim = static_cast<uint32_t>(it->second);
    }

    // by_bot（複数種別）
    for (const auto & key : {
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
         }) {
      auto it = m.numeric.find(key);
      if (it != m.numeric.end() && it->second != 0.0) {
        ge.by_bot = static_cast<uint32_t>(it->second);
        break;
      }
    }

    // location（bot_crash_unique 優先、次いで各種イベント）
    for (const auto & [x_key, y_key] : std::vector<std::pair<std::string, std::string>>{
           {ev + "/bot_crash_unique/location/x", ev + "/bot_crash_unique/location/y"},
           {ev + "/bot_too_fast_in_stop/location/x", ev + "/bot_too_fast_in_stop/location/y"},
           {ev + "/bot_pushed_bot/location/x", ev + "/bot_pushed_bot/location/y"},
           {ev + "/bot_pushed_bot_skipped/location/x", ev + "/bot_pushed_bot_skipped/location/y"},
           {ev + "/keeper_held_ball/location/x", ev + "/keeper_held_ball/location/y"},
           {ev + "/defender_in_defense_area/location/x",
            ev + "/defender_in_defense_area/location/y"},
           {ev + "/attacker_too_close_to_defense_area/location/x",
            ev + "/attacker_too_close_to_defense_area/location/y"},
           {ev + "/attacker_touched_ball_in_defense_area/location/x",
            ev + "/attacker_touched_ball_in_defense_area/location/y"},
           {ev + "/bot_held_ball_deliberately/location/x",
            ev + "/bot_held_ball_deliberately/location/y"},
           {ev + "/attacker_double_touched_ball/location/x",
            ev + "/attacker_double_touched_ball/location/y"},
           {ev + "/aimless_kick/location/x", ev + "/aimless_kick/location/y"},
         }) {
      auto xi = m.numeric.find(x_key);
      auto yi = m.numeric.find(y_key);
      if (
        xi != m.numeric.end() &&
        (xi->second != 0.0 || (yi != m.numeric.end() && yi->second != 0.0))) {
        ge.location_x = static_cast<float>(xi->second);
        ge.location_y = (yi != m.numeric.end()) ? static_cast<float>(yi->second) : 0.0f;
        break;
      }
    }

    // crash_speed
    {
      auto it = m.numeric.find(ev + "/bot_crash_unique/crash_speed");
      if (it == m.numeric.end()) it = m.numeric.find(ev + "/bot_crash_unique_skipped/crash_speed");
      if (it != m.numeric.end()) ge.crash_speed = static_cast<float>(it->second);
    }

    // speed（bot_too_fast_in_stop）
    {
      auto it = m.numeric.find(ev + "/bot_too_fast_in_stop/speed");
      if (it != m.numeric.end()) ge.speed = static_cast<float>(it->second);
    }

    // duration（keeper_held_ball）
    {
      auto it = m.numeric.find(ev + "/keeper_held_ball/duration");
      if (it != m.numeric.end()) ge.duration = static_cast<float>(it->second);
    }
  }

  return ref;
}

}  // namespace crane::bag
