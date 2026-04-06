// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "bag_events.hpp"

#include <algorithm>
#include <cmath>
#include <magic_enum/magic_enum.hpp>
#include <set>
#include <sstream>
#include <unordered_set>
#include <utility>

namespace crane::bag
{

// ─── GameEventType / Team（robocup_ssl_msgs と同じ値）────────────────────────

// clang-format off
enum class GameEventType : int32_t {
  UNKNOWN_GAME_EVENT_TYPE = 0,  PREPARED = 1,             NO_PROGRESS_IN_GAME = 2,
  PLACEMENT_FAILED = 3,         PLACEMENT_SUCCEEDED = 5,  BALL_LEFT_FIELD_TOUCH_LINE = 6,
  BALL_LEFT_FIELD_GOAL_LINE = 7, GOAL = 8,                INDIRECT_GOAL = 9,
  CHIPPED_GOAL = 10,            AIMLESS_KICK = 11,        KICK_TIMEOUT = 12,
  KEEPER_HELD_BALL = 13,        ATTACKER_DOUBLE_TOUCHED_BALL = 14,
  ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA = 15,
  ATTACKER_TOUCHED_OPPONENT_IN_DEFENSE_AREA = 16,
  BOT_DRIBBLED_BALL_TOO_FAR = 17, BOT_KICKED_BALL_TOO_FAST = 18,
  ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA = 19, BOT_INTERFERED_PLACEMENT = 20,
  BOT_CRASH_DRAWN = 21,         BOT_CRASH_UNIQUE = 22,    BOT_CRASH_UNIQUE_SKIPPED = 23,
  BOT_PUSHED_BOT = 24,          BOT_PUSHED_BOT_SKIPPED = 25,
  BOT_HELD_BALL_DELIBERATELY = 26, BOT_TIPPED_OVER = 27, BOT_TOO_FAST_IN_STOP = 28,
  DEFENDER_TOO_CLOSE_TO_KICK_POINT = 29,   DEFENDER_IN_DEFENSE_AREA_PARTIALLY = 30,
  DEFENDER_IN_DEFENSE_AREA = 31, MULTIPLE_CARDS = 32,     MULTIPLE_PLACEMENT_FAILURES = 33,
  MULTIPLE_FOULS = 34,          UNSPORTING_BEHAVIOR_MINOR = 35,
  UNSPORTING_BEHAVIOR_MAJOR = 36, BOT_SUBSTITUTION = 37, TOO_MANY_ROBOTS = 38,
  POSSIBLE_GOAL = 39,           ATTACKER_TOUCHED_OPPONENT_IN_DEFENSE_AREA_SKIPPED = 40,
  BOUNDARY_CROSSING = 41,       INVALID_GOAL = 42,        PENALTY_KICK_FAILED = 43,
  CHALLENGE_FLAG = 44,          EMERGENCY_STOP = 45,
};
// clang-format on

enum class Team : int32_t { UNKNOWN = 0, YELLOW = 1, BLUE = 2 };

// ─── 既存の実装（変更少）────────────────────────────────────────────────────────

std::vector<Event> detect_play_transitions(const BagData & data)
{
  std::vector<Event> events;
  std::string prev_cmd;
  for (const auto & tm : data.play_situations) {
    const std::string & cmd = tm.msg.command_name;
    if (cmd != prev_cmd) {
      Event e;
      e.timestamp_ns = tm.timestamp_ns;
      e.t = tm.t(data.info.start_time_ns);
      e.event_type = EVENT_PLAY;
      e.description = "PLAY: " + prev_cmd + " -> " + cmd + " (" + tm.msg.reason_text + ")";
      events.push_back(e);
      prev_cmd = cmd;
    }
  }
  return events;
}

std::vector<Event> detect_role_changes(const BagData & data)
{
  std::vector<Event> events;
  std::map<std::string, std::vector<uint8_t>> prev_roles;

  for (const auto & tm : data.robot_select_results) {
    std::map<std::string, std::vector<uint8_t>> current_roles;
    for (const auto & r : tm.msg.results) {
      current_roles[r.name] =
        std::vector<uint8_t>(r.selected_robots.begin(), r.selected_robots.end());
    }

    if (current_roles != prev_roles) {
      std::ostringstream desc;
      desc << "ROLE:";
      bool first = true;
      for (const auto & [name, robots] : current_roles) {
        auto it = prev_roles.find(name);
        if (it == prev_roles.end() || it->second != robots) {
          if (!first) desc << ",";
          first = false;
          desc << " " << name << ": [";
          auto prev = (it != prev_roles.end()) ? it->second : std::vector<uint8_t>{};
          for (size_t i = 0; i < prev.size(); ++i) {
            if (i > 0) desc << ",";
            desc << static_cast<int>(prev[i]);
          }
          desc << "] -> [";
          for (size_t i = 0; i < robots.size(); ++i) {
            if (i > 0) desc << ",";
            desc << static_cast<int>(robots[i]);
          }
          desc << "]";
        }
      }
      Event e;
      e.timestamp_ns = tm.timestamp_ns;
      e.t = tm.t(data.info.start_time_ns);
      e.event_type = EVENT_ROLE;
      e.description = desc.str();
      events.push_back(e);
      prev_roles = current_roles;
    }
  }
  return events;
}

std::vector<Event> detect_kick_events(const BagData & data)
{
  std::vector<Event> events;
  std::set<int> prev_kicking;

  for (const auto & tm : data.robot_commands) {
    std::set<int> kicking_now;
    for (const auto & rc : tm.msg.robot_commands) {
      if (rc.kick_power > 0.0f) {
        kicking_now.insert(static_cast<int>(rc.robot_id));
      }
    }
    for (int rid : kicking_now) {
      if (prev_kicking.find(rid) == prev_kicking.end()) {
        Event e;
        e.timestamp_ns = tm.timestamp_ns;
        e.t = tm.t(data.info.start_time_ns);
        e.event_type = EVENT_KICK;
        e.description = "KICK: robot=" + std::to_string(rid);
        events.push_back(e);
      }
    }
    prev_kicking = kicking_now;
  }
  return events;
}

std::vector<Event> detect_ball_speed_spikes(const BagData & data, double threshold)
{
  std::vector<Event> events;
  bool prev_above = false;

  for (const auto & tm : data.world_models) {
    const auto & ball = tm.msg.ball_info;
    double spd = std::sqrt(ball.velocity.x * ball.velocity.x + ball.velocity.y * ball.velocity.y);
    bool above = spd >= threshold;

    if (above && !prev_above) {
      char buf[128];
      std::snprintf(
        buf, sizeof(buf), "BALL_SPEED: %.2fm/s >= %.1fm/s at (%.2f,%.2f)", spd, threshold,
        ball.position.x, ball.position.y);
      Event e;
      e.timestamp_ns = tm.timestamp_ns;
      e.t = tm.t(data.info.start_time_ns);
      e.event_type = EVENT_BALL_SPEED;
      e.description = buf;
      events.push_back(e);
    }
    prev_above = above;
  }
  return events;
}

std::vector<Event> detect_goals(const BagData & data)
{
  std::vector<Event> events;
  if (data.world_models.empty()) return events;

  const auto & field_info = data.world_models.front().msg.field_info;
  double half_length = field_info.x / 2.0;
  constexpr double GOAL_HALF_WIDTH = 0.5;

  bool prev_in_goal = false;

  for (const auto & tm : data.world_models) {
    const auto & ball = tm.msg.ball_info;
    double bx = ball.position.x, by = ball.position.y;
    bool in_goal = (std::abs(bx) >= half_length && std::abs(by) <= GOAL_HALF_WIDTH);

    if (in_goal && !prev_in_goal) {
      const char * side = (bx > 0) ? "THEIR_GOAL" : "OUR_GOAL";
      char buf[128];
      std::snprintf(buf, sizeof(buf), "GOAL: %s ball=(%.2f,%.2f)", side, bx, by);
      Event e;
      e.timestamp_ns = tm.timestamp_ns;
      e.t = tm.t(data.info.start_time_ns);
      e.event_type = EVENT_GOAL;
      e.description = buf;
      events.push_back(e);
    }
    prev_in_goal = in_goal;
  }
  return events;
}

std::string game_event_type_to_string(int32_t v)
{
  auto e = magic_enum::enum_cast<GameEventType>(v);
  return e ? std::string(magic_enum::enum_name(*e)) : "GAME_EVENT(" + std::to_string(v) + ")";
}

namespace
{

const std::unordered_set<GameEventType> & foul_event_types()
{
  static const std::unordered_set<GameEventType> s = {
    GameEventType::BOT_PUSHED_BOT,
    GameEventType::BOT_PUSHED_BOT_SKIPPED,
    GameEventType::BOT_HELD_BALL_DELIBERATELY,
    GameEventType::BOT_TIPPED_OVER,
    GameEventType::BOT_TOO_FAST_IN_STOP,
    GameEventType::DEFENDER_TOO_CLOSE_TO_KICK_POINT,
    GameEventType::DEFENDER_IN_DEFENSE_AREA,
    GameEventType::DEFENDER_IN_DEFENSE_AREA_PARTIALLY,
    GameEventType::ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA,
    GameEventType::ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA,
    GameEventType::ATTACKER_DOUBLE_TOUCHED_BALL,
    GameEventType::ATTACKER_TOUCHED_OPPONENT_IN_DEFENSE_AREA,
    GameEventType::ATTACKER_TOUCHED_OPPONENT_IN_DEFENSE_AREA_SKIPPED,
    GameEventType::BOT_CRASH_UNIQUE,
    GameEventType::BOT_CRASH_UNIQUE_SKIPPED,
    GameEventType::BOT_CRASH_DRAWN,
    GameEventType::BOT_KICKED_BALL_TOO_FAST,
    GameEventType::BOT_DRIBBLED_BALL_TOO_FAR,
    GameEventType::BOUNDARY_CROSSING,
    GameEventType::KEEPER_HELD_BALL,
    GameEventType::BOT_INTERFERED_PLACEMENT,
    GameEventType::UNSPORTING_BEHAVIOR_MINOR,
    GameEventType::UNSPORTING_BEHAVIOR_MAJOR,
    GameEventType::MULTIPLE_FOULS,
    GameEventType::AIMLESS_KICK,
  };
  return s;
}

std::string team_name(int32_t team_value)
{
  auto t = magic_enum::enum_cast<Team>(team_value);
  return t ? std::string(magic_enum::enum_name(*t)) : "UNKNOWN";
}

std::string foul_description(const GameEventInfo & ge)
{
  const std::string type_name = game_event_type_to_string(ge.type_value);
  char buf[256];

  switch (static_cast<GameEventType>(ge.type_value)) {
    case GameEventType::BOT_CRASH_UNIQUE:
    case GameEventType::BOT_CRASH_UNIQUE_SKIPPED:
      std::snprintf(
        buf, sizeof(buf), "%s: %s violator=%u victim=%u at (%.2f,%.2f) speed=%.2fm/s",
        type_name.c_str(), team_name(ge.by_team_value).c_str(), ge.violator, ge.victim,
        ge.location_x, ge.location_y, ge.crash_speed);
      return buf;
    case GameEventType::BOT_TOO_FAST_IN_STOP:
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u at (%.2f,%.2f) speed=%.2fm/s", type_name.c_str(),
        team_name(ge.by_team_value).c_str(), ge.by_bot, ge.location_x, ge.location_y, ge.speed);
      return buf;
    case GameEventType::BOT_PUSHED_BOT:
    case GameEventType::BOT_PUSHED_BOT_SKIPPED:
      std::snprintf(
        buf, sizeof(buf), "%s: %s violator=%u victim=%u at (%.2f,%.2f)", type_name.c_str(),
        team_name(ge.by_team_value).c_str(), ge.violator, ge.victim, ge.location_x, ge.location_y);
      return buf;
    case GameEventType::BOT_DRIBBLED_BALL_TOO_FAR:
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u", type_name.c_str(), team_name(ge.by_team_value).c_str(),
        ge.by_bot);
      return buf;
    case GameEventType::KEEPER_HELD_BALL:
      std::snprintf(
        buf, sizeof(buf), "%s: %s at (%.2f,%.2f) duration=%.2fs", type_name.c_str(),
        team_name(ge.by_team_value).c_str(), ge.location_x, ge.location_y, ge.duration);
      return buf;
    case GameEventType::DEFENDER_IN_DEFENSE_AREA:
    case GameEventType::DEFENDER_IN_DEFENSE_AREA_PARTIALLY:
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u at (%.2f,%.2f)", type_name.c_str(),
        team_name(ge.by_team_value).c_str(), ge.by_bot, ge.location_x, ge.location_y);
      return buf;
    case GameEventType::ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA:
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u at (%.2f,%.2f)", type_name.c_str(),
        team_name(ge.by_team_value).c_str(), ge.by_bot, ge.location_x, ge.location_y);
      return buf;
    case GameEventType::ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA:
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u at (%.2f,%.2f)", type_name.c_str(),
        team_name(ge.by_team_value).c_str(), ge.by_bot, ge.location_x, ge.location_y);
      return buf;
    case GameEventType::BOT_HELD_BALL_DELIBERATELY:
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u at (%.2f,%.2f)", type_name.c_str(),
        team_name(ge.by_team_value).c_str(), ge.by_bot, ge.location_x, ge.location_y);
      return buf;
    case GameEventType::BOT_INTERFERED_PLACEMENT:
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u", type_name.c_str(), team_name(ge.by_team_value).c_str(),
        ge.by_bot);
      return buf;
    case GameEventType::ATTACKER_DOUBLE_TOUCHED_BALL:
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u at (%.2f,%.2f)", type_name.c_str(),
        team_name(ge.by_team_value).c_str(), ge.by_bot, ge.location_x, ge.location_y);
      return buf;
    case GameEventType::BOT_KICKED_BALL_TOO_FAST:
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u", type_name.c_str(), team_name(ge.by_team_value).c_str(),
        ge.by_bot);
      return buf;
    case GameEventType::AIMLESS_KICK:
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u at (%.2f,%.2f)", type_name.c_str(),
        team_name(ge.by_team_value).c_str(), ge.by_bot, ge.location_x, ge.location_y);
      return buf;
    default:
      return type_name;
  }
}

}  // namespace

std::vector<Event> detect_fouls(const BagData & data)
{
  std::vector<Event> events;
  if (data.referees.empty()) return events;

  int64_t bag_start = data.info.start_time_ns;
  std::set<std::pair<int32_t, uint32_t>> seen;

  for (const auto & tm : data.referees) {
    const auto & msg = tm.msg;
    for (const auto & ge : msg.game_events) {
      if (!foul_event_types().count(static_cast<GameEventType>(ge.type_value))) continue;

      auto key = std::make_pair(ge.type_value, msg.command_counter);
      if (seen.count(key)) continue;
      seen.insert(key);

      Event e;
      e.timestamp_ns = tm.timestamp_ns;
      e.t = tm.t(bag_start);
      e.event_type = EVENT_FOUL;
      e.description = foul_description(ge);
      events.push_back(e);
    }
  }

  std::sort(events.begin(), events.end(), [](const Event & a, const Event & b) {
    return a.timestamp_ns < b.timestamp_ns;
  });
  return events;
}

std::vector<Event> detect_events(const BagData & data, const std::vector<std::string> & types)
{
  const std::vector<std::string> & target = types.empty() ? ALL_EVENT_TYPES : types;
  auto has = [&](const char * t) {
    return std::find(target.begin(), target.end(), t) != target.end();
  };

  std::vector<Event> all;
  if (has(EVENT_PLAY)) {
    auto v = detect_play_transitions(data);
    all.insert(all.end(), v.begin(), v.end());
  }
  if (has(EVENT_ROLE)) {
    auto v = detect_role_changes(data);
    all.insert(all.end(), v.begin(), v.end());
  }
  if (has(EVENT_KICK)) {
    auto v = detect_kick_events(data);
    all.insert(all.end(), v.begin(), v.end());
  }
  if (has(EVENT_BALL_SPEED)) {
    auto v = detect_ball_speed_spikes(data);
    all.insert(all.end(), v.begin(), v.end());
  }
  if (has(EVENT_GOAL)) {
    auto v = detect_goals(data);
    all.insert(all.end(), v.begin(), v.end());
  }
  if (has(EVENT_FOUL)) {
    auto v = detect_fouls(data);
    all.insert(all.end(), v.begin(), v.end());
  }

  std::sort(all.begin(), all.end(), [](const Event & a, const Event & b) {
    return a.timestamp_ns < b.timestamp_ns;
  });
  return all;
}

}  // namespace crane::bag
