// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "bag_events.hpp"

#include <algorithm>
#include <cmath>
#include <robocup_ssl_msgs/msg/game_event_one_of_event.hpp>
#include <robocup_ssl_msgs/msg/game_event_type.hpp>
#include <robocup_ssl_msgs/msg/team.hpp>
#include <set>
#include <sstream>
#include <unordered_set>
#include <utility>

namespace crane::bag
{

std::vector<Event> detect_play_transitions(const BagData & data)
{
  std::vector<Event> events;
  std::string prev_cmd;
  for (const auto & tm : data.play_situations) {
    const std::string & cmd = tm.msg.command.name;
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
    // 立ち上がりエッジ
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
  using T = robocup_ssl_msgs::msg::GameEventType;
  switch (v) {
    case T::UNKNOWN_GAME_EVENT_TYPE:
      return "UNKNOWN_GAME_EVENT_TYPE";
    case T::BALL_LEFT_FIELD_TOUCH_LINE:
      return "BALL_LEFT_FIELD_TOUCH_LINE";
    case T::BALL_LEFT_FIELD_GOAL_LINE:
      return "BALL_LEFT_FIELD_GOAL_LINE";
    case T::AIMLESS_KICK:
      return "AIMLESS_KICK";
    case T::ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA:
      return "ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA";
    case T::DEFENDER_IN_DEFENSE_AREA:
      return "DEFENDER_IN_DEFENSE_AREA";
    case T::BOUNDARY_CROSSING:
      return "BOUNDARY_CROSSING";
    case T::KEEPER_HELD_BALL:
      return "KEEPER_HELD_BALL";
    case T::BOT_DRIBBLED_BALL_TOO_FAR:
      return "BOT_DRIBBLED_BALL_TOO_FAR";
    case T::BOT_PUSHED_BOT:
      return "BOT_PUSHED_BOT";
    case T::BOT_HELD_BALL_DELIBERATELY:
      return "BOT_HELD_BALL_DELIBERATELY";
    case T::BOT_TIPPED_OVER:
      return "BOT_TIPPED_OVER";
    case T::ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA:
      return "ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA";
    case T::BOT_KICKED_BALL_TOO_FAST:
      return "BOT_KICKED_BALL_TOO_FAST";
    case T::BOT_CRASH_UNIQUE:
      return "BOT_CRASH_UNIQUE";
    case T::BOT_CRASH_DRAWN:
      return "BOT_CRASH_DRAWN";
    case T::DEFENDER_TOO_CLOSE_TO_KICK_POINT:
      return "DEFENDER_TOO_CLOSE_TO_KICK_POINT";
    case T::BOT_TOO_FAST_IN_STOP:
      return "BOT_TOO_FAST_IN_STOP";
    case T::BOT_INTERFERED_PLACEMENT:
      return "BOT_INTERFERED_PLACEMENT";
    case T::POSSIBLE_GOAL:
      return "POSSIBLE_GOAL";
    case T::GOAL:
      return "GOAL";
    case T::INVALID_GOAL:
      return "INVALID_GOAL";
    case T::ATTACKER_DOUBLE_TOUCHED_BALL:
      return "ATTACKER_DOUBLE_TOUCHED_BALL";
    case T::PLACEMENT_SUCCEEDED:
      return "PLACEMENT_SUCCEEDED";
    case T::PENALTY_KICK_FAILED:
      return "PENALTY_KICK_FAILED";
    case T::NO_PROGRESS_IN_GAME:
      return "NO_PROGRESS_IN_GAME";
    case T::PLACEMENT_FAILED:
      return "PLACEMENT_FAILED";
    case T::MULTIPLE_CARDS:
      return "MULTIPLE_CARDS";
    case T::MULTIPLE_FOULS:
      return "MULTIPLE_FOULS";
    case T::BOT_SUBSTITUTION:
      return "BOT_SUBSTITUTION";
    case T::TOO_MANY_ROBOTS:
      return "TOO_MANY_ROBOTS";
    case T::CHALLENGE_FLAG:
      return "CHALLENGE_FLAG";
    case T::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
    case T::UNSPORTING_BEHAVIOR_MINOR:
      return "UNSPORTING_BEHAVIOR_MINOR";
    case T::UNSPORTING_BEHAVIOR_MAJOR:
      return "UNSPORTING_BEHAVIOR_MAJOR";
    case T::PREPARED:
      return "PREPARED";
    case T::INDIRECT_GOAL:
      return "INDIRECT_GOAL";
    case T::CHIPPED_GOAL:
      return "CHIPPED_GOAL";
    case T::KICK_TIMEOUT:
      return "KICK_TIMEOUT";
    case T::ATTACKER_TOUCHED_OPPONENT_IN_DEFENSE_AREA:
      return "ATTACKER_TOUCHED_OPPONENT_IN_DEFENSE_AREA";
    case T::ATTACKER_TOUCHED_OPPONENT_IN_DEFENSE_AREA_SKIPPED:
      return "ATTACKER_TOUCHED_OPPONENT_IN_DEFENSE_AREA_SKIPPED";
    case T::BOT_CRASH_UNIQUE_SKIPPED:
      return "BOT_CRASH_UNIQUE_SKIPPED";
    case T::BOT_PUSHED_BOT_SKIPPED:
      return "BOT_PUSHED_BOT_SKIPPED";
    case T::DEFENDER_IN_DEFENSE_AREA_PARTIALLY:
      return "DEFENDER_IN_DEFENSE_AREA_PARTIALLY";
    case T::MULTIPLE_PLACEMENT_FAILURES:
      return "MULTIPLE_PLACEMENT_FAILURES";
    default:
      return "GAME_EVENT(" + std::to_string(v) + ")";
  }
}

namespace
{

const std::unordered_set<int32_t> & foul_event_types()
{
  using T = robocup_ssl_msgs::msg::GameEventType;
  static const std::unordered_set<int32_t> s = {
    T::BOT_PUSHED_BOT,
    T::BOT_PUSHED_BOT_SKIPPED,
    T::BOT_HELD_BALL_DELIBERATELY,
    T::BOT_TIPPED_OVER,
    T::BOT_TOO_FAST_IN_STOP,
    T::DEFENDER_TOO_CLOSE_TO_KICK_POINT,
    T::DEFENDER_IN_DEFENSE_AREA,
    T::DEFENDER_IN_DEFENSE_AREA_PARTIALLY,
    T::ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA,
    T::ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA,
    T::ATTACKER_DOUBLE_TOUCHED_BALL,
    T::ATTACKER_TOUCHED_OPPONENT_IN_DEFENSE_AREA,
    T::ATTACKER_TOUCHED_OPPONENT_IN_DEFENSE_AREA_SKIPPED,
    T::BOT_CRASH_UNIQUE,
    T::BOT_CRASH_UNIQUE_SKIPPED,
    T::BOT_CRASH_DRAWN,
    T::BOT_KICKED_BALL_TOO_FAST,
    T::BOT_DRIBBLED_BALL_TOO_FAR,
    T::BOUNDARY_CROSSING,
    T::KEEPER_HELD_BALL,
    T::BOT_INTERFERED_PLACEMENT,
    T::UNSPORTING_BEHAVIOR_MINOR,
    T::UNSPORTING_BEHAVIOR_MAJOR,
    T::MULTIPLE_FOULS,
    T::AIMLESS_KICK,
  };
  return s;
}

std::string team_name(int32_t team_value)
{
  using Team = robocup_ssl_msgs::msg::Team;
  if (team_value == Team::YELLOW) return "YELLOW";
  if (team_value == Team::BLUE) return "BLUE";
  return "UNKNOWN";
}

std::string foul_description(const robocup_ssl_msgs::msg::GameEvent & ge)
{
  using T = robocup_ssl_msgs::msg::GameEventType;
  const auto & ev = ge.event;
  const std::string type_name = game_event_type_to_string(ge.type.value);
  char buf[256];

  switch (ge.type.value) {
    case T::BOT_CRASH_UNIQUE:
    case T::BOT_CRASH_UNIQUE_SKIPPED: {
      const auto & e = ev.bot_crash_unique;
      std::snprintf(
        buf, sizeof(buf), "%s: %s violator=%u victim=%u at (%.2f,%.2f) speed=%.2fm/s",
        type_name.c_str(), team_name(e.by_team.value).c_str(), e.violator, e.victim, e.location.x,
        e.location.y, e.crash_speed);
      return buf;
    }
    case T::BOT_TOO_FAST_IN_STOP: {
      const auto & e = ev.bot_too_fast_in_stop;
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u at (%.2f,%.2f) speed=%.2fm/s", type_name.c_str(),
        team_name(e.by_team.value).c_str(), e.by_bot, e.location.x, e.location.y, e.speed);
      return buf;
    }
    case T::BOT_PUSHED_BOT:
    case T::BOT_PUSHED_BOT_SKIPPED: {
      const auto & e = ev.bot_pushed_bot;
      std::snprintf(
        buf, sizeof(buf), "%s: %s violator=%u victim=%u at (%.2f,%.2f)", type_name.c_str(),
        team_name(e.by_team.value).c_str(), e.violator, e.victim, e.location.x, e.location.y);
      return buf;
    }
    case T::BOT_DRIBBLED_BALL_TOO_FAR: {
      const auto & e = ev.bot_dribbled_ball_too_far;
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u", type_name.c_str(), team_name(e.by_team.value).c_str(),
        e.by_bot);
      return buf;
    }
    case T::KEEPER_HELD_BALL: {
      const auto & e = ev.keeper_held_ball;
      std::snprintf(
        buf, sizeof(buf), "%s: %s at (%.2f,%.2f) duration=%.2fs", type_name.c_str(),
        team_name(e.by_team.value).c_str(), e.location.x, e.location.y, e.duration);
      return buf;
    }
    case T::DEFENDER_IN_DEFENSE_AREA:
    case T::DEFENDER_IN_DEFENSE_AREA_PARTIALLY: {
      const auto & e = ev.defender_in_defense_area;
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u at (%.2f,%.2f)", type_name.c_str(),
        team_name(e.by_team.value).c_str(), e.by_bot, e.location.x, e.location.y);
      return buf;
    }
    case T::ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA: {
      const auto & e = ev.attacker_too_close_to_defense_area;
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u at (%.2f,%.2f)", type_name.c_str(),
        team_name(e.by_team.value).c_str(), e.by_bot, e.location.x, e.location.y);
      return buf;
    }
    case T::ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA: {
      const auto & e = ev.attacker_touched_ball_in_defense_area;
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u at (%.2f,%.2f)", type_name.c_str(),
        team_name(e.by_team.value).c_str(), e.by_bot, e.location.x, e.location.y);
      return buf;
    }
    case T::BOT_HELD_BALL_DELIBERATELY: {
      const auto & e = ev.bot_held_ball_deliberately;
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u at (%.2f,%.2f)", type_name.c_str(),
        team_name(e.by_team.value).c_str(), e.by_bot, e.location.x, e.location.y);
      return buf;
    }
    case T::BOT_INTERFERED_PLACEMENT: {
      const auto & e = ev.bot_interfered_placement;
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u", type_name.c_str(), team_name(e.by_team.value).c_str(),
        e.by_bot);
      return buf;
    }
    case T::ATTACKER_DOUBLE_TOUCHED_BALL: {
      const auto & e = ev.attacker_double_touched_ball;
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u at (%.2f,%.2f)", type_name.c_str(),
        team_name(e.by_team.value).c_str(), e.by_bot, e.location.x, e.location.y);
      return buf;
    }
    case T::BOT_KICKED_BALL_TOO_FAST: {
      const auto & e = ev.bot_kicked_ball_too_fast;
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u", type_name.c_str(), team_name(e.by_team.value).c_str(),
        e.by_bot);
      return buf;
    }
    case T::AIMLESS_KICK: {
      const auto & e = ev.aimless_kick;
      std::snprintf(
        buf, sizeof(buf), "%s: %s bot=%u at (%.2f,%.2f)", type_name.c_str(),
        team_name(e.by_team.value).c_str(), e.by_bot, e.location.x, e.location.y);
      return buf;
    }
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
  // 重複排除: (type_value, command_counter) ペア
  std::set<std::pair<int32_t, uint32_t>> seen;

  for (const auto & tm : data.referees) {
    const auto & msg = tm.msg;
    for (const auto & ge : msg.game_events) {
      int32_t type_val = ge.type.value;
      if (foul_event_types().find(type_val) == foul_event_types().end()) continue;

      auto key = std::make_pair(type_val, msg.command_counter);
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
