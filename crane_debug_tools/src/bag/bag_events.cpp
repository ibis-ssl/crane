// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "bag_events.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>

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

  std::sort(all.begin(), all.end(), [](const Event & a, const Event & b) {
    return a.timestamp_ns < b.timestamp_ns;
  });
  return all;
}

}  // namespace crane::bag
