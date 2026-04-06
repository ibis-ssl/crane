// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "bag_referee.hpp"

#include <map>

namespace crane::bag
{

namespace
{

// ─── Referee コマンド文字列ルックアップ ────────────────────────────────────────
// robocup_ssl_msgs::msg::RefereeCommand / RefereeStage の整数定数と同じ値

static const std::map<int, std::string> referee_command_map = {
  {0, "HALT"},
  {1, "STOP"},
  {2, "NORMAL_START"},
  {3, "FORCE_START"},
  {4, "PREPARE_KICKOFF_YELLOW"},
  {5, "PREPARE_KICKOFF_BLUE"},
  {6, "PREPARE_PENALTY_YELLOW"},
  {7, "PREPARE_PENALTY_BLUE"},
  {8, "DIRECT_FREE_YELLOW"},
  {9, "DIRECT_FREE_BLUE"},
  {10, "INDIRECT_FREE_YELLOW"},
  {11, "INDIRECT_FREE_BLUE"},
  {12, "TIMEOUT_YELLOW"},
  {13, "TIMEOUT_BLUE"},
  {14, "GOAL_YELLOW"},
  {15, "GOAL_BLUE"},
  {16, "BALL_PLACEMENT_YELLOW"},
  {17, "BALL_PLACEMENT_BLUE"},
};

static const std::map<int, std::string> stage_map = {
  {0, "NORMAL_FIRST_HALF_PRE"},  {1, "NORMAL_FIRST_HALF"},  {2, "NORMAL_HALF_TIME"},
  {3, "NORMAL_SECOND_HALF_PRE"}, {4, "NORMAL_SECOND_HALF"}, {5, "EXTRA_TIME_BREAK"},
  {6, "EXTRA_FIRST_HALF_PRE"},   {7, "EXTRA_FIRST_HALF"},   {8, "EXTRA_HALF_TIME"},
  {9, "EXTRA_SECOND_HALF_PRE"},  {10, "EXTRA_SECOND_HALF"}, {11, "PENALTY_SHOOTOUT_BREAK"},
  {12, "PENALTY_SHOOTOUT"},      {13, "POST_GAME"},
};

RefereeSnapshot make_snapshot(const TimestampedMsg<Referee> & tm, int64_t bag_start_ns)
{
  const auto & msg = tm.msg;
  RefereeSnapshot snap;
  snap.t = tm.t(bag_start_ns);
  snap.timestamp_ns = tm.timestamp_ns;
  snap.command_value = msg.command_value;
  snap.command_name = command_to_string(msg.command_value);
  snap.stage_value = msg.stage_value;
  snap.stage_name = stage_to_string(msg.stage_value);
  snap.command_counter = msg.command_counter;

  snap.yellow = msg.yellow;
  snap.blue = msg.blue;

  snap.has_designated_position = (msg.has_field & REFEREE_DESIGNATED_POSITION_FIELD_SET) != 0;
  snap.designated_x = snap.has_designated_position ? msg.designated_position_x / 1000.0f : 0.0f;
  snap.designated_y = snap.has_designated_position ? msg.designated_position_y / 1000.0f : 0.0f;

  snap.has_next_command = (msg.has_field & REFEREE_NEXT_COMMAND_FIELD_SET) != 0;
  snap.next_command_value = snap.has_next_command ? msg.next_command_value : 0;
  snap.next_command_name = snap.has_next_command ? command_to_string(msg.next_command_value) : "";

  snap.current_action_time_remaining_us = msg.current_action_time_remaining;
  return snap;
}

}  // namespace

std::vector<RefereeSnapshot> extract_referee_transitions(const BagData & data)
{
  std::vector<RefereeSnapshot> result;
  if (data.referees.empty()) return result;

  int64_t bag_start = data.info.start_time_ns;
  uint32_t prev_counter = UINT32_MAX;

  for (const auto & tm : data.referees) {
    if (tm.msg.command_counter != prev_counter) {
      result.push_back(make_snapshot(tm, bag_start));
      prev_counter = tm.msg.command_counter;
    }
  }
  return result;
}

std::vector<RefereeSnapshot> sample_referee(const BagData & data, double interval_sec)
{
  std::vector<RefereeSnapshot> result;
  if (data.referees.empty()) return result;

  int64_t bag_start = data.info.start_time_ns;
  auto sampled = BagData::sample(data.referees, interval_sec);
  result.reserve(sampled.size());
  for (const auto * tm : sampled) {
    result.push_back(make_snapshot(*tm, bag_start));
  }
  return result;
}

std::string command_to_string(int32_t cmd)
{
  auto it = referee_command_map.find(cmd);
  return it != referee_command_map.end() ? it->second : "UNKNOWN";
}

std::string stage_to_string(int32_t stage)
{
  auto it = stage_map.find(stage);
  return it != stage_map.end() ? it->second : "UNKNOWN";
}

}  // namespace crane::bag
