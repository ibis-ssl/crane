// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "bag_referee.hpp"

#include <crane_msg_wrappers/play_situation_wrapper.hpp>
#include <robocup_ssl_msgs/msg/referee.hpp>

namespace crane::bag
{

namespace
{

using RefereeMsg = robocup_ssl_msgs::msg::Referee;

RefereeSnapshot make_snapshot(const TimestampedMsg<RefereeMsg> & tm, int64_t bag_start_ns)
{
  const auto & msg = tm.msg;
  RefereeSnapshot snap;
  snap.t = tm.t(bag_start_ns);
  snap.timestamp_ns = tm.timestamp_ns;
  snap.command_value = msg.command.value;
  snap.command_name = command_to_string(msg.command.value);
  snap.stage_value = msg.stage.value;
  snap.stage_name = stage_to_string(msg.stage.value);
  snap.command_counter = msg.command_counter;

  snap.yellow.name = msg.yellow.name;
  snap.yellow.score = msg.yellow.score;
  snap.yellow.yellow_cards = msg.yellow.yellow_cards;
  snap.yellow.red_cards = msg.yellow.red_cards;
  snap.yellow.foul_counter = msg.yellow.foul_counter;
  snap.yellow.goalkeeper = msg.yellow.goalkeeper;

  snap.blue.name = msg.blue.name;
  snap.blue.score = msg.blue.score;
  snap.blue.yellow_cards = msg.blue.yellow_cards;
  snap.blue.red_cards = msg.blue.red_cards;
  snap.blue.foul_counter = msg.blue.foul_counter;
  snap.blue.goalkeeper = msg.blue.goalkeeper;

  snap.has_designated_position = (msg.has_field & RefereeMsg::DESIGNATED_POSITION_FIELD_SET) != 0;
  snap.designated_x = snap.has_designated_position ? msg.designated_position.x / 1000.0f : 0.0f;
  snap.designated_y = snap.has_designated_position ? msg.designated_position.y / 1000.0f : 0.0f;

  snap.has_next_command = (msg.has_field & RefereeMsg::NEXT_COMMAND_FIELD_SET) != 0;
  snap.next_command_value = snap.has_next_command ? msg.next_command.value : 0;
  snap.next_command_name = snap.has_next_command ? command_to_string(msg.next_command.value) : "";

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
  return crane::getRefereeCommandText(static_cast<uint32_t>(cmd));
}

std::string stage_to_string(int32_t stage)
{
  return crane::getStageText(static_cast<uint32_t>(stage));
}

}  // namespace crane::bag
