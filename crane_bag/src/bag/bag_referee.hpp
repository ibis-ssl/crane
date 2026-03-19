// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "bag_reader.hpp"

namespace crane::bag
{

struct RefereeSnapshot
{
  double t;
  int64_t timestamp_ns;
  std::string command_name;
  std::string stage_name;
  int32_t command_value;
  int32_t stage_value;
  uint32_t command_counter;

  struct TeamSnapshot
  {
    std::string name;
    uint32_t score;
    uint32_t yellow_cards;
    uint32_t red_cards;
    uint32_t foul_counter;
    uint32_t goalkeeper;
  };
  TeamSnapshot yellow;
  TeamSnapshot blue;

  bool has_designated_position;
  float designated_x;  // meters
  float designated_y;  // meters

  bool has_next_command;
  std::string next_command_name;
  int32_t next_command_value;

  int32_t current_action_time_remaining_us;
};

/// command_counter 変化時のみ（コマンド遷移）を抽出する
std::vector<RefereeSnapshot> extract_referee_transitions(const BagData & data);

/// Referee メッセージを指定間隔でサンプリングして返す
std::vector<RefereeSnapshot> sample_referee(const BagData & data, double interval_sec = 1.0);

std::string command_to_string(int32_t cmd);
std::string stage_to_string(int32_t stage);

}  // namespace crane::bag
