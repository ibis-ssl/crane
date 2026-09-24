// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <map>
#include <string>
#include <unordered_set>
#include <vector>

#include "bag_reader.hpp"

namespace crane::bag
{

struct Event
{
  int64_t timestamp_ns;
  double t;
  std::string event_type;
  std::string description;
};

/// イベントタイプ定数
constexpr const char * EVENT_GOAL = "goal";
constexpr const char * EVENT_PLAY = "play";
constexpr const char * EVENT_ROLE = "role";
constexpr const char * EVENT_KICK = "kick";
constexpr const char * EVENT_BALL_SPEED = "ball_speed";
constexpr const char * EVENT_FOUL = "foul";
constexpr const char * EVENT_PASS = "pass";

inline const std::vector<std::string> ALL_EVENT_TYPES = {
  EVENT_GOAL, EVENT_PLAY, EVENT_ROLE, EVENT_KICK, EVENT_BALL_SPEED, EVENT_FOUL, EVENT_PASS};

/// pass イベント検出には /world_model のロボット配列が必要（ball-only モード不可）
bool event_types_require_full_world_model(const std::vector<std::string> & types);

std::vector<Event> detect_events(const BagData & data, const std::vector<std::string> & types = {});

/// 要求された event type の検出に必要なトピック集合を返す（空 types = 全タイプ）。
/// read() に渡して不要トピックのデシリアライズを省くために使う。
std::unordered_set<std::string> topics_for_event_types(const std::vector<std::string> & types);

std::vector<Event> detect_play_transitions(const BagData & data);
std::vector<Event> detect_role_changes(const BagData & data);
std::vector<Event> detect_kick_events(const BagData & data);
std::vector<Event> detect_ball_speed_spikes(const BagData & data, double threshold = 3.0);
std::vector<Event> detect_goals(const BagData & data);
std::vector<Event> detect_fouls(const BagData & data);
std::vector<Event> detect_pass_attempt_events(const BagData & data);

std::string game_event_type_to_string(int32_t type_value);

}  // namespace crane::bag
