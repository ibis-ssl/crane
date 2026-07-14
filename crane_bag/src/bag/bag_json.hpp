// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <nlohmann/json.hpp>

#include "bag_control.hpp"
#include "bag_events.hpp"
#include "bag_kick_stats.hpp"
#include "bag_pass.hpp"
#include "bag_reader.hpp"
#include "bag_referee.hpp"
#include "bag_tracking.hpp"

namespace crane::bag
{

namespace detail
{
inline nlohmann::json factors_to_json(
  const std::vector<std::pair<std::string, std::string>> & factors)
{
  nlohmann::json arr = nlohmann::json::array();
  for (const auto & [k, val] : factors) {
    arr.push_back({{"name", k}, {"value", val}});
  }
  return arr;
}
}  // namespace detail

// BagInfo
inline void to_json(nlohmann::json & j, const BagInfo & v)
{
  nlohmann::json topics = nlohmann::json::array();
  for (const auto & [name, count] : v.topic_counts) {
    auto it = v.topic_types.find(name);
    std::string type = (it != v.topic_types.end()) ? it->second : "unknown";
    topics.push_back({{"name", name}, {"count", count}, {"type", type}});
  }
  j = {
    {"path", v.path},
    {"duration", v.duration_sec},
    {"start_time_ns", v.start_time_ns},
    {"end_time_ns", v.end_time_ns},
    {"topics", topics},
  };
}

// Event
inline void to_json(nlohmann::json & j, const Event & v)
{
  j = {
    {"t", v.t},
    {"timestamp_ns", v.timestamp_ns},
    {"type", v.event_type},
    {"description", v.description},
  };
}

// RobotState / BallState
// clang-format off
// clang-format off
// NOLINTNEXTLINE(whitespace/line_length)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RobotState, t, robot_id, x, y, theta, vx, vy, speed, dist_to_ball, detected)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(BallState, t, x, y, vx, vy, speed)
// clang-format on
// clang-format on

// ControlSnapshot
inline void to_json(nlohmann::json & j, const ControlSnapshot & v)
{
  j = {
    {"t", v.t},
    {"robot_id", v.robot_id},
    {"kick_power", v.kick_power},
    {"dribble_power", v.dribble_power},
    {"stop_flag", v.stop_flag},
    {"planner_name", v.planner_name},
    {"planning_factors", detail::factors_to_json(v.planning_factors)},
  };
  if (v.target_x.has_value()) {
    j["target_x"] = *v.target_x;
    j["target_y"] = *v.target_y;
  } else {
    j["target_x"] = nullptr;
    j["target_y"] = nullptr;
  }
  if (v.velocity_r.has_value()) {
    j["velocity_r"] = *v.velocity_r;
    j["velocity_theta"] = *v.velocity_theta;
  } else {
    j["velocity_r"] = nullptr;
    j["velocity_theta"] = nullptr;
  }
}

// FactorTransition
inline void to_json(nlohmann::json & j, const FactorTransition & v)
{
  j = {
    {"t", v.t},
    {"robot_id", v.robot_id},
    {"old_factors", detail::factors_to_json(v.old_factors)},
    {"new_factors", detail::factors_to_json(v.new_factors)},
  };
}

// TeamInfo (= RefereeSnapshot::TeamSnapshot)
// clang-format off
// NOLINTNEXTLINE(whitespace/line_length)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TeamInfo, name, score, yellow_cards, red_cards, foul_counter, goalkeeper)
// clang-format on

// RefereeSnapshot
inline void to_json(nlohmann::json & j, const RefereeSnapshot & v)
{
  j = {
    {"t", v.t},
    {"timestamp_ns", v.timestamp_ns},
    {"command", v.command_name},
    {"command_value", v.command_value},
    {"stage", v.stage_name},
    {"stage_value", v.stage_value},
    {"command_counter", v.command_counter},
    {"yellow", v.yellow},
    {"blue", v.blue},
    {"action_time_remaining_us", v.current_action_time_remaining_us},
  };
  if (v.has_designated_position) {
    j["designated_position"] = {{"x", v.designated_x}, {"y", v.designated_y}};
  } else {
    j["designated_position"] = nullptr;
  }
  if (v.has_next_command) {
    j["next_command"] = v.next_command_name;
    j["next_command_value"] = v.next_command_value;
  } else {
    j["next_command"] = nullptr;
    j["next_command_value"] = nullptr;
  }
}

// KickPowerBin / KickStatsGroup / KickStats
inline void to_json(nlohmann::json & j, const KickPowerBin & v)
{
  j = {
    {"power_lo", v.power_lo},
    {"power_hi", v.power_hi},
    {"count", v.count},
    {"mean_predicted_speed", v.mean_predicted_speed},
    {"mean_actual_speed", v.mean_actual_speed},
    {"mean_predicted_distance", v.mean_predicted_distance},
    {"mean_actual_distance", v.mean_actual_distance},
  };
}

inline void to_json(nlohmann::json & j, const KickStatsGroup & v)
{
  j = {
    {"count", v.count},
    {"speed_bias_percent", v.speed_bias_percent},
    {"speed_abs_mean", v.speed_abs_mean},
    {"speed_abs_p50", v.speed_abs_p50},
    {"speed_abs_p90", v.speed_abs_p90},
    {"dist_bias_percent", v.dist_bias_percent},
    {"dist_abs_mean", v.dist_abs_mean},
    {"dist_abs_p50", v.dist_abs_p50},
    {"dist_abs_p90", v.dist_abs_p90},
    {"power_bins", v.power_bins},
  };
}

inline void to_json(nlohmann::json & j, const KickStats & v)
{
  j = {
    {"total", v.total},
    {"with_actual", v.with_actual},
    {"straight", v.straight},
    {"chip", v.chip},
  };
}

// PassEvent
inline void to_json(nlohmann::json & j, const PassEvent & v)
{
  j = {
    {"t", v.t},
    {"timestamp_ns", v.timestamp_ns},
    {"kicker_id", v.kicker_id},
    {"intended_receiver_id", v.intended_receiver_id},
    {"reserved_receiver_id", v.reserved_receiver_id},
    {"outcome", to_string(v.outcome)},
    {"first_toucher_id", v.first_toucher_id},
    {"first_toucher_ours", v.first_toucher_ours},
    {"kick_speed", v.kick_speed},
    {"pass_distance", v.pass_distance},
    {"forward_progress", v.forward_progress},
    {"duration", v.duration},
    {"kick_pos", {{"x", v.kick_pos.x}, {"y", v.kick_pos.y}}},
    {"end_pos", {{"x", v.end_pos.x}, {"y", v.end_pos.y}}},
  };
}

// PassSummary
inline void to_json(nlohmann::json & j, const PassSummary & v)
{
  nlohmann::json bands = nlohmann::json::array();
  for (size_t b = 0; b < v.band_attempts.size(); ++b) {
    bands.push_back(
      {{"band", kPassDistanceBandLabels[b]},
       {"attempts", v.band_attempts[b]},
       {"success", v.band_success[b]}});
  }
  j = {
    {"attempts", v.attempts},
    {"success", v.success},
    {"wrong_receiver", v.wrong_receiver},
    {"intercepted", v.intercepted},
    {"overrun", v.overrun},
    {"out_of_play", v.out_of_play},
    {"unresolved", v.unresolved},
    {"avg_distance", v.avg_distance},
    {"avg_forward_progress", v.avg_forward_progress},
    {"avg_duration", v.avg_duration},
    {"avg_kick_speed", v.avg_kick_speed},
    {"by_distance_band", bands},
  };
}

}  // namespace crane::bag
