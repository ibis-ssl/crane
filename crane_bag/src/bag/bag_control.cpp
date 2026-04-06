// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "bag_control.hpp"

namespace crane::bag
{

namespace
{

ControlSnapshot extract_snapshot(double t, const RobotCommand & rc)
{
  ControlSnapshot s;
  s.t = t;
  s.robot_id = static_cast<int>(rc.robot_id);
  for (const auto & pf : rc.planning_factors) {
    s.planning_factors.emplace_back(pf.name, pf.value);
  }
  s.kick_power = rc.kick_power;
  s.dribble_power = rc.dribble_power;
  s.stop_flag = rc.stop_flag;
  s.planner_name = rc.planner_name;

  if (!rc.position_target_mode.empty()) {
    s.target_x = rc.position_target_mode[0].target_x;
    s.target_y = rc.position_target_mode[0].target_y;
  }
  if (!rc.polar_velocity_target_mode.empty()) {
    s.velocity_r = rc.polar_velocity_target_mode[0].target_velocity_r;
    s.velocity_theta = rc.polar_velocity_target_mode[0].target_velocity_theta;
  }
  return s;
}

}  // namespace

std::vector<ControlSnapshot> analyze_control(
  const BagData & data, int robot_id, double interval,
  std::optional<std::pair<double, double>> time_range)
{
  std::vector<ControlSnapshot> result;
  int64_t bag_start = data.info.start_time_ns;

  auto [filter_start, filter_end] = make_ns_range(bag_start, time_range);

  int64_t interval_ns = static_cast<int64_t>(interval * 1e9);
  int64_t last_ns = 0;

  for (const auto & tm : data.control_targets) {
    if (tm.timestamp_ns < filter_start || tm.timestamp_ns > filter_end) continue;
    if (tm.timestamp_ns - last_ns < interval_ns) continue;

    for (const auto & rc : tm.msg.robot_commands) {
      if (static_cast<int>(rc.robot_id) == robot_id) {
        result.push_back(extract_snapshot(tm.t(bag_start), rc));
        last_ns = tm.timestamp_ns;
        break;
      }
    }
  }
  return result;
}

std::vector<FactorTransition> detect_factor_transitions(
  const BagData & data, std::optional<int> robot_id)
{
  std::vector<FactorTransition> result;
  int64_t bag_start = data.info.start_time_ns;
  std::map<int, std::vector<std::pair<std::string, std::string>>> prev_factors;

  for (const auto & tm : data.control_targets) {
    for (const auto & rc : tm.msg.robot_commands) {
      int rid = static_cast<int>(rc.robot_id);
      if (robot_id.has_value() && rid != robot_id.value()) continue;

      std::vector<std::pair<std::string, std::string>> current;
      for (const auto & pf : rc.planning_factors) {
        current.emplace_back(pf.name, pf.value);
      }

      auto it = prev_factors.find(rid);
      if (it != prev_factors.end() && it->second != current) {
        FactorTransition tr;
        tr.t = tm.t(bag_start);
        tr.robot_id = rid;
        tr.old_factors = it->second;
        tr.new_factors = current;
        result.push_back(tr);
      }
      prev_factors[rid] = current;
    }
  }
  return result;
}

}  // namespace crane::bag
