// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "bag_reader.hpp"

namespace crane::bag
{

inline std::string format_factor_pairs(const std::vector<std::pair<std::string, std::string>> & fs)
{
  if (fs.empty()) return "(empty)";
  std::string s;
  for (size_t i = 0; i < fs.size(); ++i) {
    if (i > 0) s += "; ";
    s += fs[i].first + "=" + fs[i].second;
  }
  return s;
}

struct ControlSnapshot
{
  double t;
  int robot_id;
  std::vector<std::pair<std::string, std::string>> planning_factors;
  float kick_power;
  float dribble_power;
  std::optional<float> target_x, target_y;
  std::optional<float> velocity_r, velocity_theta;
  bool stop_flag;
  std::string planner_name;
};

struct FactorTransition
{
  double t;
  int robot_id;
  std::vector<std::pair<std::string, std::string>> old_factors;
  std::vector<std::pair<std::string, std::string>> new_factors;
};

std::vector<ControlSnapshot> analyze_control(
  const BagData & data, int robot_id, double interval = 0.1,
  std::optional<std::pair<double, double>> time_range = std::nullopt);

std::vector<FactorTransition> detect_factor_transitions(
  const BagData & data, std::optional<int> robot_id = std::nullopt);

}  // namespace crane::bag
