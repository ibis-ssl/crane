// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__COMMAND_WRAPPER_BASE_HPP_
#define CRANE_MSG_WRAPPERS__COMMAND_WRAPPER_BASE_HPP_

#include <algorithm>
#include <crane_msgs/msg/robot_command.hpp>
#include <cstdio>
#include <string>

namespace crane
{

/**
 * @brief planning_factors の文字列フォーマット（snprintf ベース）
 */
inline auto formatPlanningDouble(double value, int precision = 3) -> std::string
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.*f", precision, value);
  return std::string(buf);
}

/**
 * @brief raw RobotCommand の planning_factors を追加または更新するフリー関数
 */
inline auto addOrUpdatePlanningFactor(
  crane_msgs::msg::RobotCommand & command, const std::string & name, const std::string & value)
  -> void
{
  auto it = std::find_if(
    command.planning_factors.begin(), command.planning_factors.end(),
    [&name](const auto & factor) { return factor.name == name; });
  if (it == command.planning_factors.end()) {
    crane_msgs::msg::NamedString factor;
    factor.name = name;
    factor.value = value;
    command.planning_factors.emplace_back(factor);
  } else if (it->value != value) {
    it->value = value;
  }
}

}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__COMMAND_WRAPPER_BASE_HPP_
