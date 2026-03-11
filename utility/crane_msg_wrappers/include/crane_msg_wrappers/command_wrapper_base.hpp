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

/**
 * @brief RobotCommandWrapper と VelocityCommandWrapper の共通メソッドを提供する CRTP 基底クラス
 * @tparam Derived 派生クラス
 *
 * 派生クラスは以下のメソッドを提供する必要がある（friend宣言推奨）:
 *   crane_msgs::msg::RobotCommand & getLatestMsg()
 *   const crane_msgs::msg::RobotCommand & getLatestMsg() const
 */
template <typename Derived>
class CommandWrapperBase
{
  auto & msg() { return static_cast<Derived &>(*this).getLatestMsg(); }
  const auto & msg() const { return static_cast<const Derived &>(*this).getLatestMsg(); }

public:
  auto getMsg() const -> const crane_msgs::msg::RobotCommand & { return msg(); }

  auto getEditableMsg() -> crane_msgs::msg::RobotCommand & { return msg(); }

  auto dribble(double power) -> Derived &
  {
    msg().dribble_power = power;
    msg().kick_power = 0.0;
    return static_cast<Derived &>(*this);
  }

  auto withDribble(double power) -> Derived &
  {
    msg().dribble_power = power;
    return static_cast<Derived &>(*this);
  }

  auto stopEmergency(bool flag = true) -> Derived &
  {
    msg().stop_flag = flag;
    return static_cast<Derived &>(*this);
  }

  auto addPlanningFactor(const std::string & name, const std::string & state) -> void
  {
    addOrUpdatePlanningFactor(msg(), name, state);
  }

  auto clearPlanningFactors() -> void { msg().planning_factors.clear(); }
};

}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__COMMAND_WRAPPER_BASE_HPP_
