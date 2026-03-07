// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__COMMAND_WRAPPER_BASE_HPP_
#define CRANE_MSG_WRAPPERS__COMMAND_WRAPPER_BASE_HPP_

#include <algorithm>
#include <crane_msgs/msg/robot_command.hpp>
#include <string>

namespace crane
{

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
    auto it = std::find_if(
      msg().planning_factors.begin(), msg().planning_factors.end(),
      [&name](const auto & pf) { return pf.name == name; });
    if (it == msg().planning_factors.end()) {
      crane_msgs::msg::NamedString ns;
      ns.name = name;
      ns.value = state;
      msg().planning_factors.emplace_back(ns);
    } else if (it->value != state) {
      it->value = state;
    }
  }

  auto clearPlanningFactors() -> void { msg().planning_factors.clear(); }
};

}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__COMMAND_WRAPPER_BASE_HPP_
