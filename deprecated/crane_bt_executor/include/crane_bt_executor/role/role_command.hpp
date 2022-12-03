// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__ROLE__ROLE_COMMAND_HPP_
#define CRANE_BT_EXECUTOR__ROLE__ROLE_COMMAND_HPP_

#include <map>
#include <string>
#include <vector>

#include "crane_msgs/msg/role_command.hpp"

class RoleCommand
{
public:
  enum class State
  {
    PARAM_CHANGE,
    ASSIGN_CHANGE,
  };
  explicit RoleCommand(crane_msgs::msg::RoleCommand cmd)
  {
    sub_role_ = cmd.sub_role;
    for (int i = 0; i < cmd.params.size(); i++)
    {
      parameter_[cmd.param_names.at(i)] = cmd.params.at(i);
    }
  }

  State checkChange(crane_msgs::msg::RoleCommand new_cmd)
  {
    if (new_cmd.sub_role != sub_role_)
    {
      sub_role_ = new_cmd.sub_role;
      return State::ASSIGN_CHANGE;
    }

    return State::PARAM_CHANGE;
  }

private:
  State state_;
  std::vector<crane_msgs::msg::SubRole> sub_role_;
  std::map<std::string, float> parameter_;
};
#endif  // CRANE_BT_EXECUTOR__ROLE__ROLE_COMMAND_HPP_
