// Copyright (c) 2020 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

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
    for (int i = 0; i < cmd.params.size(); i++) {
      parameter_[cmd.param_names.at(i)] = cmd.params.at(i);
    }
  }

  State checkChange(crane_msgs::msg::RoleCommand new_cmd)
  {
    if (new_cmd.sub_role != sub_role_) {
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
