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

#ifndef CRANE_UTILITY__ROLE_COMMAND_WRAPPER_HPP_
#define CRANE_UTILITY__ROLE_COMMAND_WRAPPER_HPP_

#include "crane_msgs/msg/role_command.hpp"
#include <string>

class RoleCommandWrapper
{
public:
  void init()
  {
    crane_msgs::msg::SubRole sub_role;
    sub_role.sub_role_id = crane_msgs::msg::SubRole::NULL;
    msg_.sub_role.clear();
    for (int i = 0; i < 11; i++) {
      msg_.sub_role.push_back(sub_role);
    }
    msg_.params.clear();
    msg_.param_names.clear();
  }
  void addSubRole(uint8_t sub_role_id, uint8_t robot_id)
  {
    msg_.sub_role.at(robot_id) = sub_role_id;
  }
  void setRoleID(uint8_t id)
  {
    msg_.role_id = id;
  }

  void addParam(std::string name, float value)
  {
    msg.param_names.push_back(name);
    msg.params.push_back(value);
  }

  crane_msgs::msg::RoleCommand getMsg()
  {
    return msg_;
  }

private:
  crane_msgs::msg::RoleCommand msg_;
};
#endif  // CRANE_UTILITY__ROLE_COMMAND_WRAPPER_HPP_
