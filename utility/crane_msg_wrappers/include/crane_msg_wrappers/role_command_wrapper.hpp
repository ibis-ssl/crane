// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__ROLE_COMMAND_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__ROLE_COMMAND_WRAPPER_HPP_

#include <crane_msgs/msg/role_command.hpp>
#include <string>

class RoleCommandWrapper
{
public:
  RoleCommandWrapper() { init(); }
  void init()
  {
    crane_msgs::msg::SubRole sub_role;
    sub_role.sub_role_id = crane_msgs::msg::SubRole::EMPTY;
    msg.sub_role.clear();
    for (int i = 0; i < 11; i++) {
      msg.sub_role.push_back(sub_role);
    }
    msg.params.clear();
    msg.param_names.clear();
  }
  void addSubRole(uint8_t sub_role_id, uint8_t robot_id)
  {
    msg.sub_role.at(robot_id).sub_role_id = sub_role_id;
  }
  void setRoleID(uint8_t id) { msg.role_id = id; }

  void addParam(std::string name, float value)
  {
    msg.param_names.push_back(name);
    msg.params.push_back(value);
  }

  crane_msgs::msg::RoleCommand getMsg() { return msg; }

private:
  crane_msgs::msg::RoleCommand msg;
};
#endif  // CRANE_MSG_WRAPPERS__ROLE_COMMAND_WRAPPER_HPP_
