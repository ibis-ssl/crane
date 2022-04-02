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

#ifndef CRANE_BT_EXECUTOR__BT_EXECUTOR_COMPONENT_HPP_
#define CRANE_BT_EXECUTOR__BT_EXECUTOR_COMPONENT_HPP_

#include <vector>
#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"

//#include "crane_world_observer/world_model.hpp"

#include "crane_bt_executor/role/role_id.hpp"
#include "crane_bt_executor/role/role_builder.hpp"
#include "crane_bt_executor/role/test/test_move.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msg_wrappers/role_command_wrapper.hpp"
#include "crane_msgs/msg/role_commands.hpp"
#include "crane_msgs/msg/robot_commands.hpp"

enum class ChangeCode
{
  NOCHANGE,
  PARAM_CHANGE,
  ASSIGN_CHANGE,
  ROLE_CHANGE
};
class BTExecutorComponent : public rclcpp::Node
{
public:
  BTExecutorComponent()
  : Node("bt_executor_node")
  {
    world_model_ = std::make_shared<WorldModelWrapper>();

    role_commands_sub_ = create_subscription<crane_msgs::msg::RoleCommands>(
      "/crane_role_assignor/role_commands", 1,
      std::bind(&BTExecutorComponent::callbackRoleCommands, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "SUBSCRIBER [/crane_role_assignor/role_commands] SET UP!");

    wm_sub_ = create_subscription<crane_msgs::msg::WorldModel>("/world_observer/world_model", 1,
        std::bind(&BTExecutorComponent::test, this, std::placeholders::_1));

    cmds_pub_ = create_publisher<crane_msgs::msg::RobotCommands>("robot_commands", 1);

    for (auto & role : roles_) {
      role.is_valid = false;
    }
  }

  void callbackRoleCommands(crane_msgs::msg::RoleCommands::SharedPtr msg)
  {
    world_model_->update(msg->world_model);
    bool is_changed = checkRoleChange(msg);
    if (is_changed) {
      std::cout << "cmd changed!" << std::endl;
      for (auto & role : roles_) {
        role.is_valid = false;
      }
      for (auto cmd : msg->commands) {
        std::cout << "role added : " << cmd.role_id << std::endl;
        roles_.at(cmd.role_id).role = role_builder_.build(static_cast<RoleID>(cmd.role_id));
        roles_.at(cmd.role_id).is_valid = true;
      }
    }

    crane_msgs::msg::RobotCommands robot_cmds;
    for (auto & role : roles_) {
      if (role.is_valid) {
        role.role->update(world_model_);
        role.role->getCommands(robot_cmds.robot_commands);
      }
    }

    robot_cmds.is_yellow = false;
    robot_cmds.header = msg->world_model.header;
    cmds_pub_->publish(robot_cmds);

    prev_cmds_ = *msg;
  }

  bool checkRoleChange(crane_msgs::msg::RoleCommands::ConstSharedPtr msg)
  {
    if (prev_cmds_.commands.empty()) {
      return true;
    }

    //  role change check
    std::vector<uint8_t> prev_role, current_role;
    for (auto cmd : prev_cmds_.commands) {
      prev_role.emplace_back(cmd.role_id);
    }
    for (auto cmd : msg->commands) {
      current_role.emplace_back(cmd.role_id);
    }
    std::sort(prev_role.begin(), prev_role.end());
    std::sort(current_role.begin(), current_role.end());

    if (prev_role != current_role) {
      return true;
    }

    return false;
  }
  void test(crane_msgs::msg::WorldModel::ConstSharedPtr msg)
  {
    auto role_cmds = std::make_shared<crane_msgs::msg::RoleCommands>();

    role_cmds->world_model = *msg;
    RoleCommandWrapper wrapper;
    wrapper.setRoleID(static_cast<uint8_t>(RoleID::TEST_MOVE));
    for (int i = 0; i < 8; i++) {
      wrapper.addSubRole(crane_msgs::msg::SubRole::ROLE1, i);
    }

    role_cmds->commands.emplace_back(wrapper.getMsg());
    callbackRoleCommands(role_cmds);
  }

  void testDefense(crane_msgs::msg::WorldModel::ConstSharedPtr msg)
  {
    auto role_cmds = std::make_shared<crane_msgs::msg::RoleCommands>();
    role_cmds->world_model = *msg;

    RoleCommandWrapper wrapper;
    wrapper.setRoleID(0);  // TODO(HansRobo): make enum for role_id
    wrapper.addSubRole(crane_msgs::msg::SubRole::GOALIE, 0);
    wrapper.addSubRole(crane_msgs::msg::SubRole::FIRST_DEFENDER, 1);
    wrapper.addSubRole(crane_msgs::msg::SubRole::SECOND_DEFENDER, 2);

    role_cmds->commands.emplace_back(wrapper.getMsg());
    callbackRoleCommands(role_cmds);
  }

private:
  rclcpp::Subscription<crane_msgs::msg::RoleCommands>::SharedPtr role_commands_sub_;
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr wm_sub_;
  rclcpp::Publisher<crane_msgs::msg::RobotCommands>::SharedPtr cmds_pub_;
  std::shared_ptr<WorldModelWrapper> world_model_;
  RoleBuilder role_builder_;
  crane_msgs::msg::RoleCommands prev_cmds_;
  struct RoleBox
  {
    bool is_valid;
    std::shared_ptr<RoleBase> role;
  };
  std::array<RoleBox, static_cast<uint8_t>(RoleID::ROLE_ID_NUM)> roles_;
};
#endif  // CRANE_BT_EXECUTOR__BT_EXECUTOR_COMPONENT_HPP_