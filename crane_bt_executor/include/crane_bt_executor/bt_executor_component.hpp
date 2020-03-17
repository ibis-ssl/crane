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

#include <crane_bt_executor/utils/world_model.hpp>
#include <crane_bt_executor/role/role_id.hpp>
#include <crane_bt_executor/role/role_builder.hpp>
#include <crane_msgs/msg/role_commands.hpp>
#include <rclcpp/rclcpp.hpp>

enum class ChangeCode{
    NOCHANGE,
    PARAM_CHANGE,
    ASSIGN_CHANGE,
    ROLE_CHANGE
};
class BTExecutorComponent : public rclcpp::Node {
public:
  BTExecutorComponent() : Node("bt_executor_node")
  {
    role_commands_sub_ = create_subscription<crane_msgs::msg::RoleCommands>(
        "/crane_role_assignor/role_commands", 1,
        std::bind(&BTExecutorComponent::callbackRoleCommands, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "SUBSCRIBER [/crane_role_assignor/role_commands] SET UP!");

      wm_sub_ = create_subscription<crane_msgs::msg::WorldModel>(
              "/world_model_node/world_model", 1,
              std::bind(&BTExecutorComponent::test, this, std::placeholders::_1));
  }

  void callbackRoleCommands(crane_msgs::msg::RoleCommands::ConstSharedPtr msg){
    world_model.update(msg->world_model);
    auto change_code = checkChange(msg);
    for(auto cmd : msg->commands){
      auto role = role_builder.build(static_cast<RoleID>(RoleID::DEFENDER));
    }
    prev_cmds = *msg;
  }

  ChangeCode checkChange(crane_msgs::msg::RoleCommands::ConstSharedPtr msg){
    if(prev_cmds.commands.empty()){
      return ChangeCode::ROLE_CHANGE;
    }

    //  role change check
    std::vector<uint8_t> prev_role,current_role;
    for(auto cmd : prev_cmds.commands){
        prev_role.emplace_back(cmd.role_id);
    }
    for(auto cmd : msg->commands){
        current_role.emplace_back(cmd.role_id);
    }
    std::sort(prev_role.begin(),prev_role.end());
    std::sort(current_role.begin(),current_role.end());

    if(prev_role != current_role){
      return ChangeCode ::ROLE_CHANGE;
    }

    // assign change check
    // parameter change check

  }
  void test(crane_msgs::msg::WorldModel::ConstSharedPtr msg){
      auto role_cmds = std::make_shared<crane_msgs::msg::RoleCommands>();

      role_cmds->world_model = *msg;
      crane_msgs::msg::RoleCommand cmd;

      cmd.role_id = static_cast<uint8_t>(RoleID::DEFENDER);
      cmd.robot_ids.emplace_back(0);
      cmd.robot_ids.emplace_back(1);

      role_cmds->commands.emplace_back(cmd);

      callbackRoleCommands(role_cmds);
  }

private:
  rclcpp::Subscription<crane_msgs::msg::RoleCommands>::SharedPtr role_commands_sub_;
    rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr wm_sub_;
  WorldModel world_model;
  RoleBuilder role_builder;
  crane_msgs::msg::RoleCommands prev_cmds;
};
#endif  // CRANE_BT_EXECUTOR__BT_EXECUTOR_COMPONENT_HPP_

