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

class BTExecutorComponent : public rclcpp::Node {
public:
  BTExecutorComponent() : Node("bt_executor_node")
  {
    role_commands_sub_ = create_subscription<crane_msgs::msg::RoleCommands>(
        "/crane_role_assignor/role_commands", 1,
        std::bind(&BTExecutorComponent::callbackRoleCommands, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "SUBSCRIBER [/crane_role_assignor/role_commands] SET UP!");
  }

  void callbackRoleCommands(crane_msgs::msg::RoleCommands::ConstSharedPtr msg){
    world_model.update(msg->world_model);

    for(auto cmd : msg->commands){
      auto role = role_builder.build(static_cast<RoleID>(cmd.role_id));
    }
  }

  void test(){
      crane_msgs::msg::RoleCommands role_cmds;
      crane_msgs::msg::WorldModel wm;

      wm.field_info.x = 10;
      wm.field_info.y = 5;

      wm.ball_info.disappeared = false;
      wm.ball_info.detected = true;
      wm.ball_info.pose.x = 1.0f;
      wm.ball_info.pose.y = 1.0f;
      wm.ball_info.velocity.x = 1.0f;
      wm.ball_info.velocity.y = 1.0f;
      wm.ball_info.last_detection_pose.x = wm.ball_info.pose.x - 0.1f;
      wm.ball_info.last_detection_pose.y = wm.ball_info.pose.y - 0.1f;

      for(int i=0;i<3;i++){
          consai2r2_msgs::msg::RobotInfo info;
          info.robot_id = i;
          info.detected = true;
          info.disappeared = false;
          info.pose.x = -2;
          info.pose.y = -1 + i;
          info.pose.theta = M_PI;
          info.velocity.x = 0;
          info.velocity.y = 0;
          info.velocity.theta = 0;

          wm.robot_info_ours.emplace_back(info);
      }

      for(int i=0;i<3;i++){
          consai2r2_msgs::msg::RobotInfo info;
          info.robot_id = i;
          info.detected = true;
          info.disappeared = false;
          info.pose.x = 2;
          info.pose.y = -1 + i;
          info.pose.theta = 0;
          info.velocity.x = 0;
          info.velocity.y = 0;
          info.velocity.theta = 0;

          wm.robot_info_theirs.emplace_back(info);
      }

      wm.our_goalie_id = 0;
      wm.their_goalie_id = 0;

      role_cmds.world_model = wm;

      crane_msgs::msg::RoleCommand cmd;

      cmd.role_id = static_cast<uint8_t>(RoleID::DEFENDER);
      cmd.robot_ids.emplace_back(0);
      cmd.robot_ids.emplace_back(1);

      role_cmds.commands.emplace_back(cmd);

      crane_msgs::msg::RoleCommands::ConstSharedPtr msg(&role_cmds);
      callbackRoleCommands(msg);

  }

private:
  rclcpp::Subscription<crane_msgs::msg::RoleCommands>::SharedPtr role_commands_sub_;
  WorldModel world_model;
  RoleBuilder role_builder;
};
#endif  // CRANE_BT_EXECUTOR__BT_EXECUTOR_COMPONENT_HPP_

