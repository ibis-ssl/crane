// Copyright (c) 2021 ibis-ssl
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

#ifndef CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_

#include <functional>
#include <memory>

#include "RVO.h"
#include "crane_local_planner/visibility_control.h"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/robot_commands.hpp"
#include "crane_msgs/msg/world_model.hpp"
#include "rclcpp/rclcpp.hpp"

namespace crane
{
class LocalPlanner : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit LocalPlanner(const rclcpp::NodeOptions & options)
  : rclcpp::Node("local_planner", options)
  {
    float time_step = 1.0 / 30.0f;
    float neighbor_dist = 2.0f;
    size_t max_neighbors = 5;
    float time_horizon = 10.f;
    float time_horizon_obst = 10.f;
    float radius = 0.09f;
    float max_speed = 3.0f;
    rvo_sim_ = std::make_unique<RVO::RVOSimulator>(
      time_step, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, radius, max_speed);

    // TODO(HansRobo): add goal area as obstacles

    // TODO(HansRobo): add external area as obstacles
    // TODO(HansRobo): make agents
    // friend robots -> 0~19
    // enemy robots -> 20~39
    for (int i = 0; i < 40; i++) {
      rvo_sim_->addAgent(RVO::Vector2(20.0f, 20.0f));
    }

    commnads_pub_ = this->create_publisher<crane_msgs::msg::RobotCommands>("/robot_commands", 10);
    raw_commands_sub_ = this->create_subscription<crane_msgs::msg::RobotCommands>(
      "/robot_commands_raw", 10,
      std::bind(&LocalPlanner::rawCommandsCallback, this, std::placeholders::_1));
  }

  void worldModeCallback(const crane_msgs::msg::WorldModel & msg)
  {
    world_model_.update(msg);
    has_world_model_updated_ = true;
  }
  void rawCommandsCallback(crane_msgs::msg::RobotCommands::ConstSharedPtr msg)
  {
    if (!has_world_model_updated_) {
      return;
    }
    // update robot position and set preffered velocity
    int index = 0;
    for (const auto & friend_robot : world_model_.ours.robots) {
      if (friend_robot->available) {
        auto robot_cmd = std::find_if(
          msg->robot_commands.begin(), msg->robot_commands.end(),
          [index](const auto & x) { return x.robot_id == index; });
        if (robot_cmd == msg->robot_commands.end()) {
          continue;
        }
        const auto & pos = friend_robot->pose.pos;
        const auto & vel = RVO::Vector2(robot_cmd->target.x, robot_cmd->target.y);
        rvo_sim_->setAgentPosition(friend_robot->id, RVO::Vector2(pos.x(), pos.y()));
        rvo_sim_->setAgentPrefVelocity(friend_robot->id, vel);
      } else {
        rvo_sim_->setAgentPosition(friend_robot->id, RVO::Vector2(20.f, 20.f));
        rvo_sim_->setAgentPrefVelocity(friend_robot->id, RVO::Vector2(0.f, 0.f));
      }
      index++;
    }
    for (const auto & enemy_robot : world_model_.theirs.robots) {
      if (enemy_robot->available) {
        const auto & pos = enemy_robot->pose.pos;
        const auto & vel = enemy_robot->vel.linear;
        ;
        rvo_sim_->setAgentPosition(enemy_robot->id + 20, RVO::Vector2(pos.x(), pos.y()));
        rvo_sim_->setAgentPrefVelocity(enemy_robot->id + 20, RVO::Vector2(vel.x(), vel.y()));
      } else {
        rvo_sim_->setAgentPosition(enemy_robot->id + 20, RVO::Vector2(20.f, 20.f));
        rvo_sim_->setAgentPrefVelocity(enemy_robot->id + 20, RVO::Vector2(0.f, 0.f));
      }
    }
    //update rvo simulator
    rvo_sim_->doStep();

    // apply fixed velocity;
    crane_msgs::msg::RobotCommands new_commands;
    new_commands.header = msg->header;
    new_commands.is_yellow = msg->is_yellow;
    for (int i = 0; i < msg->robot_commands.size(); i++) {
      const auto & robot_cmd = msg->robot_commands.at(i);
      crane_msgs::msg::RobotCommand new_robot_cmd = robot_cmd;
      auto vel = rvo_sim_->getAgentVelocity(new_robot_cmd.robot_id);
      new_robot_cmd.target.x = vel.x();
      new_robot_cmd.target.y = vel.y();
      new_commands.robot_commands.emplace_back(new_robot_cmd);
    }
    commnads_pub_->publish(new_commands);
  }

private:
  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr raw_commands_sub_;
  rclcpp::Publisher<crane_msgs::msg::RobotCommands>::SharedPtr commnads_pub_;
  std::unique_ptr<RVO::RVOSimulator> rvo_sim_;
  WorldModelWrapper world_model_;
  bool has_world_model_updated_ = false;
};

}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
