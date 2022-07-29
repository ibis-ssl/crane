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
//#include "crane_msgs/msg/control_targets.hpp"
#include "crane_msgs/msg/robot_commands.hpp"
#include "crane_msgs/msg/world_model.hpp"
#include "rclcpp/rclcpp.hpp"

namespace crane
{
class LocalPlannerComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit LocalPlannerComponent(const rclcpp::NodeOptions & options)
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

    world_model_ = std::make_shared<WorldModelWrapper>(*this);

    // TODO(HansRobo): add goal area as obstacles

    // TODO(HansRobo): add external area as obstacles
    // friend robots -> 0~19
    // enemy robots -> 20~39
    for (int i = 0; i < 40; i++) {
      rvo_sim_->addAgent(RVO::Vector2(20.0f, 20.0f));
    }

    commnads_pub_ = this->create_publisher<crane_msgs::msg::RobotCommands>("/robot_commands", 10);
    control_targets_sub_ = this->create_subscription<crane_msgs::msg::RobotCommands>(
      "/control_targets", 10,
      std::bind(&LocalPlannerComponent::callbackControlTarget, this, std::placeholders::_1));
  }

  void callbackControlTarget(crane_msgs::msg::RobotCommands::ConstSharedPtr msg)
  {
    if (!world_model_->hasUpdated()) {
      return;
    }
    // update robot position and set preffered velocity
    int index = 0;
    for (const auto & friend_robot : world_model_->ours.robots) {
      if (friend_robot->available) {
        auto robot_target = std::find_if(
          msg->robot_commands.begin(), msg->robot_commands.end(),
          [index](const auto & x) { return x.robot_id == index; });

        const auto & pos = friend_robot->pose.pos;
        rvo_sim_->setAgentPosition(friend_robot->id, RVO::Vector2(pos.x(), pos.y()));
        if (robot_target == msg->robot_commands.end()) {
          // if the robot is not contained in control_targets,
          // set observed velocity as preffered velocity
          const auto vel = RVO::Vector2(friend_robot->vel.linear.x(), friend_robot->vel.linear.y());
          rvo_sim_->setAgentPrefVelocity(friend_robot->id, vel);
          continue;
        } else {
          if(robot_target->motion_mode_enable){
            // velocity control
            // set goal as a preffered velocity directly
            const auto vel = RVO::Vector2(robot_target->target.x, robot_target->target.y);
            rvo_sim_->setAgentPrefVelocity(friend_robot->id, vel);
          }else{
            // position control
            auto diff_pos = Point(robot_target->target.x,robot_target->target.y) - pos;
            {
              //trapezoidal velocity control
              double current_speed = friend_robot->vel.linear.norm();
              // TODO : import settings via topics
              constexpr double MAX_ACC = 1.0;
              constexpr double FRAME_RATE = 30;
              constexpr double MAX_SPEED = 3.0;
              double distance = diff_pos.norm();
              // 2ax = v^2 - v0^2
              // v^2 - 2ax = v0^2
              // v0 = sqrt(v^2 - 2ax)
              double max_speed_for_stop = std::sqrt(0*0 - 2.0*(-MAX_ACC)*distance);
              double max_speed_for_acc = current_speed + MAX_ACC/FRAME_RATE;

              double target_speed = std::min({max_speed_for_acc, max_speed_for_stop, MAX_SPEED});
              auto target_vel_eigen = diff_pos.normalized() * target_speed;
              const auto vel = RVO::Vector2(target_vel_eigen.x(), target_vel_eigen.y());
              rvo_sim_->setAgentPrefVelocity(friend_robot->id, vel);
            }
          }
        }
      } else {
        rvo_sim_->setAgentPosition(friend_robot->id, RVO::Vector2(20.f, 20.f));
        rvo_sim_->setAgentPrefVelocity(friend_robot->id, RVO::Vector2(0.f, 0.f));
      }
      index++;
    }
    for (const auto & enemy_robot : world_model_->theirs.robots) {
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
    crane_msgs::msg::RobotCommands commands;
//    commands.header = msg->header;
//    commands.is_yellow = msg->is_yellow;
    for (int i = 0; i < msg->robot_commands.size(); i++) {
      const auto & target = msg->robot_commands.at(i);
      crane_msgs::msg::RobotCommand command = target;
      command.current_theta = world_model_->getRobot({true, target.robot_id})->pose.theta;
      if(!target.motion_mode_enable){
        command.motion_mode_enable = true;
        auto vel = rvo_sim_->getAgentVelocity(target.robot_id);
        command.target.x = vel.x();
        command.target.y = vel.y();
        command.target.theta = target.target.theta;
      }
      commands.robot_commands.emplace_back(command);
    }
    commnads_pub_->publish(commands);
  }

private:
  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr control_targets_sub_;
  rclcpp::Publisher<crane_msgs::msg::RobotCommands>::SharedPtr commnads_pub_;
  std::unique_ptr<RVO::RVOSimulator> rvo_sim_;
  WorldModelWrapper::SharedPtr world_model_;
};

}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
