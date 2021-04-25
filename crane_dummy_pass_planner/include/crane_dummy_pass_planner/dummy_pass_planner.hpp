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

#ifndef CRANE_DUMMY_PASS_PLANNER__DUMMY_PASS_PLANNER_HPP_
#define CRANE_DUMMY_PASS_PLANNER__DUMMY_PASS_PLANNER_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "crane_dummy_pass_planner/visibility_control.h"
#include "crane_msgs/msg/world_model.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"

namespace crane
{
class DummyPassPlanner : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit DummyPassPlanner(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr sub_world_model_;
  std::shared_ptr<WorldModelWrapper> world_model_;
  bool is_enabled_;
  int robot_id_;

  void world_model_callback(const crane_msgs::msg::WorldModel::SharedPtr msg)
  {
    world_model_->update(*msg);
    auto ball = world_model_->ball;
    auto robot = world_model_->ours.robots.at(robot_id_);

    auto pos = robot.info->pose.pos;

    float ball_vel = ball.vel.dot((pos - ball.pos).normalized());
    if (ball_vel < -0.5f) {  // 通り過ぎた
      return;
    } else if (ball_vel > 0.5f) {
      ClosestPoint result;
      Segment ball_line(ball.pos, (ball.pos + ball.vel.normalized() * (ball.pos - pos).norm()));
      bg::closest_point(pos, ball_line, result);
      robot.builder->addDribble(0.5f);
      robot.builder->setTargetPos(result.closest_point, false);
      robot.builder->setTargetTheta(tool::getAngle(ball.pos - pos));
    }
//    else {
//      robot.builder->setTargetPos(receive_pos);
//      robot.builder->setTargetTheta(tool::getAngle(ball.pos - pos));
//    }
  }
};

}  // namespace crane
#endif  // CRANE_DUMMY_PASS_PLANNER__DUMMY_PASS_PLANNER_HPP_
