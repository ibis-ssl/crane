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

#ifndef CRANE_TARGET_AREA_PLANNER__TARGET_AREA_PLANNER_HPP_
#define CRANE_TARGET_AREA_PLANNER__TARGET_AREA_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "crane_target_area_planner/visibility_control.hpp"
#include "eigen3/Eigen/Core"
#include "crane_msgs/msg/pass_info.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"

namespace crane
{
struct FutureKick
{
  float remaining_time;
  Eigen::Vector2f position;
};
class TargetAreaPlanner : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit TargetAreaPlanner(const rclcpp::NodeOptions & options)
  : rclcpp::Node("target_area_planner", options)
  {
    world_model_ = std::make_shared<WorldModelWrapper>();
    pass_info_sub_ = create_subscription<crane_msgs::msg::PassInfo>(
      "/receiver_planner/pass_info", 1,
      std::bind(&TargetAreaPlanner::passInfoCallback, this, std::placeholders::_1));
    pass_info_pub_ = create_publisher<crane_msgs::msg::PassInfo>("pass_info", 1);
  }
  void passInfoCallback(const crane_msgs::msg::PassInfo::SharedPtr msg)
  {
    world_model_->update(msg->world_model);
    FutureKick kick;
    kick.remaining_time = msg->passer_receive_time_s.data;
    kick.position << msg->passer_receive_position.x, msg->passer_receive_position.y;
    auto a = calcTarget(kick, msg->receiver_id.data);

  }

private:
  Eigen::Vector2f calcTarget(FutureKick kick, int receiver_id)
  {
    auto pos = world_model_->ours.robots.at(receiver_id)->pose.pos;
    float grid_size = 0.3f;
    float window_radius = 3.0f;

    float max_score = 0.0f;
    Eigen::Vector2f target;
    for (float x = pos.x() - window_radius; x <= pos.x() + window_radius; x += grid_size) {
      for (float y = pos.y() - window_radius; y <= pos.y() + window_radius; y += grid_size) {
        float score = calcScore(x, y);
        if (score > max_score) {
          target << x, y;
        }
      }
    }
    return target;
  }
  float calcScore(float x, float y)
  {
    auto field = world_model_->field_size;
    if (abs(x) > field.x() * 0.5f || abs(y) > field.y() * 0.5f) {
      return 0.0f;
    }
    // TODO(HansRobo): check enemy block
    // TODO(HansRobo):
    return 0.0f;
  }
  rclcpp::Subscription<crane_msgs::msg::PassInfo>::SharedPtr pass_info_sub_;
  rclcpp::Publisher<crane_msgs::msg::PassInfo>::SharedPtr pass_info_pub_;
  std::shared_ptr<WorldModelWrapper> world_model_;
};
}
#endif  // CRANE_TARGET_AREA_PLANNER__TARGET_AREA_PLANNER_HPP_
