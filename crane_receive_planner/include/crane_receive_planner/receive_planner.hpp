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

#ifndef CRANE_RECEIVE_PLANNER__RECEIVE_PLANNER_HPP_
#define CRANE_RECEIVE_PLANNER__RECEIVE_PLANNER_HPP_

#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"

#include "crane_receive_planner/visibility_control.h"
#include "crane_msgs/msg/world_model.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/receiver_plan.hpp"
#include "crane_msgs/msg/pass_info.hpp"
#include "crane_geometry/eigen_adapter.hpp"
#include "crane_msgs/srv/pass_request.hpp"

namespace crane
{
class ReceivePlanner : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit ReceivePlanner(const rclcpp::NodeOptions & options)
  : rclcpp::Node("receive_planner", options)
  {
    pass_info_pub_ = create_publisher<crane_msgs::msg::PassInfo>("path_info", 1);
    using namespace std::placeholders;
    pass_req_service_ = create_service<crane_msgs::srv::PassRequest>("pass_request", std::bind(&ReceivePlanner::passRequestHandle,this,_1,_2,_3));
    world_model_ = std::make_shared<WorldModelWrapper>();
  }

  void passRequestHandle(const std::shared_ptr<rmw_request_id_t> request_header,
                           const std::shared_ptr<crane_msgs::srv::PassRequest::Request> request,
                           const std::shared_ptr<crane_msgs::srv::PassRequest::Response> response){
    RCLCPP_INFO(get_logger(), "receive pass request!");
    (void)request_header;
    pass_info_.passer_id = request->pass.passer_id;
    pass_info_.receiver_id = request->pass.receiver_id;
    publishReceivePoint(request->pass.passer_id.data);
    response->success = true;
    response->message = "publish receiver point successfully";
  }

  void publishReceivePoint(int passer_id)
  {
    auto & ball = world_model_->ball;
    auto pos = world_model_->ours.robots.at(passer_id)->pose.pos;
    //  こちらへ向かう速度成分
    float ball_vel = ball.vel.dot((pos - ball.pos).normalized());
    Point target;
    if (ball_vel > 0.5f) {
      //  ボールの進路上に移動
      ClosestPoint result;
      Segment ball_line(ball.pos, (ball.pos + ball.vel.normalized() * (ball.pos - pos).norm()));
      bg::closest_point(pos, ball_line, result);
      target = result.closest_point;
    }

    auto & recv_pos = pass_info_.passer_receive_position;
    recv_pos.x = target.x();
    recv_pos.y = target.y();
    recv_pos.z = 0.0;
    pass_info_pub_->publish(pass_info_);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr sub_world_model_;
  std::shared_ptr<WorldModelWrapper> world_model_;
  rclcpp::Publisher<crane_msgs::msg::PassInfo>::SharedPtr pass_info_pub_;
  rclcpp::Service<crane_msgs::srv::PassRequest>::SharedPtr pass_req_service_;
  crane_msgs::msg::PassInfo pass_info_;


  void world_model_callback(const crane_msgs::msg::WorldModel::SharedPtr msg)
  {
    world_model_->update(*msg);
    pass_info_.world_model = *msg;
  }
};

}  // namespace crane
#endif  // CRANE_RECEIVE_PLANNER__RECEIVE_PLANNER_HPP_
