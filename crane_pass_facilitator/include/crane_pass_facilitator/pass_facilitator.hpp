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

#ifndef CRANE_PASS_FACILITATOR__PASS_FACILITATOR_HPP_
#define CRANE_PASS_FACILITATOR__PASS_FACILITATOR_HPP_

#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"

#include "crane_pass_facilitator/visibility_control.h"
#include "crane_msgs/srv/pass_request.hpp"
#include "crane_geometry/eigen_adapter.hpp"

namespace crane
{
class PassFacilitator : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit PassFacilitator(const rclcpp::NodeOptions & options)
  : rclcpp::Node("pass_facilitator", options)
  {
    using std::chrono_literals::operator""ms;
    receive_point_pub_ = create_publisher<geometry_msgs::msg::Point>("receive_point", 1);
    pass_req_client_ = create_client<crane_msgs::srv::PassRequest>("pass_request");
    auto req = std::make_shared<crane_msgs::srv::PassRequest::Request>();
    req->pass.receiver_id.data = 0;
    req->pass.passer_id.data = 1;
    auto response_future = pass_req_client_->async_send_request(req);

    crane_msgs::srv::PassRequest::Response::SharedPtr response;
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), response_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      response = response_future.get();
      RCLCPP_INFO(get_logger(),"%s",response->message.data());
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr receive_point_pub_;
  rclcpp::Client<crane_msgs::srv::PassRequest>::SharedPtr pass_req_client_;
};

}  // namespace crane
#endif  // CRANE_PASS_FACILITATOR__PASS_FACILITATOR_HPP_
