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
#include "rclcpp/rclcpp.hpp"

#include "crane_pass_facilitator/visibility_control.h"
#include "crane_msgs/msg/world_model.hpp"
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
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr receive_point_pub_;
};

}  // namespace crane
#endif  // CRANE_PASS_FACILITATOR__PASS_FACILITATOR_HPP_
