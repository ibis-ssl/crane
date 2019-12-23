// Copyright (c) 2019 ibis-ssl
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

#ifndef CRANE_BT_EXECUTOR__BT_EXECUTOR_HPP_
#define CRANE_BT_EXECUTOR__BT_EXECUTOR_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>


// using namespace std::chrono_literals;

class BTExecutorNode : public rclcpp::Node
{
public:
  BTExecutorNode()
  : rclcpp::Node("crane_bt_executor") {}

private:
  void timerCallback()
  {
    tree.root_node->executeTick();
    RCLCPP_INFO(this->get_logger(), "tick");
  }

  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  rclcpp::TimerBase::SharedPtr timer;
  std::unique_ptr<BT::PublisherZMQ> publisher_zmq;
};

#endif  // CRANE_BT_EXECUTOR__BT_EXECUTOR_HPP_
