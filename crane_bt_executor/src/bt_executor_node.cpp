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

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <chrono>
#include <crane_msgs/msg/world_model.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class BTExecutorNode : public rclcpp::Node {
public:
  BTExecutorNode() : Node("bt_executor_node") {
    // Grootへ実行情報を送信する
    publisher_zmq = std::make_unique<BT::PublisherZMQ>(tree);
    world_model_sub = create_subscription<crane_msgs::msg::WorldModel>(
        "/world_model", 10,
        std::bind(&BTExecutorNode::callbackWorldModel, this, _1));
    timer = create_wall_timer(500ms,
                              std::bind(&BTExecutorNode::timerCallback, this));
  }

  void initTree() {}

  void updateTree() {
    publisher_zmq = std::make_unique<BT::PublisherZMQ>(tree, 1666);
  }

private:
  void timerCallback() {
    tree.root_node->executeTick();
    RCLCPP_INFO(this->get_logger(), "tick");
  }
  void
  callbackWorldModel(const crane_msgs::msg::WorldModel::SharedPtr msg) const {
    // TODO : world_model
  }

private:
  rclcpp::TimerBase::SharedPtr timer;
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  std::unique_ptr<BT::PublisherZMQ> publisher_zmq;
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr world_model_sub;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BTExecutorNode>());
  rclcpp::shutdown();
  return 0;
}
