// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_msgs/msg/robot_feedback.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

class SimpleAINode : public rclcpp::Node
{
public:
  SimpleAINode(rclcpp::NodeOptions & options) : rclcpp::Node("simple_ai_node", options)
  {
    using namespace std::chrono_literals;
    timer = create_wall_timer(10ms, [&]() {

    });
  }

  rclcpp::TimerBase::SharedPtr timer;

//  rclcpp::Publisher<crane_msgs::msg::RobotFeedbackArray>::SharedPtr publisher;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;
  auto node = std::make_shared<SimpleAINode>(options);
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
