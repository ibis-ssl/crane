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

#include <rclcpp/rclcpp.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <class_loader/visibility_control.hpp>

#include <memory>

class RealSenderNode : public rclcpp::Node {
public:
  CLASS_LOADER_PUBLIC
  explicit RealSenderNode(const rclcpp::NodeOptions &options) : Node("real_sender_node", options){
    sub_commnads_ = create_subscription<crane_msgs::msg::RobotCommands>("~/robot_commands",
        std::bind(&RealSenderNode::robotCommandsCallback, this, std::placeholders::_1));
  }
  void robotCommandsCallback(crane_msgs::msg::RobotCommands::ConstSharedPtr msg){
    // TODO(okada_tech) : send commands to robots
  }

private:
  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr sub_commnads_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;
  std::shared_ptr<RealSenderNode> real_sender_node =
      std::make_shared<RealSenderNode>(options);

  exe.add_node(real_sender_node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
