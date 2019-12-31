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
#include <behaviortree_cpp_v3/utils/shared_library.h>
#include <rclcpp/rclcpp.hpp>
#include <crane_msgs/msg/behavior_tree_command.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <memory>
#include <chrono>


 using namespace std::chrono_literals;

class BTExecutorNode : public rclcpp::Node {
public:
  explicit BTExecutorNode(uint8_t robot_id,std::vector<std::string> plugin_names) : Node("bt_executor_node") {
    //プラグイン読み込み
    BT::SharedLibrary loader;
    for(auto plugin : plugin_names){
      factory.registerFromPlugin(loader.getOSName(plugin));
      RCLCPP_INFO(this->get_logger(), "PLUGIN [%s] LOADED!",loader.getOSName(plugin).c_str());
    }
    RCLCPP_INFO(this->get_logger(), "PLUGIN LOAD..");
    // Grootへ実行情報を送信する
    publisher_zmq = nullptr;
    zmp_port = 1666 + robot_id;

    world_model_sub = create_subscription<crane_msgs::msg::WorldModel>(
        "/world_model",10,
        std::bind(&BTExecutorNode::callbackWorldModel, this,std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "SUBSCRIBER [/world_model] SET UP!");
    std::stringstream ss;
    ss << "/robot" << std::to_string(robot_id) << "/bt_cmd";
    bt_cmd_sub = create_subscription<crane_msgs::msg::BehaviorTreeCommand>(
        ss.str(),10,
        std::bind(&BTExecutorNode::callbackBTCommand,this,std::placeholders::_1)
        );
    RCLCPP_INFO(this->get_logger(), "SUBSCRIBER [%s] SET UP!",ss.str().c_str());

    timer = create_wall_timer(500ms,
                              std::bind(&BTExecutorNode::timerCallback,this));
    RCLCPP_INFO(this->get_logger(), "TIMER SET UP!");
  }

  void initTree() {}

  void updateTree() {

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
  void callbackBTCommand(const crane_msgs::msg::BehaviorTreeCommand::SharedPtr msg){
    std::stringstream ss;
//    ss << <<
//    factory.createTreeFromFile()
    //
    publisher_zmq = std::make_unique<BT::PublisherZMQ>(tree, zmp_port);
  }

private:
  int zmp_port;
  rclcpp::TimerBase::SharedPtr timer;
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  std::unique_ptr<BT::PublisherZMQ> publisher_zmq;
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr world_model_sub;
  rclcpp::Subscription<crane_msgs::msg::BehaviorTreeCommand>::SharedPtr bt_cmd_sub;
};


#endif  // CRANE_BT_EXECUTOR__BT_EXECUTOR_HPP_
