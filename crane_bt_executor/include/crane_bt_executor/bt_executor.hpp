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

#include <crane_msgs/msg/behavior_tree_command.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <crane_bt_executor/utils/world_model.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/utils/shared_library.h>
#include <behaviortree_cpp_v3/xml_parsing.h>
#include <chrono>

#include <fstream>
#include <string>
#include <vector>
#include <utility>
#include <memory>


class BTExecutorNode : public rclcpp::Node
{
public:
  explicit BTExecutorNode(uint8_t robot_id, std::vector<std::string> plugin_names)
  : Node("bt_executor_node")
  {
    // プラグイン読み込み
    BT::SharedLibrary loader;
    for (auto plugin : plugin_names) {
      factory.registerFromPlugin(loader.getOSName(plugin));
      RCLCPP_INFO(this->get_logger(), "PLUGIN [%s] LOADED!", loader.getOSName(plugin).c_str());
    }

    // Create the blackboard that will be shared by all of the nodes in the tree
    blackboard = BT::Blackboard::create();

    // Put items on the blackboard
    auto world_model_ptr = std::shared_ptr<WorldModel>(&world_model);
    blackboard->set<std::shared_ptr<WorldModel>>("world_model", world_model_ptr);  // NOLINT
    // Read the input BT XML from the specified file into a string

    initTree("sample.xml");

    RCLCPP_INFO(this->get_logger(), "PLUGIN LOAD..");
    // Grootへ実行情報を送信する
    publisher_zmq = nullptr;
    zmp_port = 1666 + robot_id;

    world_model_sub = create_subscription<crane_msgs::msg::WorldModel>(
      "/world_model", 10,
      std::bind(&BTExecutorNode::callbackWorldModel, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "SUBSCRIBER [/world_model] SET UP!");
    std::stringstream ss;
    ss << "/robot" << std::to_string(robot_id) << "/bt_cmd";
    bt_cmd_sub = create_subscription<crane_msgs::msg::BehaviorTreeCommand>(
      ss.str(), 10,
      std::bind(&BTExecutorNode::callbackBTCommand, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "SUBSCRIBER [%s] SET UP!", ss.str().c_str());

    timer = create_wall_timer(std::chrono::milliseconds(500),
        std::bind(&BTExecutorNode::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "TIMER SET UP!");
  }

  void initTree(std::string xml_file_name)
  {
    xml_file_name = ament_index_cpp::get_package_share_directory("crane_bt_executor") +
      "/behavior_trees/" + xml_file_name;
    std::ifstream xml_file(xml_file_name);

    if (!xml_file.good()) {
      RCLCPP_ERROR(get_logger(), "Couldn't open input XML file: %s", xml_file_name.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "XML file [%s] LOADED!", xml_file_name.c_str());
    }

    std::string xml_string = std::string(std::istreambuf_iterator<char>(xml_file),
        std::istreambuf_iterator<char>());

    // Create the Behavior Tree from the XML input (after registering our own node types)
    //    BT::Tree temp_tree = bt_->buildTreeFromText(xml_string_, blackboard_);

    BT::XMLParser xml_parser(factory);
    xml_parser.loadFromText(xml_string);
    BT::Tree tmp_tree = xml_parser.instantiateTree(blackboard);

    tree = std::make_unique<BT::Tree>();
    tree->root_node = tmp_tree.root_node;
    tree->nodes = std::move(tmp_tree.nodes);
    tmp_tree.root_node = nullptr;
  }

private:
  void timerCallback()
  {
    tree->root_node->executeTick();
    RCLCPP_INFO(this->get_logger(), "tick");
  }
  void
  callbackWorldModel(const crane_msgs::msg::WorldModel::SharedPtr msg)
  {
    world_model.update(msg);
  }
  void callbackBTCommand(const crane_msgs::msg::BehaviorTreeCommand::SharedPtr msg)
  {
    std::stringstream ss;
    ss << std::to_string(msg->role_id) << ".xml";
    initTree(ss.str());
//    publisher_zmq = std::make_unique<BT::PublisherZMQ>(*tree, zmp_port);
  }

private:
  int zmp_port;
  rclcpp::TimerBase::SharedPtr timer;
  BT::BehaviorTreeFactory factory;
  std::unique_ptr<BT::Tree> tree;
  std::shared_ptr<BT::Blackboard> blackboard;
  std::unique_ptr<BT::PublisherZMQ> publisher_zmq;
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr world_model_sub;
  rclcpp::Subscription<crane_msgs::msg::BehaviorTreeCommand>::SharedPtr bt_cmd_sub;
  WorldModel world_model;
};


#endif  // CRANE_BT_EXECUTOR__BT_EXECUTOR_HPP_
