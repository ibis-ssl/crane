// Copyright (c) 2019, OUXT-Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <crane_behavior_tree/behavior_tree_node.hpp>

#include <string>
#include <memory>
#include <utility>

namespace ibis_behavior_tree
{
class ExampleAction : public TransferActionNode
{
public:
  ExampleAction(
    const std::string & name,
    const BT::NodeConfiguration & config)
  :  TransferActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("test_input")};
  }

protected:
//  BT::NodeStatus update(const std::shared_ptr<WorldModel> world_model) override
//  {
//    return BT::NodeStatus::FAILURE;
//  }

public:
  std::pair<BT::NodeStatus, crane_msgs::msg::RobotCommand>
  update(const std::shared_ptr<WorldModel> world_model) override
  {
    return std::pair<BT::NodeStatus, crane_msgs::msg::RobotCommand>();
  }
};
}  // namespace ibis_behavior_tree

BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string & name,
      const BT::NodeConfiguration & config) {
      return std::make_unique<ibis_behavior_tree::ExampleAction>(name, config);
    };
  factory.registerBuilder<ibis_behavior_tree::ExampleAction>("ExampleAction", builder);
}
