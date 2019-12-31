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

#include <string>
#include <memory>

namespace robotx_behavior_tree
{
class ExampleAction : public BT::SyncActionNode
{
public:
  ExampleAction(
    const std::string & name,
    const BT::NodeConfiguration & config)
  :  BT::SyncActionNode(name, config)
  {
    setRegistrationID(name);
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("test_input")};
  }

protected:
  BT::NodeStatus tick() override {return BT::NodeStatus::FAILURE;}
};
}  // namespace robotx_behavior_tree

BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string & name,
      const BT::NodeConfiguration & config) {
      return std::make_unique<robotx_behavior_tree::ExampleAction>(name, config);
    };
  factory.registerBuilder<robotx_behavior_tree::ExampleAction>("Example", builder);
}
