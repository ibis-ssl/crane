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
#include <crane_behavior_tree/world_model.hpp>
#include "robot_command_builder.hpp"

class ActionNode : public BT::SyncActionNode
{
public:
  ActionNode(
      const std::string & name,
      const BT::NodeConfiguration & config)
      :  BT::SyncActionNode(name, config)
  {
    setRegistrationID(name);
  }
  virtual std::pair<BT::NodeStatus,crane_msgs::msg::RobotCommand> update(const std::shared_ptr<WorldModel> world_model) = 0;

protected:
  BT::NodeStatus tick() final
  {
    std::shared_ptr<WorldModel> world_model;
    getInput("world_model", world_model);
    auto ret = update(world_model);

    return ret.first;
  }
};

class TransferActionNode : public ActionNode{
public:
  TransferActionNode(
      const std::string & name,
      const BT::NodeConfiguration & config)
      :  ActionNode(name, config)
  {}
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
        BT::InputPort<std::shared_ptr<WorldModel>>("world_model")
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }
};