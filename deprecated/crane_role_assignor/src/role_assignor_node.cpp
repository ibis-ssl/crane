// Copyright (c) 2022 ibis-ssl
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

#include <iostream>
#include <memory>
#include <vector>
#include <functional>

class WorldModel
{
};

class ActionNode
{
public:
  using SharedPtr = std::shared_ptr<ActionNode>;
  explicit ActionNode(WorldModel wm, uint8_t robot_id) : world_model_(wm), ROBOT_ID_(robot_id)
  {
  }

  std::vector<std::shared_ptr<ActionNode>> children_;
  WorldModel world_model_;
  const uint8_t ROBOT_ID_;
  double node_feasibility_;
  double total_feasibility_;

  virtual bool findOptimalAction() = 0;

  virtual double calcActionFeasibility() = 0;

  virtual uint8_t getNextRobotID() = 0;
};

class RootAction : public ActionNode
{
public:
  using SharedPtr = std::shared_ptr<RootAction>;
  explicit RootAction(WorldModel wm, uint8_t robot_id) : ActionNode(wm, robot_id)
  {
    total_feasibility_ = 1.0;
    node_feasibility_ = 1.0;
  }

  bool findOptimalAction() override
  {
    return true;
  }

  double calcActionFeasibility() override
  {
    return 1;
  }

  uint8_t getNextRobotID() override
  {
    return ROBOT_ID_;
  }
};

class PassAction : public ActionNode
{
public:
  using SharedPtr = std::shared_ptr<PassAction>;
  explicit PassAction(WorldModel wm, uint8_t robot_id, uint8_t receiver_id)
    : ActionNode(wm, robot_id), RECEIVER_ID_(receiver_id)
  {
  }

  uint8_t RECEIVER_ID_;

  bool findOptimalAction() override
  {
    return false;
  }

  double calcActionFeasibility() override
  {
    return 0;
  }

  uint8_t getNextRobotID() override
  {
    return RECEIVER_ID_;
  }
};

class ShootAction : public ActionNode
{
public:
  using SharedPtr = std::shared_ptr<ShootAction>;
  explicit ShootAction(WorldModel wm, uint8_t robot_id) : ActionNode(wm, robot_id)
  {
  }

  bool findOptimalAction() override
  {
    return false;
  }

  double calcActionFeasibility() override
  {
    return 0;
  }

  uint8_t getNextRobotID() override
  {
    return 255;
  }
};

class DribbleAction : public ActionNode
{
public:
  using SharedPtr = std::shared_ptr<DribbleAction>;
  explicit DribbleAction(WorldModel wm, uint8_t robot_id) : ActionNode(wm, robot_id)
  {
  }

  bool findOptimalAction() override
  {
    return false;
  }

  double calcActionFeasibility() override
  {
    return 0;
  }

  uint8_t getNextRobotID() override
  {
    return ROBOT_ID_;
  }
};

class Tree
{
public:
  Tree(WorldModel wm, uint8_t robot_id)
  {
    root_node_ = std::make_shared<RootAction>(wm, robot_id);
    expand(root_node_);
  }

  ActionNode::SharedPtr root_node_;

  void execute()
  {
    expand(root_node_);
  }

  /**
   *
   * @param node
   * @param parent_total_feasibility
   */
  void evaluateFeasibility(ActionNode::SharedPtr node, double parent_total_feasibility)
  {
    node->findOptimalAction();
    node->node_feasibility_ = node->calcActionFeasibility();
    node->total_feasibility_ = parent_total_feasibility * node->node_feasibility_;
  }
  /**
   *
   * @param node
   * @return
   */
  bool expand(ActionNode::SharedPtr node)
  {
    if (!node->findOptimalAction())
    {
      return false;
    }
    node->node_feasibility_ = node->calcActionFeasibility();
    const int next_id = node->getNextRobotID();
    // shoot
    auto shoot = std::make_shared<ShootAction>(node->world_model_, next_id);
    //        node->children_.emplace_back(shoot);
    // pass
    constexpr int N = 8;
    for (int i = 0; i < N; i++)
    {
      if (i != next_id)
      {
        auto pass = std::make_shared<PassAction>(node->world_model_, next_id, i);
        //                node->children_.emplace_back(pass);
      }
    }

    // dribble
    auto dribble = std::make_shared<DribbleAction>(node->world_model_, next_id);
    //        node->children_.emplace_back(dribble);
  }

  void changeRootNode(ActionNode::SharedPtr next_root)
  {
    auto root = std::make_shared<RootAction>(next_root->world_model_, next_root->ROBOT_ID_);
    root->children_ = next_root->children_;
    this->root_node_ = root;

    updateTotalFeasibility();
  }
  void updateTotalFeasibility()
  {
    auto visitor = [](ActionNode::SharedPtr node, double parent_total_feasibility) {
      node->total_feasibility_ = parent_total_feasibility * node->node_feasibility_;
    };
    applyRecursiveVisitor(root_node_, 1.0, visitor);
  }
  void applyRecursiveVisitor(ActionNode::SharedPtr root, double parent_total_feasibility,
                             std::function<void(ActionNode::SharedPtr, double)> visitor)
  {
    visitor(root, parent_total_feasibility);
    for (const auto& child : root->children_)
    {
      applyRecursiveVisitor(child, root->total_feasibility_, visitor);
    }
  }
  void applyRecursiveVisitor(ActionNode::SharedPtr root, const std::function<void(ActionNode::SharedPtr)>& visitor)
  {
    visitor(root);

    for (const auto& child : root->children_)
    {
      applyRecursiveVisitor(child, visitor);
    }
  }
};

int main()
{
  std::cout << "hello, this is role_assignor_node" << std::endl;
  return 0;
}
