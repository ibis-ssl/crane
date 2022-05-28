// Copyright (c) 2021 ibis-ssl
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

#ifndef CRANE_PLANNER_TEMPLATE__PLANNER_TEMPLATE_HPP_
#define CRANE_PLANNER_TEMPLATE__PLANNER_TEMPLATE_HPP_

#include <functional>
#include <memory>

#include "crane_geometry/eigen_adapter.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/robot_commands.hpp."
#include "crane_msgs/srv/robot_select.hpp"
#include "rclcpp/rclcpp.hpp"

namespace crane
{

class PlannerBase
{
public:
  explicit PlannerBase(const std::string name, rclcpp::Node & node)
  {
    world_model_ = std::make_shared<WorldModelWrapper>(node);
    control_target_publisher_ = node.create_publisher<crane_msgs::msg::RobotCommands>("/control_targets")
    using namespace std::placeholders;
    robot_select_srv_ = node.create_service<crane_msgs::srv::RobotSelect>(
      "session/" + name + "/robot_select",
      std::bind(&PlannerBase::handleRobotSelect, this, _1, _2));
    world_model_->addCallback(
      [this](void) -> void {
        auto control_targets = calculateControlTarget(robots_);
        crane_msgs::msg::ControlTargets msg;
        for(auto target : control_targets){
          msg.control_targets.emplace_back(target);
        }

      });
  }

  void handleRobotSelect(
    const crane_msgs::srv::RobotSelect::Request::SharedPtr request,
    const crane_msgs::srv::RobotSelect::Response::SharedPtr response)
  {
    std::vector<std::pair<int, double>> robot_with_score;
    for (auto id : request->selectable_robots) {
      robot_with_score.emplace_back(id, getRoleScore(world_model_->getRobot({true, id})));
    }
    std::sort(
      std::begin(robot_with_score), std::end(robot_with_score),
      [](const auto & a, const auto & b) -> bool {
        // greater score forst
        return a.second > b.second;
      });
    response->selected_robots.clear();
    for (int i = 0; i < request->selectable_robots_num; i++) {
      if (i >= robot_with_score.size()) {
        break;
      }
      response->selected_robots.emplace_back(robot_with_score.at(i).first);
    }
    robots_.clear();
    for (auto id : response->selected_robots) {
      RobotIdentifier robot_id{true, id};
      robots_.emplace_back(robot_id);
    }
  }

protected:
  WorldModelWrapper::SharedPtr world_model_;

  rclcpp::Service<crane_msgs::srv::RobotSelect>::SharedPtr robot_select_srv_;
  virtual double getRoleScore(std::shared_ptr<RobotInfo> robot) = 0;

  std::vector<RobotIdentifier> robots_;
  virtual std::vector<crane_msgs::msg::RobotCommands> && calculateControlTarget(
    const std::vector<RobotIdentifier> & robots) = 0;
  rclcpp::Publisher<crane_msgs::msg::RobotCommands> control_target_publisher_;


};

}  // namespace crane
#endif  // CRANE_PLANNER_TEMPLATE__PLANNER_TEMPLATE_HPP_
