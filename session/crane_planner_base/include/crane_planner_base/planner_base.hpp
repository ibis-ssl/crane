// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_TEMPLATE__PLANNER_TEMPLATE_HPP_
#define CRANE_PLANNER_TEMPLATE__PLANNER_TEMPLATE_HPP_

#include <functional>
#include <memory>

#include "crane_geometry/eigen_adapter.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/robot_commands.hpp"
#include "crane_msgs/srv/robot_select.hpp"
#include <rclcpp/rclcpp.hpp>

namespace crane
{
class PlannerBase
{
public:
  explicit PlannerBase(const std::string name, rclcpp::Node & node) : name_(name)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("session/" + name_ + "/robot_select"), "PlannerBase::PlannerBase");
    world_model_ = std::make_shared<WorldModelWrapper>(node);
    control_target_publisher_ =
      node.create_publisher<crane_msgs::msg::RobotCommands>("/control_targets", 1);
    using namespace std::placeholders;
    robot_select_srv_ = node.create_service<crane_msgs::srv::RobotSelect>(
      "session/" + name_ + "/robot_select",
      std::bind(&PlannerBase::handleRobotSelect, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger("session/" + name_ + "/robot_select"), "service created");

    world_model_->addCallback([&](void) -> void {
      if (robots_.empty()) {
        return;
      }
      auto control_targets = calculateControlTarget(robots_);
      crane_msgs::msg::RobotCommands msg;
      for (auto target : control_targets) {
        msg.robot_commands.emplace_back(target);
      }
      control_target_publisher_->publish(msg);
    });
  }

  void handleRobotSelect(
    const crane_msgs::srv::RobotSelect::Request::SharedPtr request,
    const crane_msgs::srv::RobotSelect::Response::SharedPtr response)
  {
    RCLCPP_INFO(rclcpp::get_logger("session/" + name_ + "/robot_select"), "request received");
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

    for (auto && callback : robot_select_callbacks_) {
      callback();
    }
  }

  void addRobotSelectCallback(std::function<void(void)> f)
  {
    robot_select_callbacks_.emplace_back(f);
  }

protected:
  const std::string name_;
  WorldModelWrapper::SharedPtr world_model_;

  rclcpp::Service<crane_msgs::srv::RobotSelect>::SharedPtr robot_select_srv_;
  virtual double getRoleScore(std::shared_ptr<RobotInfo> robot) = 0;

  std::vector<RobotIdentifier> robots_;
  virtual std::vector<crane_msgs::msg::RobotCommand> calculateControlTarget(
    const std::vector<RobotIdentifier> & robots) = 0;

private:
  rclcpp::Publisher<crane_msgs::msg::RobotCommands>::SharedPtr control_target_publisher_;
  std::vector<std::function<void(void)>> robot_select_callbacks_;
};

}  // namespace crane
#endif  // CRANE_PLANNER_TEMPLATE__PLANNER_TEMPLATE_HPP_
