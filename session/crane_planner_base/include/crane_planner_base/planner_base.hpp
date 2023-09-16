// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_BASE__PLANNER_BASE_HPP_
#define CRANE_PLANNER_BASE__PLANNER_BASE_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "crane_geometry/eigen_adapter.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/robot_commands.hpp"
#include "crane_msgs/srv/robot_select.hpp"

namespace crane
{
class PlannerBase
{
public:
  explicit PlannerBase(const std::string name, rclcpp::Node & node) : name(name)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("session/" + name + "/robot_select"), "PlannerBase::PlannerBase");
    world_model = std::make_shared<WorldModelWrapper>(node);
    control_target_publisher =
      node.create_publisher<crane_msgs::msg::RobotCommands>("/control_targets", 1);
    using namespace std::placeholders;
    robot_select_srv = node.create_service<crane_msgs::srv::RobotSelect>(
      "session/" + name + "/robot_select",
      std::bind(&PlannerBase::handleRobotSelect, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger("session/" + name + "/robot_select"), "service created");

    world_model->addCallback([&](void) -> void {
      if (robots.empty()) {
        return;
      }
      auto control_targets = calculateControlTarget(robots);
      crane_msgs::msg::RobotCommands msg;
      msg.is_yellow = world_model->isYellow();
      for (auto target : control_targets) {
        msg.robot_commands.emplace_back(target);
      }
      control_target_publisher->publish(msg);
    });
  }

  void handleRobotSelect(
    const crane_msgs::srv::RobotSelect::Request::SharedPtr request,
    const crane_msgs::srv::RobotSelect::Response::SharedPtr response)
  {
    //    RCLCPP_INFO(rclcpp::get_logger("session/" + name + "/robot_select"), "request received");
    std::vector<std::pair<int, double>> robot_with_score;
    for (auto id : request->selectable_robots) {
      robot_with_score.emplace_back(id, getRoleScore(world_model->getRobot({true, id})));
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
    robots.clear();
    for (auto id : response->selected_robots) {
      RobotIdentifier robot_id{true, id};
      robots.emplace_back(robot_id);
    }

    for (auto && callback : robot_select_callbacks) {
      callback();
    }
  }

  void addRobotSelectCallback(std::function<void(void)> f)
  {
    robot_select_callbacks.emplace_back(f);
  }

protected:
  virtual auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots)
    -> std::vector<uint8_t> = 0;

  auto getSelectedRobotsByScore(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    std::function<double(const std::shared_ptr<RobotInfo> &)> score_func) -> std::vector<uint8_t>
  {
    std::vector<std::pair<int, double>> robot_with_score;
    for (const auto & id : selectable_robots) {
      robot_with_score.emplace_back(id, score_func(world_model->getRobot({true, id})));
    }
    std::sort(
      std::begin(robot_with_score), std::end(robot_with_score),
      [](const auto & a, const auto & b) -> bool {
        // greater score forst
        return a.second > b.second;
      });

    std::vector<uint8_t> selected_robots;
    for (int i = 0; i < selectable_robots_num; i++) {
      if (i >= robot_with_score.size()) {
        break;
      }
      selected_robots.emplace_back(robot_with_score.at(i).first);
    }
    return selected_robots;
  }

  const std::string name;

  WorldModelWrapper::SharedPtr world_model;

  rclcpp::Service<crane_msgs::srv::RobotSelect>::SharedPtr robot_select_srv;

  std::vector<RobotIdentifier> robots;

  virtual std::vector<crane_msgs::msg::RobotCommand> calculateControlTarget(
    const std::vector<RobotIdentifier> & robots) = 0;

private:
  rclcpp::Publisher<crane_msgs::msg::RobotCommands>::SharedPtr control_target_publisher;

  std::vector<std::function<void(void)>> robot_select_callbacks;
};

}  // namespace crane
#endif  // CRANE_PLANNER_BASE__PLANNER_BASE_HPP_
