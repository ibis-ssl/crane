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
  PlannerBase() {}

  virtual void construct(WorldModelWrapper::SharedPtr world_model) = 0;

  void construct(const std::string name, WorldModelWrapper::SharedPtr world_model)
  {
    this->name = name;
    this->world_model = world_model;
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
    });
  }

  void addRobotSelectCallback(std::function<void(void)> f)
  {
    robot_select_callbacks.emplace_back(f);
  }

  std::optional<std::vector<RobotIdentifier>> assign(
    std::vector<int> selectable_robots, const int selectable_robots_num)
  {
    std::vector<std::pair<int, double>> robot_with_score;
    for (auto id : selectable_robots) {
      robot_with_score.emplace_back(id, getRoleScore(world_model->getRobot({true, id})));
    }
    std::sort(
      std::begin(robot_with_score), std::end(robot_with_score),
      [](const auto & a, const auto & b) -> bool {
        // greater score forst
        return a.second > b.second;
      });

    robots.clear();
    for (int i = 0; i < selectable_robots_num; i++) {
      if (i >= robot_with_score.size()) {
        break;
      }
      robots.push_back({true, robot_with_score.at(i).first});
    }

    for (auto && callback : robot_select_callbacks) {
      callback();
    }

    return robots;
  }

protected:
  std::string name;

  virtual double getRoleScore(std::shared_ptr<RobotInfo> robot) = 0;

  std::vector<RobotIdentifier> robots;

  virtual std::vector<crane_msgs::msg::RobotCommand> calculateControlTarget(
    const std::vector<RobotIdentifier> & robots) = 0;

  WorldModelWrapper::SharedPtr world_model = nullptr;

private:
  std::vector<std::function<void(void)>> robot_select_callbacks;
};

}  // namespace crane
#endif  // CRANE_PLANNER_BASE__PLANNER_BASE_HPP_
