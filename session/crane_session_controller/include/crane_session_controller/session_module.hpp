// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_CONTROLLER__SESSION_MODULE_HPP_
#define CRANE_SESSION_CONTROLLER__SESSION_MODULE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "crane_msgs/srv/robot_select.hpp"
#include "crane_planner_base/planner_base.hpp"

namespace crane
{
class SessionModule
{
public:
  using SharedPtr = std::shared_ptr<SessionModule>;

  SessionModule(std::string name) : name(name) {}

  void construct(pluginlib::ClassLoader<PlannerBase> & plugin_loader, rclcpp::Node & node)
  {
     planner = plugin_loader.createSharedInstance(name);
    //    planner->initialize(node);
  }

  auto assign(
    std::vector<int> selectable_robots, const int selectable_robots_num)
  {
    std::cout << "Assigning robots to " << name << std::endl;
    return planner->assign(selectable_robots, selectable_robots_num);
  }

  void observe() {}

private:
  const std::string name;

  std::vector<int> assigned_robots;

  std::shared_ptr<PlannerBase> planner = nullptr;
};
}  // namespace crane
#endif  // CRANE_SESSION_CONTROLLER__SESSION_MODULE_HPP_
