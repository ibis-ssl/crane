// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_BASE__PLANNER_BASE_HPP_
#define CRANE_PLANNER_BASE__PLANNER_BASE_HPP_

#include <algorithm>
#include <crane_basics/eigen_adapter.hpp>
#include <crane_msg_wrappers/consai_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace crane
{
struct RobotRole
{
  std::string planner_name;
  std::string role_name;
  double score = 0.;
};

class PlannerBase
{
public:
  using SharedPtr = std::shared_ptr<PlannerBase>;

  using UniquePtr = std::unique_ptr<PlannerBase>;

  enum class Status {
    SUCCESS,
    FAILURE,
    RUNNING,
  };

  explicit PlannerBase(
    const std::string name, WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : name(name), world_model(world_model), visualizer(visualizer)
  {
  }

  crane_msgs::srv::RobotSelect::Response doRobotSelect(
    const crane_msgs::srv::RobotSelect::Request::SharedPtr request,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles)
  {
    crane_msgs::srv::RobotSelect::Response response;
    response.selected_robots =
      getSelectedRobots(request->selectable_robots_num, request->selectable_robots, prev_roles);

    robots.clear();
    for (auto id : response.selected_robots) {
      RobotIdentifier robot_id{true, id};
      robots.emplace_back(robot_id);
    }

    for (auto && callback : robot_select_callbacks) {
      callback();
    }
    return response;
  }

  auto getRobotCommands() -> crane_msgs::msg::RobotCommands
  {
    auto [latest_status, robot_commands] = calculateRobotCommand(robots);
    status = latest_status;
    crane_msgs::msg::RobotCommands msg;
    msg.is_yellow = world_model->isYellow();
    msg.on_positive_half = world_model->onPositiveHalf();
    for (const auto & command : robot_commands) {
      msg.robot_commands.emplace_back(command);
    }
    return msg;
  }

  void addRobotSelectCallback(std::function<void(void)> f)
  {
    robot_select_callbacks.emplace_back(f);
  }

  bool isSameConfiguration(PlannerBase * other_planner)
  {
    return name == other_planner->name && robots.size() == other_planner->robots.size() && [&]() {
      std::vector<RobotIdentifier> ours = this->robots;
      std::vector<RobotIdentifier> others = other_planner->robots;
      std::sort(ours.begin(), ours.end(), [](const auto & a, const auto & b) -> bool {
        return a.robot_id < b.robot_id;
      });
      std::sort(others.begin(), others.end(), [](const auto & a, const auto & b) -> bool {
        return a.robot_id < b.robot_id;
      });
      return ours == others;
    }();
  }

  Status getStatus() const { return status; }

  const std::vector<RobotIdentifier> & getRobots() const { return robots; }

  static std::shared_ptr<std::unordered_map<uint8_t, RobotRole>> robot_roles;

  const std::string name;

protected:
  virtual auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> = 0;

  auto getSelectedRobotsByScore(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    std::function<double(const std::shared_ptr<RobotInfo> &)> score_func,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles,
    std::function<double(const std::shared_ptr<RobotInfo> &)> hysteresis_func =
      [](const std::shared_ptr<RobotInfo> &) { return 0.; }) -> std::vector<uint8_t>
  {
    std::vector<std::pair<int, double>> robot_with_score;
    for (const auto & id : selectable_robots) {
      if (auto prev = prev_roles.find(id);
          prev != prev_roles.end() && prev->second.planner_name == name) {
        robot_with_score.emplace_back(
          id,
          score_func(world_model->getOurRobot(id)) + hysteresis_func(world_model->getOurRobot(id)));
      } else {
        robot_with_score.emplace_back(id, score_func(world_model->getOurRobot(id)));
      }
    }
    std::sort(
      std::begin(robot_with_score), std::end(robot_with_score),
      [](const auto & a, const auto & b) -> bool {
        // greater score first
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

  std::vector<RobotIdentifier> robots;

  WorldModelWrapper::SharedPtr world_model;

  virtual std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) = 0;

  ConsaiVisualizerWrapper::SharedPtr visualizer;

  Status status = Status::RUNNING;

private:
  std::vector<std::function<void(void)>> robot_select_callbacks;
};

}  // namespace crane
#endif  // CRANE_PLANNER_BASE__PLANNER_BASE_HPP_
