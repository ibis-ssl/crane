// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__PLANNER_BASE_HPP_
#define CRANE_PLANNER_PLUGINS__PLANNER_BASE_HPP_

#include <algorithm>
#include <crane_basics/eigen_adapter.hpp>
#include <crane_basics/stream.hpp>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
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

using PlannerContext = std::unordered_map<std::string, std::unordered_map<std::string, double>>;

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

  explicit PlannerBase(const std::string & name, WorldModelWrapper::SharedPtr & world_model)
  : name(name),
    world_model(world_model),
    visualizer(std::make_unique<CraneVisualizerBuffer::MessageBuilder>("session_planner/" + name))
  {
  }

  virtual ~PlannerBase() { visualizer->clearBuffer(); }

  crane_msgs::srv::RobotSelect::Response doRobotSelect(
    const crane_msgs::srv::RobotSelect::Request::SharedPtr request,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
  {
    crane_msgs::srv::RobotSelect::Response response;
    response.selected_robots = getSelectedRobots(
      request->selectable_robots_num, request->selectable_robots, prev_roles, context);

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

  auto getRobotCommands(PlannerContext & context) -> crane_msgs::msg::RobotCommands
  {
    auto [latest_status, robot_commands] = calculateRobotCommand(robots, context);
    auto wrong_ids =
      robot_commands |
      // remove robot_command.robot_id is included in robots
      ranges::views::filter([&](const auto & command) {
        return std::ranges::find_if(robots, [&](const auto & robot) {
                 return robot.id == command.robot_id;
               }) == robots.end();
      }) |
      ranges::views::transform([](const auto & command) { return command.robot_id; }) |
      ranges::to<std::vector>();
    if (not wrong_ids.empty()) {
      std::stringstream what;
      what << "RobotCommands from " << name << " planner includes wrong robot_id : " << wrong_ids
           << std::endl;
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("PlannerBase"), what.str());
    }
    status = latest_status;
    crane_msgs::msg::RobotCommands msg;
    msg.is_yellow = world_model->isYellow();
    msg.on_positive_half = world_model->onPositiveHalf();
    for (const auto & command : robot_commands) {
      msg.robot_commands.emplace_back(command);
    }
    visualizer->flush();
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
      std::ranges::sort(ours, [](const auto & a, const auto & b) -> bool { return a.id < b.id; });
      std::ranges::sort(others, [](const auto & a, const auto & b) -> bool { return a.id < b.id; });
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
    const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
    -> std::vector<uint8_t> = 0;

  auto getSelectedRobotsByScore(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::function<double(const std::shared_ptr<RobotInfo> &)> & score_func,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context,
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
    std::ranges::sort(robot_with_score, [](const auto & a, const auto & b) -> bool {
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
    const std::vector<RobotIdentifier> & robots, PlannerContext & context) = 0;

  Status status = Status::RUNNING;

  CraneVisualizerBuffer::MessageBuilder::UniquePtr visualizer;

private:
  std::vector<std::function<void(void)>> robot_select_callbacks;
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__PLANNER_BASE_HPP_
