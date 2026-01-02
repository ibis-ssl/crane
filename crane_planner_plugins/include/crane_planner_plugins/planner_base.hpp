// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__PLANNER_BASE_HPP_
#define CRANE_PLANNER_PLUGINS__PLANNER_BASE_HPP_

#include <algorithm>
#include <crane_geometry/vector2d_adapter.hpp>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/position_commands.hpp>
#include <crane_physics/position_assignments.hpp>
#include <crane_utils/stream.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace crane
{
using PlannerParameterType = std::variant<double, bool, int, std::string>;

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

  explicit PlannerBase(const std::string & name, WorldModelWrapper::SharedPtr & world_model)
  : name(name),
    world_model(world_model),
    visualizer(std::make_shared<VisualizerMessageBuilder>("session_planner/" + name))
  {
  }

  virtual ~PlannerBase() { visualizer->clearBuffer(); }

  auto selectRobots(
    const std::vector<uint8_t> & available_robots, const uint8_t max_selection_count,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
  {
    auto selected_robots = getSelectedRobots(max_selection_count, available_robots, prev_roles);

    robots.clear();
    for (auto id : selected_robots) {
      RobotIdentifier robot_id{true, id};
      robots.emplace_back(robot_id);
    }

    return selected_robots;
  }

  auto getPositionCommands() -> crane_msgs::msg::PositionCommands
  {
    auto [latest_status, position_commands] = calculatePositionCommand(robots);
    auto wrong_ids =
      position_commands |
      // remove position_command.robot_id is included in robots
      ranges::views::filter([&](const auto & command) {
        return std::ranges::find_if(robots, [&](const auto & robot) {
                 return robot.id == command.robot_id;
               }) == robots.end();
      }) |
      ranges::views::transform([](const auto & command) { return command.robot_id; }) |
      ranges::to<std::vector>();
    if (not wrong_ids.empty()) {
      std::stringstream what;
      what << "PositionCommands from " << name << " planner includes wrong robot_id : " << wrong_ids
           << std::endl;
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("PlannerBase"), what.str());
    }
    status = latest_status;
    crane_msgs::msg::PositionCommands msg;
    msg.is_yellow = world_model->isYellow();
    msg.on_positive_half = world_model->onPositiveHalf();
    for (const auto & command : position_commands) {
      msg.robot_commands.emplace_back(command);
    }
    visualizer->flush();
    return msg;
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

  const std::string name;

  // セッションパラメータ管理
  void setSessionParameters(const std::unordered_map<std::string, PlannerParameterType> & params)
  {
    session_params_ = params;
  }

  template <typename T>
  T getSessionParameter(const std::string & key, const T & default_value) const
  {
    auto it = session_params_.find(key);
    if (it != session_params_.end()) {
      if (auto * val = std::get_if<T>(&it->second)) {
        return *val;
      }
    }
    return default_value;
  }

  bool hasSessionParameter(const std::string & key) const { return session_params_.contains(key); }

protected:
  virtual auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> = 0;

  auto getSelectedRobotsByScore(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::function<double(const std::shared_ptr<RobotInfo> &)> & score_func,
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

  // ロボットを最適な位置に割り当て、位置コマンドを生成する共通メソッド
  auto assignRobotsToPoints(
    const std::vector<RobotIdentifier> & robots, const std::vector<Point> & target_points,
    const std::string & command_name, const Point & look_at_point,
    const std::function<void(std::shared_ptr<PositionCommandWrapper> &)> & customize_command =
      [](std::shared_ptr<PositionCommandWrapper> &) {})
    -> std::vector<crane_msgs::msg::PositionCommand>
  {
    if (robots.empty() || target_points.empty()) {
      return {};
    }

    // ロボットの現在位置を収集
    std::vector<Point> robot_points;
    for (const auto & robot_id : robots) {
      robot_points.emplace_back(world_model->getRobot(robot_id)->pose.pos);
    }

    // 最適割り当てを計算
    auto solution = getOptimalAssignments(robot_points, target_points);

    // 各ロボットに位置コマンドを生成
    std::vector<crane_msgs::msg::PositionCommand> position_commands;
    for (auto robot_id = robots.begin(); robot_id != robots.end(); ++robot_id) {
      int index = std::distance(robots.begin(), robot_id);
      Point target_point = target_points[solution[index]];

      auto command = std::make_shared<PositionCommandWrapper>(command_name, robot_id->id, world_model);

      command->setTargetPosition(target_point);
      command->setTargetTheta(getAngle(look_at_point - target_point));

      // カスタム設定を適用
      customize_command(command);

      position_commands.emplace_back(command->getMsg());
    }

    return position_commands;
  }

  std::vector<RobotIdentifier> robots;

  WorldModelWrapper::SharedPtr world_model;

  std::unordered_map<std::string, PlannerParameterType> session_params_;

  virtual std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) = 0;

  Status status = Status::RUNNING;

  VisualizerMessageBuilder::SharedPtr visualizer;
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__PLANNER_BASE_HPP_
