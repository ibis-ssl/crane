// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__SIMPLE_AI_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__SIMPLE_AI_PLANNER_HPP_

#include <algorithm>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/action/skill_execution.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_plugins/planner_base.hpp>
#include <crane_robot_skills/attacker.hpp>
#include <crane_robot_skills/skills.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
template <typename T>
std::string getStringFromArray(const std::vector<T> & array)
{
  std::stringstream ss;
  for (const auto & e : array) {
    // uint8_tがcharとして出力されるの防ぐ
    if constexpr (std::is_same_v<T, uint8_t>) {
      ss << static_cast<int>(e) << ", ";
    } else {
      ss << e << ", ";
    }
  }
  // 最後のカンマを取り除く
  if (ss.str().size() > 2) {
    return ss.str().substr(0, ss.str().size() - 2);
  } else {
    return ss.str();
  }
}

struct Task
{
  std::string getText() const
  {
    std::string str = name + "(";
    str += ")";
    return str;
  }
  std::string name;

  std::unordered_map<std::string, skills::ParameterType> parameters;

  std::shared_ptr<skills::SkillInterface> skill = nullptr;

  double retry_time = -1.0;

  std::chrono::time_point<std::chrono::steady_clock> start_time;

  bool retry() const
  {
    if (retry_time <= 0.0) {
      return false;
    }
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
    return duration.count() < retry_time * 1000;
  }

  double getRestTime() const
  {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
    return std::max(retry_time * 1000. - duration.count(), 0.0) / 1000;
  }
};

class SimpleAIPlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC explicit SimpleAIPlanner(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node);

  ~SimpleAIPlanner();

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getSelectedRobots(
    [[maybe_unused]] uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override;

  template <class SkillType>
  void setUpSkillDictionary()
  {
    auto wm = std::make_shared<crane::WorldModelWrapper>(*action_node);
    auto skill = std::make_shared<SkillType>(static_cast<uint8_t>(0), wm);
    skill_generators[skill->name] =
      [](
        [[maybe_unused]] const std::string & name, uint8_t id,
        const std::shared_ptr<WorldModelWrapper> & wm) -> std::shared_ptr<skills::SkillInterface> {
      return std::make_shared<SkillType>(id, wm);
    };
  }

  std::unordered_map<
    std::string,
    std::function<std::shared_ptr<skills::SkillInterface>(
      const std::string & name, uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)>>
    skill_generators;

  rclcpp_action::Server<crane_msgs::action::SkillExecution>::SharedPtr skill_execution_server;

  std::shared_ptr<rclcpp_action::ServerGoalHandle<crane_msgs::action::SkillExecution>>
    skill_execution_goal_handle;

  std::shared_ptr<skills::SkillInterface> running_skill = nullptr;

  skills::Status skill_status;

  std::unordered_map<std::string, skills::ParameterType> parameters;

  uint8_t robot_id = 0;

  rclcpp::Node::SharedPtr action_node;

  WorldModelWrapper::SharedPtr world_model;

  rclcpp::TimerBase::SharedPtr action_sync_timer;
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__SIMPLE_AI_PLANNER_HPP_
