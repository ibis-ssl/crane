// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__SIMPLE_AI_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__SIMPLE_AI_PLANNER_HPP_

#include <algorithm>
#include <crane_basics/boost_geometry.hpp>
#include <crane_basics/interval.hpp>
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

class SimpleAIPlanner : public PlannerBase, public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC explicit SimpleAIPlanner(WorldModelWrapper::SharedPtr & world_model)
  : PlannerBase("SimpleAI", world_model), Node("SimpleAI")
  {
    {
      setUpSkillDictionary<skills::CmdKickWithChip>();
      setUpSkillDictionary<skills::CmdKickStraight>();
      setUpSkillDictionary<skills::CmdDribble>();
      setUpSkillDictionary<skills::CmdSetVelocity>();
      setUpSkillDictionary<skills::CmdSetTargetPosition>();
      setUpSkillDictionary<skills::CmdSetDribblerTargetPosition>();
      setUpSkillDictionary<skills::CmdSetTargetTheta>();
      setUpSkillDictionary<skills::CmdStopHere>();
      setUpSkillDictionary<skills::Teleop>();
      setUpSkillDictionary<skills::CmdDisablePlacementAvoidance>();
      setUpSkillDictionary<skills::CmdEnablePlacementAvoidance>();
      setUpSkillDictionary<skills::CmdDisableBallAvoidance>();
      setUpSkillDictionary<skills::CmdEnableBallAvoidance>();
      setUpSkillDictionary<skills::CmdDisableCollisionAvoidance>();
      setUpSkillDictionary<skills::CmdEnableCollisionAvoidance>();
      setUpSkillDictionary<skills::CmdDisableGoalAreaAvoidance>();
      setUpSkillDictionary<skills::CmdEnableGoalAreaAvoidance>();
      setUpSkillDictionary<skills::CmdSetGoalieDefault>();
      setUpSkillDictionary<skills::CmdEnableBallCenteringControl>();
      setUpSkillDictionary<skills::CmdEnableLocalGoalie>();
      setUpSkillDictionary<skills::CmdSetMaxVelocity>();
      setUpSkillDictionary<skills::Attacker>();
      setUpSkillDictionary<skills::CmdSetMaxAcceleration>();
      setUpSkillDictionary<skills::CmdSetTerminalVelocity>();
      setUpSkillDictionary<skills::CmdEnableStopFlag>();
      setUpSkillDictionary<skills::CmdDisableStopFlag>();
      setUpSkillDictionary<skills::CmdLiftUpDribbler>();
      setUpSkillDictionary<skills::CmdLookAt>();
      setUpSkillDictionary<skills::CmdLookAtBall>();
      setUpSkillDictionary<skills::CmdLookAtBallFrom>();
      setUpSkillDictionary<skills::GetBallContact>();
      setUpSkillDictionary<skills::Idle>();
      setUpSkillDictionary<skills::Goalie>();
      setUpSkillDictionary<skills::GoalKick>();
      setUpSkillDictionary<skills::Kick>();
      setUpSkillDictionary<skills::MoveWithBall>();
      setUpSkillDictionary<skills::Sleep>();
      setUpSkillDictionary<skills::Receive>();
      setUpSkillDictionary<skills::GoOverBall>();
      setUpSkillDictionary<skills::SimpleKickOff>();
      setUpSkillDictionary<skills::StealBall>();
      setUpSkillDictionary<skills::SubAttacker>();
      setUpSkillDictionary<skills::TestMotionPosition>();
      setUpSkillDictionary<skills::Marker>();
      setUpSkillDictionary<skills::SingleBallPlacement>();
      setUpSkillDictionary<skills::KickoffAttack>();
      setUpSkillDictionary<skills::KickoffSupport>();
      setUpSkillDictionary<skills::EmplaceRobot>();
    }
    using crane_msgs::action::SkillExecution;
    skill_execution_server = rclcpp_action::create_server<SkillExecution>(
      get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
      get_node_waitables_interface(), "/simple_ai/skill_execution",
      // ゴール（通常の指令）のコールバック
      [&](const rclcpp_action::GoalUUID, std::shared_ptr<const SkillExecution::Goal> goal)
        -> rclcpp_action::GoalResponse {
        std::cout << "Received goal: " << goal->name << std::endl;
        if (running_skill) {
          std::cout << "Skill is already running: " << goal->name << std::endl;
          return rclcpp_action::GoalResponse::REJECT;
        } else {
          if (auto skill_generator = skill_generators.find(goal->name);
              skill_generator != skill_generators.end()) {
            std::cout << "Start executing skill: " << goal->name << std::endl;
            auto command_base =
              std::make_shared<RobotCommandWrapperBase>(goal->name, goal->robot_id, world_model);
            running_skill = skill_generator->second(command_base);
            skill_status = skills::Status::RUNNING;
            parameters.clear();
            for (auto e : goal->parameter.bool_values) {
              parameters[e.name] = e.value;
            }
            for (auto e : goal->parameter.float_values) {
              parameters[e.name] = static_cast<double>(e.value);
            }
            for (auto e : goal->parameter.int_values) {
              parameters[e.name] = static_cast<int>(e.value);
            }
            for (auto e : goal->parameter.string_values) {
              parameters[e.name] = e.value;
            }
            for (auto e : goal->parameter.position_values) {
              Point p(e.x, e.y);
              parameters[e.name] = p;
            }
            std::cout << "Start executing skill: " << goal->name << std::endl;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
          } else {
            std::cerr << "No skill found: " << goal->name << std::endl;
            return rclcpp_action::GoalResponse::REJECT;
          }
        }
      },
      // キャンセルのコールバック
      [&](const std::shared_ptr<rclcpp_action::ServerGoalHandle<SkillExecution>> goal_handle)
        -> rclcpp_action::CancelResponse {
        std::cout << "Canceling goal: " << std::endl;
        skill_execution_goal_handle.reset();
        if (running_skill) {
          running_skill.reset();
        }
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      // 実行関数（ログの転送）
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<SkillExecution>> goal_handle)
        -> void {
        skill_execution_goal_handle = goal_handle;
        std::cout << "Executing goal: " << std::endl;
        // TODO(HansRobo): ログ転送の実装
        while (running_skill && skill_status == skills::Status::RUNNING) {
          std::cout << "Skill status: " << static_cast<int>(skill_status) << std::endl;
          const auto goal = goal_handle->get_goal();
          auto feedback = std::make_shared<SkillExecution::Feedback>();
          goal_handle->publish_feedback(feedback);
          rclcpp::sleep_for(std::chrono::milliseconds(100));  // 100ms待機
        }
        std::cout << "Goal succeeded: " << std::endl;
        auto result = std::make_shared<SkillExecution::Result>();
        goal_handle->succeed(result);
      });
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots, PlannerContext & context) override;

  auto getSelectedRobots(
    [[maybe_unused]] uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
    -> std::vector<uint8_t> override;

  template <class SkillType>
  void setUpSkillDictionary()
  {
    auto wm = std::make_shared<crane::WorldModelWrapper>(*this);
    auto command_base = std::make_shared<RobotCommandWrapperBase>("simple_ai", 0, wm);
    auto skill = std::make_shared<SkillType>(command_base);
    Task default_task;
    default_task.name = skill->name;
    default_task.parameters = skill->getParameters();
    default_task_dict[skill->name] = default_task;
    skill_generators[skill->name] =
      [](RobotCommandWrapperBase::SharedPtr & base) -> std::shared_ptr<skills::SkillInterface> {
      return std::make_shared<SkillType>(base);
    };
  }

  std::unordered_map<
    std::string, std::function<std::shared_ptr<skills::SkillInterface>(
                   RobotCommandWrapperBase::SharedPtr & base)>>
    skill_generators;

  std::unordered_map<std::string, Task> default_task_dict;

  rclcpp_action::Server<crane_msgs::action::SkillExecution>::SharedPtr skill_execution_server;

  std::shared_ptr<rclcpp_action::ServerGoalHandle<crane_msgs::action::SkillExecution>>
    skill_execution_goal_handle;

  std::shared_ptr<skills::SkillInterface> running_skill = nullptr;

  skills::Status skill_status;

  std::unordered_map<std::string, skills::ParameterType> parameters;
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__SIMPLE_AI_PLANNER_HPP_
