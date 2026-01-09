// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_tactics/simple_ai_tactic.hpp>

// スキルのインクルード（.cppでのみ必要）
#include <crane_robot_skills/attacker.hpp>
#include <crane_robot_skills/emplace_robot.hpp>
#include <crane_robot_skills/goal_kick.hpp>
#include <crane_robot_skills/goalie.hpp>
#include <crane_robot_skills/idle.hpp>
#include <crane_robot_skills/kick.hpp>
#include <crane_robot_skills/marker.hpp>
#include <crane_robot_skills/receive.hpp>
#include <crane_robot_skills/simple_kickoff.hpp>
#include <crane_robot_skills/single_ball_placement.hpp>
#include <crane_robot_skills/sleep.hpp>
#include <crane_robot_skills/sub_attacker.hpp>
#include <crane_robot_skills/teleop.hpp>

namespace crane
{
// スキル登録用ヘルパーマクロ
#define REGISTER_SKILL(SkillType)                                                        \
  do {                                                                                   \
    auto dummy_wm = std::make_shared<crane::WorldModelWrapper>(*action_node);            \
    auto dummy_skill = std::make_shared<SkillType>(static_cast<uint8_t>(0), dummy_wm);   \
    registerSkill(                                                                       \
      dummy_skill->name, [](uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm) { \
        return std::make_shared<SkillType>(id, wm);                                      \
      });                                                                                \
  } while (0)

SimpleAITactic::SimpleAITactic(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
: TacticBase("SimpleAI", world_model), world_model(world_model)
{
  action_node = node.create_sub_node("simple_ai");
  {
    // スキルを登録
    REGISTER_SKILL(skills::Teleop);
    REGISTER_SKILL(skills::Attacker);
    REGISTER_SKILL(skills::Idle);
    REGISTER_SKILL(skills::Goalie);
    REGISTER_SKILL(skills::GoalKick);
    REGISTER_SKILL(skills::Kick);
    REGISTER_SKILL(skills::Sleep);
    REGISTER_SKILL(skills::Receive);
    REGISTER_SKILL(skills::SimpleKickOff);
    REGISTER_SKILL(skills::SubAttacker);
    REGISTER_SKILL(skills::Marker);
    REGISTER_SKILL(skills::SingleBallPlacement);
    REGISTER_SKILL(skills::EmplaceRobot);
  }

#undef REGISTER_SKILL

  using crane_msgs::action::SkillExecution;
  skill_execution_server = rclcpp_action::create_server<SkillExecution>(
    action_node->get_node_base_interface(), action_node->get_node_clock_interface(),
    action_node->get_node_logging_interface(), action_node->get_node_waitables_interface(),
    "/simple_ai/skill_execution",
    // ゴール（通常の指令）のコールバック
    [&](const rclcpp_action::GoalUUID, std::shared_ptr<const SkillExecution::Goal> goal)
      -> rclcpp_action::GoalResponse {
      RCLCPP_INFO(action_node->get_logger(), "Received goal: %s", goal->name.c_str());
      if (running_skill) {
        running_skill.reset();
      }
      if (auto skill_generator = skill_generators.find(goal->name);
          skill_generator != skill_generators.end()) {
        RCLCPP_INFO(
          action_node->get_logger(), "Start executing skill: %s for robot %d", goal->name.c_str(),
          static_cast<int>(goal->robot_id));
        robot_id = goal->robot_id;
        RCLCPP_DEBUG(
          action_node->get_logger(), "Skill pointer (before): %p",
          static_cast<void *>(running_skill.get()));
        running_skill = skill_generator->second(goal->name, goal->robot_id, this->world_model);
        RCLCPP_DEBUG(
          action_node->get_logger(), "Skill pointer (after): %p",
          static_cast<void *>(running_skill.get()));
        skill_status = skills::Status::RUNNING;
        parameters.clear();

        // Apply parameters to the skill
        for (auto e : goal->parameter.bool_values) {
          parameters[e.name] = e.value;
          running_skill->setParameter(e.name, e.value);
        }
        for (auto e : goal->parameter.float_values) {
          parameters[e.name] = static_cast<double>(e.value);
          running_skill->setParameter(e.name, static_cast<double>(e.value));
        }
        for (auto e : goal->parameter.int_values) {
          parameters[e.name] = static_cast<int>(e.value);
          running_skill->setParameter(e.name, static_cast<int>(e.value));
        }
        for (auto e : goal->parameter.string_values) {
          parameters[e.name] = e.value;
          running_skill->setParameter(e.name, e.value);
        }
        for (auto e : goal->parameter.position_values) {
          Point p(e.x, e.y);
          parameters[e.name] = p;
          running_skill->setParameter(e.name, p);
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      } else {
        RCLCPP_ERROR(action_node->get_logger(), "No skill found: %s", goal->name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
    },
    // キャンセルのコールバック
    [&](const std::shared_ptr<rclcpp_action::ServerGoalHandle<SkillExecution>> &)
      -> rclcpp_action::CancelResponse {
      RCLCPP_INFO(action_node->get_logger(), "Canceling goal");
      skill_execution_goal_handle.reset();
      if (running_skill) {
        running_skill.reset();
      }
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    // 実行関数（ログの転送）
    [this](
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<SkillExecution>> goal_handle) -> void {
      skill_execution_goal_handle = goal_handle;
      RCLCPP_INFO(action_node->get_logger(), "Accept goal callback");
    });

  action_sync_timer = action_node->create_wall_timer(std::chrono::milliseconds(200), [this]() {
    if (skill_execution_goal_handle && skill_execution_goal_handle->is_active() && running_skill) {
      if (skill_status == skills::Status::RUNNING) {
        auto feedback = std::make_shared<SkillExecution::Feedback>();
        // Use simple skill name instead of expensive print() method for performance
        feedback->message = "Executing skill: " + running_skill->name;
        skill_execution_goal_handle->publish_feedback(feedback);
      } else {
        auto result = std::make_shared<SkillExecution::Result>();
        result->result = static_cast<int>(skill_status);
        skill_execution_goal_handle->succeed(result);
      }
    }
  });
}

SimpleAITactic::~SimpleAITactic()
{
  if (skill_execution_goal_handle && skill_execution_goal_handle->is_active()) {
    skill_execution_goal_handle->abort(
      std::make_shared<crane_msgs::action::SkillExecution::Result>());
  }
}

std::pair<TacticBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
SimpleAITactic::calculatePositionCommand(const std::vector<RobotIdentifier> &)
{
  std::vector<crane_msgs::msg::PositionCommand> robot_commands;
  if (running_skill) {
    skill_status = running_skill->run(parameters);
    robot_commands.push_back(running_skill->getRobotCommand());
  }
  return {TacticBase::Status::RUNNING, robot_commands};
}

auto SimpleAITactic::getSelectedRobots(
  [[maybe_unused]] uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> &) -> std::vector<uint8_t>
{
  // For SimpleAI tactic, if we have a running skill, always return the robot_id
  // even if it's not in selectable_robots. This ensures WebUI skill execution works.
  if (running_skill) {
    return {robot_id};
  }

  // if robot_id is in selectable_robots, add it to selected robots.
  if (
    std::find(selectable_robots.begin(), selectable_robots.end(), robot_id) !=
    selectable_robots.end()) {
    return {robot_id};
  } else {
    return {};
  }
}

void SimpleAITactic::registerSkill(
  const std::string & name, std::function<std::shared_ptr<skills::SkillInterface>(
                              uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)>
                              generator)
{
  skill_generators[name] =
    [generator](
      [[maybe_unused]] const std::string & skill_name, uint8_t id,
      const std::shared_ptr<WorldModelWrapper> & wm) -> std::shared_ptr<skills::SkillInterface> {
    return generator(id, wm);
  };
}

}  // namespace crane
