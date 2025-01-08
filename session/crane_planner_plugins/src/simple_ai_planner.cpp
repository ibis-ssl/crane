// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/simple_ai_planner.hpp>

namespace crane
{
SimpleAIPlanner::SimpleAIPlanner(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
: PlannerBase("SimpleAI", world_model), Node("SimpleAI"), world_model(world_model)
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
          std::cout << "Start executing skill: " << goal->name << " for robot "
                    << static_cast<int>(goal->robot_id) << std::endl;
          auto command_base = std::make_shared<RobotCommandWrapperBase>(
            goal->name, goal->robot_id, this->world_model);
          robot_id = goal->robot_id;
          std::cout << "Skill: " << std::hex << running_skill.get() << std::endl;
          running_skill = skill_generator->second(command_base);
          std::cout << "Skill: " << std::hex << running_skill.get() << std::endl;
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
    [this](
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<SkillExecution>> goal_handle) -> void {
      skill_execution_goal_handle = goal_handle;
      std::cout << "Executing goal" << std::endl;
      while (running_skill && skill_status == skills::Status::RUNNING) {
        std::cout << "Skill status: " << static_cast<int>(skill_status) << std::endl;
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<SkillExecution::Feedback>();
        std::stringstream feedback_ss;
        running_skill->print(feedback_ss);
        feedback->message = feedback_ss.str();
        goal_handle->publish_feedback(feedback);
        rclcpp::sleep_for(std::chrono::milliseconds(100));  // 100ms待機
      }
      std::cout << "Goal succeeded: " << std::endl;
      auto result = std::make_shared<SkillExecution::Result>();
      goal_handle->succeed(result);
    });
}
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
SimpleAIPlanner::calculateRobotCommand(
  const std::vector<RobotIdentifier> & robots, PlannerContext & context)
{
  rclcpp::spin_some(this->get_node_base_interface());
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  if (running_skill) {
    std::cout << "Running skill: " << running_skill->name << std::endl;
    skill_status = running_skill->run();
    robot_commands.push_back(running_skill->getRobotCommand());
  } else {
    std::cout << "No skill running." << std::endl;
  }

  return {PlannerBase::Status::RUNNING, robot_commands};
}

auto SimpleAIPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
  -> std::vector<uint8_t>
{
  // if robot_id is in selectable_robots, add it to selected robots.
  if (
    std::find(selectable_robots.begin(), selectable_robots.end(), robot_id) !=
    selectable_robots.end()) {
    return {robot_id};
  } else {
    return {};
  }
}
}  // namespace crane
