// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <crane_msgs/action/skill_execution.hpp>
#include <crane_msgs/msg/named_value_array.hpp>
#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <string>
#include <vector>

class SkillTesterCLI : public rclcpp::Node
{
public:
  using SkillExecutionAction = crane_msgs::action::SkillExecution;
  using SkillExecutionClient = rclcpp_action::Client<SkillExecutionAction>;

  SkillTesterCLI() : Node("skill_tester_cli")
  {
    client_ =
      rclcpp_action::create_client<SkillExecutionAction>(this, "/simple_ai/skill_execution");

    available_skills_ = {
      "Sleep",
      "Idle",
      "Kick",
      "Receive",
      "Goalie",
      "Attacker",
      "SubAttacker",
      "SingleBallPlacement",
      "GoalKick",
      "SimpleKickOff",
      "Marker",
      "EmplaceRobot",
      "Forward",
      "BallNearbyPositioner",
      "SecondThreatDefender",
      "FreekickSaver",
      "PenaltyKick",
      "Teleop"};
  }

  void run()
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    printWelcomeMessage();

    std::string input;
    while (rclcpp::ok() && std::getline(std::cin, input)) {
      if (input == "quit" || input == "exit" || input == "q") {
        break;
      }

      if (input == "help" || input == "h") {
        printHelp();
        continue;
      }

      if (input == "list" || input == "l") {
        listSkills();
        continue;
      }

      if (input.empty()) {
        continue;
      }

      processCommand(input);
      std::cout << "> ";
    }
  }

private:
  void printWelcomeMessage()
  {
    std::cout << "=== Crane Skill Tester CLI ===\n";
    std::cout << "Type 'help' for commands, 'list' for available skills, 'quit' to exit\n";
    std::cout << "> ";
  }

  void printHelp()
  {
    std::cout << "\nAvailable commands:\n";
    std::cout << "  help, h          - Show this help\n";
    std::cout << "  list, l          - List available skills\n";
    std::cout << "  run <skill> <robot_id> [params] - Execute skill\n";
    std::cout << "  robots <n>       - Set number of robots for multi-robot tests (1-16)\n";
    std::cout << "  quit, exit, q    - Exit the program\n";
    std::cout << "\nExample usage:\n";
    std::cout << "  run Kick 0 target_x:1.0 target_y:2.0 kick_power:5.0\n";
    std::cout << "  run Goalie 1\n";
    std::cout << "  robots 6\n\n";
  }

  void listSkills()
  {
    std::cout << "\nAvailable skills:\n";
    int count = 0;
    for (const auto & skill : available_skills_) {
      std::cout << "  " << skill;
      if (++count % 4 == 0)
        std::cout << "\n";
      else
        std::cout << "\t\t";
    }
    if (count % 4 != 0) std::cout << "\n";
    std::cout << "\n";
  }

  void processCommand(const std::string & input)
  {
    std::istringstream iss(input);
    std::string command;
    iss >> command;

    if (command == "run") {
      std::string skill_name;
      int robot_id;
      if (!(iss >> skill_name >> robot_id)) {
        std::cout << "Usage: run <skill_name> <robot_id> [parameters]\n";
        return;
      }

      std::map<std::string, std::string> params;
      std::string param;
      while (iss >> param) {
        size_t colon_pos = param.find(':');
        if (colon_pos != std::string::npos) {
          std::string key = param.substr(0, colon_pos);
          std::string value = param.substr(colon_pos + 1);
          params[key] = value;
        }
      }

      executeSkill(skill_name, robot_id, params);
    } else if (command == "robots") {
      int num_robots;
      if (iss >> num_robots) {
        if (num_robots >= 1 && num_robots <= 16) {
          num_robots_ = num_robots;
          std::cout << "Set number of robots to " << num_robots << "\n";
        } else {
          std::cout << "Number of robots must be between 1 and 16\n";
        }
      } else {
        std::cout << "Usage: robots <number>\n";
      }
    } else {
      std::cout << "Unknown command: " << command << "\n";
      std::cout << "Type 'help' for available commands\n";
    }
  }

  void executeSkill(
    const std::string & skill_name, int robot_id, const std::map<std::string, std::string> & params)
  {
    if (
      std::find(available_skills_.begin(), available_skills_.end(), skill_name) ==
      available_skills_.end()) {
      std::cout << "Unknown skill: " << skill_name << "\n";
      std::cout << "Type 'list' to see available skills\n";
      return;
    }

    if (robot_id < 0 || robot_id >= num_robots_) {
      std::cout << "Robot ID must be between 0 and " << (num_robots_ - 1) << "\n";
      return;
    }

    auto goal_msg = SkillExecutionAction::Goal();
    goal_msg.robot_id = static_cast<uint8_t>(robot_id);
    goal_msg.name = skill_name;

    for (const auto & [key, value] : params) {
      try {
        float float_val = std::stof(value);
        crane_msgs::msg::NamedFloat param_msg;
        param_msg.name = key;
        param_msg.value = float_val;
        goal_msg.parameter.float_values.push_back(param_msg);
      } catch (const std::exception &) {
        crane_msgs::msg::NamedString param_msg;
        param_msg.name = key;
        param_msg.value = value;
        goal_msg.parameter.string_values.push_back(param_msg);
      }
    }

    std::cout << "Executing skill '" << skill_name << "' on robot " << robot_id;
    if (!params.empty()) {
      std::cout << " with parameters:";
      for (const auto & [key, value] : params) {
        std::cout << " " << key << "=" << value;
      }
    }
    std::cout << "\n";

    auto send_goal_options = rclcpp_action::Client<SkillExecutionAction>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](const SkillExecutionClient::GoalHandle::SharedPtr & goal_handle) {
        if (!goal_handle) {
          std::cout << "Goal was rejected by server\n";
        } else {
          std::cout << "Goal accepted by server, executing...\n";
        }
      };

    send_goal_options.feedback_callback =
      [this](
        const SkillExecutionClient::GoalHandle::SharedPtr &,
        const std::shared_ptr<const SkillExecutionAction::Feedback> & feedback) {
        std::cout << "Feedback: " << feedback->message << "\n";
      };

    send_goal_options.result_callback =
      [this](const SkillExecutionClient::GoalHandle::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            std::cout << "Skill execution succeeded with result: " << result.result->result << "\n";
            break;
          case rclcpp_action::ResultCode::ABORTED:
            std::cout << "Skill execution was aborted\n";
            break;
          case rclcpp_action::ResultCode::CANCELED:
            std::cout << "Skill execution was canceled\n";
            break;
          default:
            std::cout << "Unknown result code\n";
            break;
        }
        std::cout << "> ";
      };

    client_->async_send_goal(goal_msg, send_goal_options);
  }

  SkillExecutionClient::SharedPtr client_;
  std::vector<std::string> available_skills_;
  int num_robots_ = 16;  // Default to 16 robots
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SkillTesterCLI>();

  std::thread cli_thread([&node]() { node->run(); });

  rclcpp::spin(node);

  cli_thread.join();
  rclcpp::shutdown();
  return 0;
}
