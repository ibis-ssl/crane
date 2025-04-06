// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/ping_status_array.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace crane
{
enum class RobotState {
  ACTIVE,    // アクティブで診断情報を発行すべき
  INACTIVE,  // 一時的に非アクティブ（フィールド外など）
};

struct RobotData
{
  uint8_t robot_id;

  RobotState state = RobotState::INACTIVE;

  rclcpp::Time last_update_time;

  std::unique_ptr<diagnostic_updater::Updater> updater;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr direct_publisher;

  explicit RobotData(const uint8_t & id) : robot_id(id) {}
};

class DiagnosticPublisherNode : public rclcpp::Node
{
public:
  DiagnosticPublisherNode() : Node("diagnostic_publisher_node")
  {
    declare_parameter("max_robot_id", 12);  // サポートする最大ロボットID
    int max_robot_id = get_parameter("max_robot_id").as_int();

    // 最大ロボット数分の構造体を事前に準備
    for (int i = 0; i <= max_robot_id; ++i) {
      // 初期状態では空のRobotDataを作成
      robots_data.emplace_back(std::make_shared<RobotData>(i));
      initializeRobotDiagnostics(i);
    }

    // TODO(HansRobo): 温度チェック

    ping_subscription = create_subscription<crane_msgs::msg::PingStatusArray>(
      "/ping", 10, [&](const crane_msgs::msg::PingStatusArray & msg) { latest_ping_msg = msg; });

    feedback_subscription = create_subscription<crane_msgs::msg::RobotFeedbackArray>(
      "/robot_feedback", 10,
      [&](const crane_msgs::msg::RobotFeedbackArray & msg) { latest_feedback_msg = msg; });

    commands_subscription = create_subscription<crane_msgs::msg::RobotCommands>(
      "/robot_commands", 10,
      [&](const crane_msgs::msg::RobotCommands & msg) { latest_commands_msg = msg; });

    world_model = std::make_unique<WorldModelWrapper>(*this);
    world_model->addCallback([&]() {
      auto available_robot_ids = world_model->ours.getAvailableRobotIds();
      for (int id = 0; id < robots_data.size(); ++id) {
        auto & data = robots_data.at(id);
        // 状態を更新
        data->state =
          ranges::contains(available_robot_ids, id) ? RobotState::ACTIVE : RobotState::INACTIVE;
      }
    });
    timer = this->create_wall_timer(std::chrono::seconds(1), [&]() {
      for (auto & robot_data : robots_data) {
        if (robot_data->updater) {
          robot_data->updater->force_update();
          robot_data->last_update_time = now();
        }
      }
    });
  }

private:
  WorldModelWrapper::UniquePtr world_model;

  std::vector<std::shared_ptr<RobotData>> robots_data;

  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Subscription<crane_msgs::msg::PingStatusArray>::SharedPtr ping_subscription;

  crane_msgs::msg::PingStatusArray latest_ping_msg;

  rclcpp::Subscription<crane_msgs::msg::RobotFeedbackArray>::SharedPtr feedback_subscription;

  crane_msgs::msg::RobotFeedbackArray latest_feedback_msg;

  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr commands_subscription;

  crane_msgs::msg::RobotCommands latest_commands_msg;

  void initializeRobotDiagnostics(const uint8_t robot_id)
  {
    auto & data = robots_data.at(robot_id);

    data->updater = std::make_unique<diagnostic_updater::Updater>(this);
    data->updater->setHardwareID("robot_" + std::to_string(robot_id));
    data->updater->add(
      "communication",
      [this, robot_id = data->robot_id](diagnostic_updater::DiagnosticStatusWrapper & stat) {
        auto & data = robots_data.at(robot_id);
        if (data->state != RobotState::ACTIVE) {
          // 非アクティブなロボットの場合はOKとして報告
          stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Robot inactive");
          return;
        }

        if (auto ping = ranges::find_if(
              latest_ping_msg.ping, [&](const auto msg) { return msg.robot_id == data->robot_id; });
            ping != latest_ping_msg.ping.end()) {
          if (ping->ping_ms > 50.0) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Ping time high");
          } else if (ping->ping_ms > 10.0) {
            stat.summary(
              diagnostic_msgs::msg::DiagnosticStatus::WARN, "Communication latency medium");
          } else {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Communication OK");
          }
          stat.add("ping_ms", ping->ping_ms);
        } else {
          stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No ping data received");
        }
      });

    data->updater->add(
      "battery",
      [this, robot_id = data->robot_id](diagnostic_updater::DiagnosticStatusWrapper & stat) {
        auto & data = robots_data.at(robot_id);
        if (data->state != RobotState::ACTIVE) {
          stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Robot inactive");
        } else {
          if (auto feedback = ranges::find_if(
                latest_feedback_msg.feedback,
                [&](const auto & msg) { return msg.robot_id == data->robot_id; });
              feedback != latest_feedback_msg.feedback.end()) {
            if (feedback->voltage[0] < 22.0) {
              stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Low battery voltage");
            } else if (feedback->voltage[0] < 23.0) {
              stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Battery voltage medium");
            } else {
              stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Battery voltage high");
            }
            stat.add("voltage", feedback->voltage[0]);
          } else {
            stat.summary(
              diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No robot feedback received");
          }
        }
      });

    data->updater->add(
      "robot_error",
      [this, robot_id = data->robot_id](diagnostic_updater::DiagnosticStatusWrapper & stat) {
        auto & data = robots_data.at(robot_id);
        if (data->state != RobotState::ACTIVE) {
          stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Robot inactive");
        } else {
          if (auto feedback = ranges::find_if(
                latest_feedback_msg.feedback,
                [&](const auto & msg) { return msg.robot_id == data->robot_id; });
              feedback != latest_feedback_msg.feedback.end()) {
            if (feedback->error_id != 0) {
              stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "An error occurred");
              stat.add("error_id", feedback->error_id);
              stat.add("error_info", feedback->error_info);
              stat.add("error_value", feedback->error_value);
            } else {
              stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No error");
            }
          } else {
            stat.summary(
              diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No robot feedback received");
          }
        }
      });

    data->updater->add(
      "movement",
      [this, robot_id = data->robot_id](diagnostic_updater::DiagnosticStatusWrapper & stat) {
        auto & data = robots_data.at(robot_id);
        if (auto info = world_model->getOurRobot(robot_id); info) {
          if (auto command = ranges::find_if(
                latest_commands_msg.robot_commands,
                [&](const auto & msg) { msg.robot_id == data->robot_id; });
              command != latest_commands_msg.robot_commands.end()) {
            Velocity command_velocity{command->current_velocity.x, command->current_velocity.y};
          }
          // 実装
        }
      });
    data->direct_publisher = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics/robot_" + std::to_string(robot_id), 10);
  }
};
}  // namespace crane

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<crane::DiagnosticPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
