// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cmath>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/ping_status_array.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <cstring>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace crane
{
// POWERエラー定義
enum PowerErrorCode {
  POWER_NONE = 0,
  POWER_UNDER_VOLTAGE = 0x0001,
  POWER_OVER_VOLTAGE = 0x0002,
  POWER_OVER_CURRENT = 0x0004,
  POWER_SHORT_CIRCUIT = 0x0008,
  POWER_CHARGE_TIME = 0x0010,
  POWER_CHARGE_POWER = 0x0020,
  POWER_DISCHARGE = 0x0040,
  POWER_PARAMETER = 0x0080,
  POWER_COMMAND = 0x0100,
  POWER_NO_CAP = 0x0200,
  POWER_DISCHARGE_FAIL = 0x0400,
  POWER_GD_POWER_FAIL = 0x0800,
  POWER_COIL_OVER_HEAT = 0x1000,
  POWER_FET_OVER_HEAT = 0x2000
};

// BLDCエラー定義
enum BldcErrorCode {
  BLDC_NONE = 0,
  BLDC_UNDER_VOLTAGE = 0x0001,
  BLDC_OVER_CURRENT = 0x0002,
  BLDC_MOTOR_OVER_HEAT = 0x0004,
  BLDC_OVER_LOAD = 0x0008,
  BLDC_ENC_ERROR = 0x0010,
  BLDC_OVER_VOLTAGE = 0x0020,
  BLDC_FET_OVER_HEAT = 0x0040
};

// エラー情報をテキストに変換する関数
std::string convertErrorDataToStr(uint16_t id, uint16_t info)
{
  std::string result;

  if (id == 100) {  // POWERエラー
    result = "POWER : ";
    switch (info) {
      case POWER_NONE:
        result += "no error";
        break;
      case POWER_UNDER_VOLTAGE:
        result += "UNDER_VOLTAGE";
        break;
      case POWER_OVER_VOLTAGE:
        result += "OVER_VOLTAGE";
        break;
      case POWER_OVER_CURRENT:
        result += "OVER_CURRENT";
        break;
      case POWER_SHORT_CIRCUIT:
        result += "SHORT_CIRCUIT";
        break;
      case POWER_CHARGE_TIME:
        result += "CHARGE_TIME";
        break;
      case POWER_CHARGE_POWER:
        result += "CHARGE_POWER";
        break;
      case POWER_DISCHARGE:
        result += "DISCHARGE";
        break;
      case POWER_PARAMETER:
        result += "PARAMETER";
        break;
      case POWER_COMMAND:
        result += "COMMAND";
        break;
      case POWER_NO_CAP:
        result += "NO_CAP";
        break;
      case POWER_DISCHARGE_FAIL:
        result += "DISCHARGE_FAIL";
        break;
      case POWER_GD_POWER_FAIL:
        result += "GD_POWER_FAIL";
        break;
      case POWER_COIL_OVER_HEAT:
        result += "COIL_OVER_HEAT";
        break;
      case POWER_FET_OVER_HEAT:
        result += "FET_OVER_HEAT";
        break;
      default:
        result += "unknown info " + std::to_string(info);
        break;
    }
  } else {  // BLDCエラー
    if (id == 0) {
      result = "BLDC-RF : ";
    } else if (id == 1) {
      result = "BLDC-RB : ";
    } else if (id == 2) {
      result = "BLDC-LB : ";
    } else if (id == 3) {
      result = "BLDC-LF : ";
    } else {
      return "BLDC unknown id : " + std::to_string(id) + " info : " + std::to_string(info);
    }

    switch (info) {
      case BLDC_NONE:
        result += "no error";
        break;
      case BLDC_UNDER_VOLTAGE:
        result += "UNDER_VOLTAGE";
        break;
      case BLDC_OVER_CURRENT:
        result += "OVER_CURRENT";
        break;
      case BLDC_MOTOR_OVER_HEAT:
        result += "MOTOR_OVER_HEAT";
        break;
      case BLDC_OVER_LOAD:
        result += "OVER_LOAD";
        break;
      case BLDC_ENC_ERROR:
        result += "ENC_ERROR";
        break;
      case BLDC_OVER_VOLTAGE:
        result += "OVER_VOLTAGE";
        break;
      case BLDC_FET_OVER_HEAT:
        result += "FET_OVER_HEAT";
        break;
      default:
        result += "unknown info " + std::to_string(info);
        break;
    }
  }

  return result;
}

// temperature配列のインデックスから名前への変換
std::string getTemperatureLabel(int index)
{
  switch (index) {
    case 0:
      return "Motor 1";
    case 1:
      return "Motor 2";
    case 2:
      return "Motor 3";
    case 3:
      return "Motor 4";
    case 4:
      return "FET";
    case 5:
      return "Coil 1";
    case 6:
      return "Coil 2";
    default:
      return "Unknown (" + std::to_string(index) + ")";
  }
}

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

    // ロボットエラー表示の改良（より詳細なエラー情報を表示）
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
            if (feedback->error_id != 0 || feedback->error_info != 0) {
              // エラー情報を人間が読める形式に変換して表示
              std::string error_str =
                convertErrorDataToStr(feedback->error_id, feedback->error_info);
              stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error_str);

              stat.add("error_id", feedback->error_id);
              stat.add("error_info", feedback->error_info);
              stat.add("error_value", feedback->error_value);
              stat.add("error_description", error_str);
            } else {
              stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No error");
            }
          } else {
            stat.summary(
              diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No robot feedback received");
          }
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
