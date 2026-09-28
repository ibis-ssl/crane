// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <chrono>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/ping_status_array.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <crane_robot_receiver/diagnostic_publisher.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace crane
{
namespace
{
using diagnostic_msgs::msg::DiagnosticStatus;

constexpr uint8_t ROBOT_ID = 3;
constexpr uint8_t OTHER_ROBOT_ID = 5;

struct Detection
{
  bool vision = false;
  bool feedback = false;
  bool tracker = false;
};

auto makeWorldModel(const Detection & detection) -> crane_msgs::msg::WorldModel
{
  crane_msgs::msg::WorldModel msg;
  msg.field_info.x = 12.0;
  msg.field_info.y = 9.0;
  msg.penalty_area_size.x = 1.8;
  msg.penalty_area_size.y = 3.6;
  msg.goal_size.x = 0.18;
  msg.goal_size.y = 1.8;
  crane_msgs::msg::RobotInfo robot;
  robot.id = ROBOT_ID;
  robot.available_vision = detection.vision;
  robot.available_feedback = detection.feedback;
  robot.available_tracker = detection.tracker;
  msg.robot_info_ours.push_back(robot);
  return msg;
}

auto makeFeedback(uint8_t robot_id) -> crane_msgs::msg::RobotFeedback
{
  crane_msgs::msg::RobotFeedback feedback;
  feedback.robot_id = robot_id;
  feedback.feedback_age_ms = 10.0;
  feedback.packet_frequency_hz = 100.0;
  feedback.voltage = {24.0F};
  return feedback;
}

auto makePing(uint8_t robot_id) -> crane_msgs::msg::PingStatus
{
  crane_msgs::msg::PingStatus ping;
  ping.robot_id = robot_id;
  ping.ping_ms = 3.0;
  return ping;
}

// 診断ごとの出力（名前の末尾・レベル・メッセージ・key/value）
struct Status
{
  std::string name;
  int level;
  std::string message;
  std::vector<std::pair<std::string, std::string>> values;
};

class DiagnosticPublisherTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_diagnostic_publisher");
    world_model = std::make_unique<WorldModelWrapper>(*node, false);
    subscription = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10, [this](const diagnostic_msgs::msg::DiagnosticArray & msg) {
        // 同じ DDS ドメインの他ノードの診断は拾わない
        if (!msg.status.empty() && msg.status.front().hardware_id == "robot_03") {
          received = msg;
        }
      });
  }

  // RobotData を作って 1 回だけ診断を走らせ、publish された 3 診断を返す
  auto run(bool sim_mode, const Detection & detection) -> std::vector<Status>
  {
    world_model->update(makeWorldModel(detection));
    robot_data = std::make_unique<RobotData>(ROBOT_ID);
    robot_data->initializeDiagnostics(
      node.get(), world_model.get(), sim_mode, &ping_msg, &feedback_msg);
    // 事前に全種類のエラーを入れておき、OK 側の分岐で消えることも確かめる
    for (const auto & type : {"communication", "battery", "robot_error"}) {
      robot_data->updateErrorMap(type, "stale", DiagnosticStatus::ERROR, node->now());
    }

    std::vector<Status> result;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      received.reset();
      robot_data->updater->force_update();
      for (int i = 0; i < 50 && !received; ++i) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      if (received && received->status.size() == 3) {
        break;
      }
    }
    if (!received) {
      ADD_FAILURE() << "no /diagnostics received";
      return result;
    }
    for (const auto & status : received->status) {
      Status s;
      const auto pos = status.name.find("robot_03/");
      s.name = pos == std::string::npos ? status.name : status.name.substr(pos);
      s.level = status.level;
      s.message = status.message;
      for (const auto & kv : status.values) {
        s.values.emplace_back(kv.key, kv.value);
      }
      result.push_back(s);
    }
    return result;
  }

  auto errorTypes() const -> std::vector<std::pair<std::string, int>>
  {
    std::vector<std::pair<std::string, int>> types;
    for (const auto & [type, info] : robot_data->error_map) {
      if (info.message != "stale") {
        types.emplace_back(type, info.level);
      } else {
        types.emplace_back(type + "(stale)", info.level);
      }
    }
    return types;
  }

  static auto summaries(const std::vector<Status> & statuses)
    -> std::vector<std::tuple<std::string, int, std::string>>
  {
    std::vector<std::tuple<std::string, int, std::string>> result;
    for (const auto & s : statuses) {
      result.emplace_back(s.name, s.level, s.message);
    }
    return result;
  }

  rclcpp::Node::SharedPtr node;
  std::unique_ptr<WorldModelWrapper> world_model;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscription;
  std::optional<diagnostic_msgs::msg::DiagnosticArray> received;
  crane_msgs::msg::PingStatusArray ping_msg;
  crane_msgs::msg::RobotFeedbackArray feedback_msg;
  std::unique_ptr<RobotData> robot_data;
};

using Summaries = std::vector<std::tuple<std::string, int, std::string>>;
using ErrorTypes = std::vector<std::pair<std::string, int>>;

TEST_F(DiagnosticPublisherTest, NotDetectedIsOkAndClearsErrors)
{
  const auto statuses = run(false, Detection{});
  EXPECT_EQ(
    summaries(statuses), (Summaries{
                           {"robot_03/communication", DiagnosticStatus::OK, "Robot not detected"},
                           {"robot_03/battery", DiagnosticStatus::OK, "Robot not detected"},
                           {"robot_03/robot_error", DiagnosticStatus::OK, "Robot not detected"},
                         }));
  EXPECT_EQ(errorTypes(), ErrorTypes{});
}

// フィードバックだけで検出されているとき、communication は未検出扱い、
// battery・robot_error は検出扱いになる（require_feedback の違い）
TEST_F(DiagnosticPublisherTest, FeedbackOnlyDetectionGatesCommunicationOnly)
{
  const auto statuses = run(true, Detection{.feedback = true});
  EXPECT_EQ(
    summaries(statuses),
    (Summaries{
      {"robot_03/communication", DiagnosticStatus::OK, "Robot not detected"},
      {"robot_03/battery", DiagnosticStatus::OK, "Simulation mode (no battery data)"},
      {"robot_03/robot_error", DiagnosticStatus::OK, "Simulation mode (no robot error data)"},
    }));
  EXPECT_EQ(errorTypes(), ErrorTypes{});
}

TEST_F(DiagnosticPublisherTest, SimModeWithoutTelemetry)
{
  // 他のロボットのフィードバック・ping は自分のものとして扱わない
  feedback_msg.feedback.push_back(makeFeedback(OTHER_ROBOT_ID));
  ping_msg.ping.push_back(makePing(OTHER_ROBOT_ID));
  const auto statuses = run(true, Detection{.vision = true});
  EXPECT_EQ(
    summaries(statuses),
    (Summaries{
      {"robot_03/communication", DiagnosticStatus::OK, "Simulation mode (no ping data)"},
      {"robot_03/battery", DiagnosticStatus::OK, "Simulation mode (no battery data)"},
      {"robot_03/robot_error", DiagnosticStatus::OK, "Simulation mode (no robot error data)"},
    }));
  EXPECT_EQ(errorTypes(), ErrorTypes{});
}

TEST_F(DiagnosticPublisherTest, RealModeWithoutTelemetry)
{
  const auto statuses = run(false, Detection{.tracker = true});
  EXPECT_EQ(
    summaries(statuses),
    (Summaries{
      {"robot_03/communication", DiagnosticStatus::ERROR, "No robot telemetry received"},
      {"robot_03/battery", DiagnosticStatus::WARN, "No robot feedback received"},
      {"robot_03/robot_error", DiagnosticStatus::WARN, "No robot feedback received"},
    }));
  EXPECT_EQ(errorTypes(), (ErrorTypes{{"communication", DiagnosticStatus::ERROR}}));
}

TEST_F(DiagnosticPublisherTest, RealModePingWithoutFeedback)
{
  ping_msg.ping.push_back(makePing(ROBOT_ID));
  const auto statuses = run(false, Detection{.vision = true});
  EXPECT_EQ(
    summaries(statuses),
    (Summaries{
      {"robot_03/communication", DiagnosticStatus::WARN, "Ping available, robot feedback missing"},
      {"robot_03/battery", DiagnosticStatus::WARN, "No robot feedback received"},
      {"robot_03/robot_error", DiagnosticStatus::WARN, "No robot feedback received"},
    }));
  ASSERT_EQ(statuses.size(), 3u);
  EXPECT_EQ(
    statuses[0].values, (std::vector<std::pair<std::string, std::string>>{{"ping_ms", "3"}}));
  EXPECT_EQ(errorTypes(), (ErrorTypes{{"communication", DiagnosticStatus::WARN}}));
}

TEST_F(DiagnosticPublisherTest, HealthyFeedback)
{
  feedback_msg.feedback.push_back(makeFeedback(OTHER_ROBOT_ID));
  feedback_msg.feedback.push_back(makeFeedback(ROBOT_ID));
  ping_msg.ping.push_back(makePing(ROBOT_ID));
  const auto statuses = run(false, Detection{.vision = true});
  EXPECT_EQ(
    summaries(statuses),
    (Summaries{
      {"robot_03/communication", DiagnosticStatus::OK, "Robot feedback healthy"},
      {"robot_03/battery", DiagnosticStatus::OK, "Battery voltage high"},
      {"robot_03/robot_error", DiagnosticStatus::OK, "No error"},
    }));
  ASSERT_EQ(statuses.size(), 3u);
  EXPECT_EQ(statuses[0].values.size(), 8u);
  EXPECT_EQ(
    statuses[1].values, (std::vector<std::pair<std::string, std::string>>{{"voltage", "24"}}));
  EXPECT_EQ(statuses[2].values.size(), 0u);
  EXPECT_EQ(errorTypes(), ErrorTypes{});
}

TEST_F(DiagnosticPublisherTest, DegradedFeedbackWithErrors)
{
  auto feedback = makeFeedback(ROBOT_ID);
  feedback.feedback_age_ms = 300.0;
  feedback.voltage = {21.0F};
  feedback.error_id = 100;
  feedback.error_info = 0x0001;
  feedback_msg.feedback.push_back(feedback);
  const auto statuses = run(true, Detection{.vision = true});
  ASSERT_EQ(statuses.size(), 3u);
  EXPECT_EQ(statuses[0].level, DiagnosticStatus::ERROR);
  EXPECT_EQ(statuses[0].message, "Robot feedback degraded");
  EXPECT_EQ(statuses[1].level, DiagnosticStatus::ERROR);
  EXPECT_EQ(statuses[1].message, "Low battery voltage");
  EXPECT_EQ(statuses[2].level, DiagnosticStatus::ERROR);
  EXPECT_EQ(statuses[2].message, utils::convertErrorDataToStr(100, 0x0001));
  EXPECT_EQ(statuses[2].values.size(), 4u);
  EXPECT_EQ(
    errorTypes(), (ErrorTypes{
                    {"battery", DiagnosticStatus::ERROR},
                    {"communication", DiagnosticStatus::ERROR},
                    {"robot_error", DiagnosticStatus::ERROR},
                  }));
}
}  // namespace
}  // namespace crane

auto main(int argc, char ** argv) -> int
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
