// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// ibis_sender_node が /robot_commands を送信前に整える処理（kick/dribble の clamp、
// vision 経過時間の飽和、theta の保持、no_movement）を、トピックの入出力だけで固定する。
// ノードのクラスは実行ファイルのソースにしか無いので、main を改名して取り込む。
#define main ibis_sender_node_main
#include "../src/ibis_sender_node.cpp"  // NOLINT(build/include)
#undef main

#include <gtest/gtest.h>

#include <chrono>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <vector>

using crane_msgs::msg::RobotCommand;
using crane_msgs::msg::RobotCommands;
using crane_msgs::msg::WorldModel;
using std::chrono_literals::operator""ms;

class SenderCallbackTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void startSender(bool no_movement)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"target_address", "127.0.0.1"},
      {"target_port", 50999},
      {"position_control.config_port", 50998},
      {"no_movement", no_movement},
      {"kick_power_limit_straight", 0.5},
      {"kick_power_limit_chip", 0.8},
      {"latency_ms", 12.5},
    });
    sender_ = std::make_shared<crane::IbisSenderNode>(options);
    probe_ = std::make_shared<rclcpp::Node>("sender_callback_probe");
    world_model_pub_ = probe_->create_publisher<WorldModel>("/world_model", 10);
    commands_pub_ = probe_->create_publisher<RobotCommands>("/robot_commands", 10);
    sent_sub_ = probe_->create_subscription<RobotCommands>(
      "/sent_robot_commands", 10, [this](const RobotCommands & msg) { sent_.push_back(msg); });
    executor_.add_node(sender_);
    executor_.add_node(probe_);

    spinUntil([this] {
      return world_model_pub_->get_subscription_count() > 0 &&
             commands_pub_->get_subscription_count() > 0 && sent_sub_->get_publisher_count() > 0;
    });
  }

  void TearDown() override
  {
    if (sender_) {
      executor_.remove_node(sender_);
      executor_.remove_node(probe_);
    }
  }

  template <class Pred>
  bool spinUntil(Pred pred, std::chrono::milliseconds timeout = 3000ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (pred()) return true;
      executor_.spin_some(10ms);
    }
    return pred();
  }

  // id 1 を vision_age 前に検出したワールドモデルを送る
  void publishWorldModel(std::chrono::milliseconds vision_age)
  {
    WorldModel wm;
    const auto now = rclcpp::Clock(RCL_ROS_TIME).now();
    wm.header.stamp = now;
    crane_msgs::msg::RobotInfo robot;
    robot.id = 1;
    robot.available_vision = true;
    robot.available_feedback = true;
    robot.has_error = false;
    robot.vision.stamp = now - rclcpp::Duration(vision_age);
    wm.robot_info_ours.push_back(robot);
    world_model_pub_->publish(wm);
    // 受信を待つ手段が無いので、配送に十分な時間だけ回す
    spinUntil([] { return false; }, 300ms);
  }

  std::optional<RobotCommands> send(const RobotCommands & msg)
  {
    const auto before = sent_.size();
    commands_pub_->publish(msg);
    if (!spinUntil([&] { return sent_.size() > before; })) {
      return std::nullopt;
    }
    return sent_.back();
  }

  static RobotCommand command(uint8_t id)
  {
    RobotCommand cmd;
    cmd.robot_id = id;
    cmd.control_mode = RobotCommand::POLAR_VELOCITY_TARGET_MODE;
    return cmd;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<crane::IbisSenderNode> sender_;
  rclcpp::Node::SharedPtr probe_;
  rclcpp::Publisher<WorldModel>::SharedPtr world_model_pub_;
  rclcpp::Publisher<RobotCommands>::SharedPtr commands_pub_;
  rclcpp::Subscription<RobotCommands>::SharedPtr sent_sub_;
  std::vector<RobotCommands> sent_;
};

TEST_F(SenderCallbackTest, DoesNotSendBeforeWorldModel)
{
  startSender(false);
  RobotCommands msg;
  msg.robot_commands.push_back(command(1));
  commands_pub_->publish(msg);
  spinUntil([] { return false; }, 500ms);
  EXPECT_TRUE(sent_.empty());
}

TEST_F(SenderCallbackTest, ClampsKickAndDribbleAndFillsLatency)
{
  startSender(false);
  publishWorldModel(0ms);

  RobotCommands msg;
  auto straight = command(1);
  straight.kick_power = 0.9f;
  straight.dribble_power = 1.5f;
  msg.robot_commands.push_back(straight);
  auto chip = command(1);
  chip.chip_enable = true;
  chip.kick_power = 0.9f;
  chip.dribble_power = -0.2f;
  msg.robot_commands.push_back(chip);
  auto negative = command(1);
  negative.kick_power = -0.3f;
  msg.robot_commands.push_back(negative);

  const auto sent = send(msg);
  ASSERT_TRUE(sent.has_value());
  ASSERT_EQ(sent->robot_commands.size(), 3u);
  EXPECT_FLOAT_EQ(sent->robot_commands[0].kick_power, 0.5f);
  EXPECT_FLOAT_EQ(sent->robot_commands[0].dribble_power, 1.0f);
  EXPECT_FLOAT_EQ(sent->robot_commands[1].kick_power, 0.8f);
  EXPECT_FLOAT_EQ(sent->robot_commands[1].dribble_power, 0.0f);
  EXPECT_FLOAT_EQ(sent->robot_commands[2].kick_power, 0.0f);
  for (const auto & cmd : sent->robot_commands) {
    EXPECT_FLOAT_EQ(cmd.latency_ms, 12.5f);
    // POLAR モードで速度指令が空なら 0 の要素を 1 つ足す
    ASSERT_EQ(cmd.polar_velocity_target_mode.size(), 1u);
    EXPECT_FLOAT_EQ(cmd.polar_velocity_target_mode.front().target_velocity_r, 0.0f);
  }
}

TEST_F(SenderCallbackTest, ElapsedVisionTimeIsMeasuredAndSaturated)
{
  startSender(false);

  publishWorldModel(2000ms);
  RobotCommands msg;
  msg.robot_commands.push_back(command(1));
  // ワールドモデルに無い id（範囲外）は例外経路で最大値にする
  msg.robot_commands.push_back(command(25));
  auto sent = send(msg);
  ASSERT_TRUE(sent.has_value());
  EXPECT_NEAR(sent->robot_commands[0].elapsed_time_ms_since_last_vision, 2000, 500);
  EXPECT_EQ(sent->robot_commands[1].elapsed_time_ms_since_last_vision, 65535);

  // 65535ms を超えたら巻き戻らずに最大値で止まる
  publishWorldModel(100000ms);
  msg.robot_commands.resize(1);
  sent = send(msg);
  ASSERT_TRUE(sent.has_value());
  EXPECT_EQ(sent->robot_commands[0].elapsed_time_ms_since_last_vision, 65535);
}

TEST_F(SenderCallbackTest, HoldsPreviousThetaWithinTolerance)
{
  startSender(false);
  publishWorldModel(0ms);

  auto cmd = command(1);
  cmd.local_planner_config.theta_tolerance = 0.1f;
  RobotCommands msg;

  cmd.target_theta = 1.0f;
  msg.robot_commands = {cmd};
  auto sent = send(msg);
  ASSERT_TRUE(sent.has_value());
  EXPECT_FLOAT_EQ(sent->robot_commands[0].target_theta, 1.0f);

  cmd.target_theta = 1.05f;
  msg.robot_commands = {cmd};
  sent = send(msg);
  ASSERT_TRUE(sent.has_value());
  EXPECT_FLOAT_EQ(sent->robot_commands[0].target_theta, 1.0f);

  cmd.target_theta = 1.5f;
  msg.robot_commands = {cmd};
  sent = send(msg);
  ASSERT_TRUE(sent.has_value());
  EXPECT_FLOAT_EQ(sent->robot_commands[0].target_theta, 1.5f);
}

TEST_F(SenderCallbackTest, NoMovementStopsEveryRobot)
{
  startSender(true);
  publishWorldModel(0ms);

  auto cmd = command(1);
  cmd.control_mode = RobotCommand::POSITION_TARGET_MODE;
  cmd.omega_limit = 5.0f;
  cmd.chip_enable = true;
  cmd.kick_power = 0.5f;
  cmd.dribble_power = 0.5f;
  RobotCommands msg;
  msg.robot_commands = {cmd};

  const auto sent = send(msg);
  ASSERT_TRUE(sent.has_value());
  const auto & out = sent->robot_commands[0];
  EXPECT_EQ(out.control_mode, RobotCommand::POLAR_VELOCITY_TARGET_MODE);
  ASSERT_EQ(out.polar_velocity_target_mode.size(), 1u);
  EXPECT_FLOAT_EQ(out.polar_velocity_target_mode.front().target_velocity_r, 0.0f);
  EXPECT_FLOAT_EQ(out.polar_velocity_target_mode.front().target_velocity_theta, 0.0f);
  EXPECT_FLOAT_EQ(out.omega_limit, 0.0f);
  EXPECT_FALSE(out.chip_enable);
  EXPECT_FLOAT_EQ(out.kick_power, 0.0f);
  EXPECT_FLOAT_EQ(out.dribble_power, 0.0f);
}
