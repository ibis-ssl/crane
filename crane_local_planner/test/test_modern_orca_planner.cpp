// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "crane_local_planner/modern_orca_planner.hpp"
#include "crane_local_planner/rvo2_planner.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"

namespace crane
{

class ModernORCAPlannerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS if not already done
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Create separate nodes to avoid parameter conflicts
    test_node_ = std::make_shared<rclcpp::Node>("test_node_" + std::to_string(test_counter_++));
    modern_node_ =
      std::make_shared<rclcpp::Node>("modern_planner_node_" + std::to_string(test_counter_));
    rvo2_node_ =
      std::make_shared<rclcpp::Node>("rvo2_planner_node_" + std::to_string(test_counter_));

    // Create world model
    world_model_ = std::make_shared<crane::WorldModelWrapper>(*test_node_, false);

    // Create planners with separate node references
    modern_planner_ = std::make_unique<ModernORCAPlanner>(*modern_node_);
    rvo2_planner_ = std::make_unique<RVO2Planner>(*rvo2_node_);

    // Set up basic world model
    setupBasicWorldModel();
  }

  void TearDown() override
  {
    modern_planner_.reset();
    rvo2_planner_.reset();
    world_model_.reset();
    test_node_.reset();
    modern_node_.reset();
    rvo2_node_.reset();
  }

  void setupBasicWorldModel()
  {
    // Set up field size and basic world parameters
    // This would normally be initialized from actual SSL vision data
  }

  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::Node> modern_node_;
  std::shared_ptr<rclcpp::Node> rvo2_node_;
  std::shared_ptr<crane::WorldModelWrapper> world_model_;
  std::unique_ptr<ModernORCAPlanner> modern_planner_;
  std::unique_ptr<RVO2Planner> rvo2_planner_;

  static int test_counter_;
};

// Define static member
int ModernORCAPlannerTest::test_counter_ = 0;

TEST_F(ModernORCAPlannerTest, BasicInitialization)
{
  // Test that planners can be initialized and have world model
  EXPECT_NE(modern_planner_->world_model, nullptr);
  EXPECT_NE(rvo2_planner_->world_model, nullptr);

  // Test that planners are properly constructed
  EXPECT_NE(modern_planner_, nullptr);
  EXPECT_NE(rvo2_planner_, nullptr);
}

TEST_F(ModernORCAPlannerTest, SingleRobotPositionControl)
{
  // Planners already have world model initialized

  // Create a simple position target command
  crane_msgs::msg::RobotCommands commands;
  crane_msgs::msg::RobotCommand command;
  command.robot_id = 0;
  command.control_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
  command.current_pose.x = 0.0;
  command.current_pose.y = 0.0;
  command.current_velocity.x = 0.0;
  command.current_velocity.y = 0.0;

  crane_msgs::msg::PositionTargetMode pos_target;
  pos_target.target_x = 1.0;
  pos_target.target_y = 1.0;
  pos_target.position_tolerance = 0.03;
  command.position_target_mode.push_back(pos_target);

  commands.robot_commands.push_back(command);

  // Test that both planners produce valid outputs
  EXPECT_NO_THROW({
    auto modern_result = modern_planner_->calculateRobotCommand(commands, 0.0);
    EXPECT_EQ(modern_result.robot_commands.size(), 1);
    EXPECT_EQ(modern_result.robot_commands[0].robot_id, 0);
  });

  EXPECT_NO_THROW({
    auto rvo2_result = rvo2_planner_->calculateRobotCommand(commands, 0.0);
    EXPECT_EQ(rvo2_result.robot_commands.size(), 1);
    EXPECT_EQ(rvo2_result.robot_commands[0].robot_id, 0);
  });
}

TEST_F(ModernORCAPlannerTest, MultiRobotCollisionAvoidance)
{
  // Planners already have world model initialized

  // Create multi-robot scenario
  crane_msgs::msg::RobotCommands commands;

  // Robot 0: moving right
  crane_msgs::msg::RobotCommand command1;
  command1.robot_id = 0;
  command1.control_mode = crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE;
  command1.current_pose.x = -1.0;
  command1.current_pose.y = 0.0;

  crane_msgs::msg::SimpleVelocityTargetMode vel_target1;
  vel_target1.target_vx = 1.0;
  vel_target1.target_vy = 0.0;
  command1.simple_velocity_target_mode.push_back(vel_target1);

  // Robot 1: moving left
  crane_msgs::msg::RobotCommand command2;
  command2.robot_id = 1;
  command2.control_mode = crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE;
  command2.current_pose.x = 1.0;
  command2.current_pose.y = 0.0;

  crane_msgs::msg::SimpleVelocityTargetMode vel_target2;
  vel_target2.target_vx = -1.0;
  vel_target2.target_vy = 0.0;
  command2.simple_velocity_target_mode.push_back(vel_target2);

  commands.robot_commands.push_back(command1);
  commands.robot_commands.push_back(command2);

  // Test collision avoidance behavior
  auto modern_result = modern_planner_->calculateRobotCommand(commands, 0.0);
  auto rvo2_result = rvo2_planner_->calculateRobotCommand(commands, 0.0);

  EXPECT_EQ(modern_result.robot_commands.size(), 2);
  EXPECT_EQ(rvo2_result.robot_commands.size(), 2);

  // Both planners should modify velocities to avoid collision
  // The exact behavior may differ, but velocities should be reasonable
  for (const auto & cmd : modern_result.robot_commands) {
    if (!cmd.polar_velocity_target_mode.empty()) {
      EXPECT_LE(cmd.polar_velocity_target_mode[0].target_velocity_r, 2.0);
      EXPECT_GE(cmd.polar_velocity_target_mode[0].target_velocity_r, 0.0);
    }
  }
}

TEST_F(ModernORCAPlannerTest, ConstraintSystemIntegration)
{
  // Test that constraint system is properly initialized
  // This is a basic functionality test since constraint methods are private

  // Create a simple command to test constraint system internally
  crane_msgs::msg::RobotCommands commands;
  crane_msgs::msg::RobotCommand command;
  command.robot_id = 0;
  command.control_mode = crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE;

  crane_msgs::msg::SimpleVelocityTargetMode vel_target;
  vel_target.target_vx = 0.1;
  vel_target.target_vy = 0.1;
  command.simple_velocity_target_mode.push_back(vel_target);

  commands.robot_commands.push_back(command);

  // Test that system processes commands without errors
  EXPECT_NO_THROW({
    auto result = modern_planner_->calculateRobotCommand(commands, 0.0);
    EXPECT_EQ(result.robot_commands.size(), 1);
  });
}

TEST_F(ModernORCAPlannerTest, PerformanceComparison)
{
  // Planners already have world model initialized

  // Create a complex scenario with multiple robots
  crane_msgs::msg::RobotCommands commands;
  for (int i = 0; i < 6; ++i) {
    crane_msgs::msg::RobotCommand command;
    command.robot_id = i;
    command.control_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
    command.current_pose.x = (i % 3) * 1.0 - 1.0;
    command.current_pose.y = (i / 3) * 1.0 - 0.5;

    crane_msgs::msg::PositionTargetMode pos_target;
    pos_target.target_x = ((i + 3) % 3) * 1.0 - 1.0;
    pos_target.target_y = ((i + 3) / 3) * 1.0 - 0.5;
    pos_target.position_tolerance = 0.03;
    command.position_target_mode.push_back(pos_target);

    commands.robot_commands.push_back(command);
  }

  // Measure execution time for both planners
  auto start_modern = std::chrono::high_resolution_clock::now();
  auto modern_result = modern_planner_->calculateRobotCommand(commands, 0.0);
  auto end_modern = std::chrono::high_resolution_clock::now();

  auto start_rvo2 = std::chrono::high_resolution_clock::now();
  auto rvo2_result = rvo2_planner_->calculateRobotCommand(commands, 0.0);
  auto end_rvo2 = std::chrono::high_resolution_clock::now();

  auto modern_time = std::chrono::duration<double, std::milli>(end_modern - start_modern).count();
  auto rvo2_time = std::chrono::duration<double, std::milli>(end_rvo2 - start_rvo2).count();

  // Both should complete in reasonable time (less than 10ms for 6 robots)
  EXPECT_LT(modern_time, 10.0);
  EXPECT_LT(rvo2_time, 10.0);

  // Both should produce the same number of commands
  EXPECT_EQ(modern_result.robot_commands.size(), rvo2_result.robot_commands.size());

  std::cout << "ModernORCA time: " << modern_time << "ms" << std::endl;
  std::cout << "RVO2 time: " << rvo2_time << "ms" << std::endl;
}

}  // namespace crane
