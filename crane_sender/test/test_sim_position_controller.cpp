// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_sender/sim_position_controller.hpp>

namespace crane
{
namespace
{
auto makeCommand(double target_x, double target_y, double max_velocity)
  -> crane_msgs::msg::RobotCommand
{
  crane_msgs::msg::RobotCommand command;
  command.control_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
  command.current_pose.x = 0.0;
  command.current_pose.y = 0.0;
  command.position_target_mode.emplace_back();
  command.position_target_mode.front().target_x = target_x;
  command.position_target_mode.front().target_y = target_y;
  command.local_planner_config.final_planned_max_velocity.value = max_velocity;
  return command;
}
}  // namespace

TEST(SimPositionControllerTest, AppliesPositionGain)
{
  const auto command = makeCommand(0.1, 0.0, 5.0);
  const auto velocity = calculateSimGlobalVelocity(command, {2.0, 100.0});
  EXPECT_NEAR(velocity.x(), 0.2, 1e-6);
  EXPECT_NEAR(velocity.y(), 0.0, 1e-9);
}

TEST(SimPositionControllerTest, AddsTerminalVelocityFeedforward)
{
  auto command = makeCommand(0.1, 0.0, 5.0);
  command.position_target_mode.front().terminal_velocity_x = 0.4;
  command.position_target_mode.front().speed_limit_at_target = 0.4;
  const auto velocity = calculateSimGlobalVelocity(command, {2.0, 100.0});
  EXPECT_NEAR(velocity.x(), 0.6, 1e-6);
}

TEST(SimPositionControllerTest, ClampsToMaximumVelocity)
{
  const auto command = makeCommand(10.0, 0.0, 1.5);
  const auto velocity = calculateSimGlobalVelocity(command, {2.0, 100.0});
  EXPECT_NEAR(velocity.norm(), 1.5, 1e-9);
}

TEST(SimPositionControllerTest, ClampsToBrakingEnvelope)
{
  const auto command = makeCommand(1.0, 0.0, 10.0);
  const auto velocity = calculateSimGlobalVelocity(command, {10.0, 0.5});
  EXPECT_NEAR(velocity.norm(), 1.0, 1e-9);
}

TEST(SimPositionControllerTest, StopsInsideToleranceWithoutFeedforward)
{
  auto command = makeCommand(0.005, 0.0, 5.0);
  command.position_target_mode.front().position_tolerance = 0.01;
  const auto velocity = calculateSimGlobalVelocity(command, {2.0, 3.0});
  EXPECT_NEAR(velocity.norm(), 0.0, 1e-9);
}

TEST(SimPositionControllerTest, RotatesFieldVelocityByCoordinateOffset)
{
  auto command = makeCommand(0.1, 0.0, 5.0);
  command.position_target_mode.front().terminal_velocity_x = 0.4;
  command.position_target_mode.front().speed_limit_at_target = 0.4;
  command.field_coordinate_theta_offset = M_PI_2;
  const auto velocity = calculateSimGlobalVelocity(command, {2.0, 100.0});
  EXPECT_NEAR(velocity.x(), 0.0, 1e-6);
  EXPECT_NEAR(velocity.y(), 0.6, 1e-6);
}

TEST(SimPositionControllerTest, RotatesMeasuredFieldVelocityWithTheSameConvention)
{
  const auto velocity = rotateFieldVector(Vector2(1.0, 0.0), -M_PI_2);
  EXPECT_NEAR(velocity.x(), 0.0, 1e-6);
  EXPECT_NEAR(velocity.y(), -1.0, 1e-6);
}

}  // namespace crane
