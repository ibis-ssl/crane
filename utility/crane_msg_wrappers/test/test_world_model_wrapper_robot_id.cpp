// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace crane
{
namespace
{
constexpr uint8_t MAX_ROBOT_NUM = 20;

auto makeWorldModelWithRobots(uint8_t robot_num) -> crane_msgs::msg::WorldModel
{
  crane_msgs::msg::WorldModel msg;
  msg.field_info.x = 12.0;
  msg.field_info.y = 9.0;
  msg.penalty_area_size.x = 1.8;
  msg.penalty_area_size.y = 3.6;
  msg.goal_size.x = 0.18;
  msg.goal_size.y = 1.8;
  for (uint8_t i = 0; i < robot_num; i++) {
    crane_msgs::msg::RobotInfo robot;
    robot.id = i;
    robot.available_vision = true;
    robot.pose.x = i * 0.3;
    robot.pose.y = -4.3;
    msg.robot_info_ours.push_back(robot);
    msg.robot_info_theirs.push_back(robot);
  }
  return msg;
}

class WorldModelWrapperRobotIdTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_world_model_wrapper_robot_id");
    world_model = std::make_shared<WorldModelWrapper>(*node, false);
  }

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<WorldModelWrapper> world_model;
};

// 未検出スロットのIDが0に化けると、ID経由でスロットを参照する側（RVOのエージェント番号など）が
// すべて0番ロボットを指してしまい、0番ロボットだけが異常挙動になる。
TEST_F(WorldModelWrapperRobotIdTest, IdIsAssignedOnConstruction)
{
  ASSERT_EQ(world_model->ours().robots.size(), MAX_ROBOT_NUM);
  ASSERT_EQ(world_model->theirs().robots.size(), MAX_ROBOT_NUM);
  for (uint8_t i = 0; i < MAX_ROBOT_NUM; i++) {
    EXPECT_EQ(world_model->ours().robots.at(i)->id, i);
    EXPECT_EQ(world_model->theirs().robots.at(i)->id, i);
  }
}

TEST_F(WorldModelWrapperRobotIdTest, IdIsKeptForUndetectedRobots)
{
  // 11台だけ検出されている状態。残りのスロットは一度もavailableにならない。
  constexpr uint8_t DETECTED_ROBOT_NUM = 11;
  world_model->update(makeWorldModelWithRobots(DETECTED_ROBOT_NUM));

  for (uint8_t i = 0; i < MAX_ROBOT_NUM; i++) {
    const auto & our_robot = world_model->ours().robots.at(i);
    const auto & their_robot = world_model->theirs().robots.at(i);
    EXPECT_EQ(our_robot->id, i);
    EXPECT_EQ(their_robot->id, i);
    EXPECT_EQ(our_robot->available_vision, i < DETECTED_ROBOT_NUM);
  }
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
