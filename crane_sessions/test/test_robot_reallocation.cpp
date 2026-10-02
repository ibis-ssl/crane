// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <algorithm>
#include <crane_sessions/ball_near_by_positioner_skill_session.hpp>
#include <crane_sessions/emplace_robot_session.hpp>
#include <crane_sessions/forward_session.hpp>
#include <memory>
#include <vector>

namespace crane
{
// setAllocatedRobots で割り当てが変わったら、次の周期の指令が新しいロボット ID に追従すること
template <typename Session>
class RobotReallocation : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_robot_reallocation");
    wm = std::make_shared<WorldModelWrapper>(*node, false);
    crane_msgs::msg::WorldModel msg;
    msg.field_info.x = 12;
    msg.field_info.y = 9;
    msg.penalty_area_size.x = 1.8;
    msg.penalty_area_size.y = 3.6;
    msg.goal_size.y = 1.8;
    msg.our_max_allowed_bots = 6;
    msg.play_situation.command.value = crane_msgs::msg::PlaySituation::INPLAY;
    msg.play_situation.command.name = "INPLAY";
    msg.ball_info.detected = true;
    for (uint8_t id = 0; id < 6; ++id) {
      crane_msgs::msg::RobotInfo robot;
      robot.id = id;
      robot.available_vision = true;
      robot.available_feedback = true;
      robot.pose.x = -2;
      robot.pose.y = id * 0.5;
      msg.robot_info_ours.push_back(robot);
    }
    wm->update(msg);
    session = std::make_shared<Session>(wm, *node);
  }

  auto commandedIds(const std::vector<uint8_t> & robot_ids) -> std::vector<uint8_t>
  {
    session->setAllocatedRobots(robot_ids);
    std::vector<uint8_t> ids;
    for (const auto & command : session->getPositionCommands().robot_commands) {
      ids.push_back(command.robot_id);
    }
    std::ranges::sort(ids);
    return ids;
  }

  rclcpp::Node::SharedPtr node;
  WorldModelWrapper::SharedPtr wm;
  std::shared_ptr<Session> session;
};

using Sessions =
  ::testing::Types<BallNearByPositionerSkillSession, EmplaceRobotSession, ForwardSession>;
TYPED_TEST_SUITE(RobotReallocation, Sessions);

TYPED_TEST(RobotReallocation, CommandsFollowSameSizeIdSwap)
{
  EXPECT_EQ(this->commandedIds({1, 2}), std::vector<uint8_t>({1, 2}));
  EXPECT_EQ(this->commandedIds({3, 4}), std::vector<uint8_t>({3, 4}));
  EXPECT_EQ(this->commandedIds({3, 5}), std::vector<uint8_t>({3, 5}));
}

TYPED_TEST(RobotReallocation, CommandsFollowSizeChange)
{
  EXPECT_EQ(this->commandedIds({1, 2}), std::vector<uint8_t>({1, 2}));
  EXPECT_EQ(this->commandedIds({4}), std::vector<uint8_t>({4}));
  EXPECT_EQ(this->commandedIds({}), std::vector<uint8_t>());
  EXPECT_EQ(this->commandedIds({0, 5}), std::vector<uint8_t>({0, 5}));
}
}  // namespace crane

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
