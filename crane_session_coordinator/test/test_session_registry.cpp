// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_session_coordinator/session_registry.hpp>
#include <memory>

namespace crane
{
class SessionRegistryIsAssigned : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_session_registry");
    wm = std::make_shared<WorldModelWrapper>(*node, false);
  }

  auto addSession(const std::string & name, const std::vector<uint8_t> & robot_ids) -> void
  {
    auto session = registry.getOrCreatePlanner(name, wm, *node, {});
    ASSERT_NE(session, nullptr);
    session->setAllocatedRobots(robot_ids);
    registry.addPlanner(session);
  }

  rclcpp::Node::SharedPtr node;
  WorldModelWrapper::SharedPtr wm;
  SessionRegistry registry;
};

TEST_F(SessionRegistryIsAssigned, FalseWhenNoSession)
{
  EXPECT_FALSE(registry.isAssigned("attacker_skill", 0));
}

TEST_F(SessionRegistryIsAssigned, TrueOnlyForRobotInSessionWithThatName)
{
  addSession("attacker_skill", {1});
  addSession("pass_receive", {2, 3});

  EXPECT_TRUE(registry.isAssigned("attacker_skill", 1));
  EXPECT_TRUE(registry.isAssigned("pass_receive", 3));
  // 別名のセッションに居るロボットは数えない
  EXPECT_FALSE(registry.isAssigned("attacker_skill", 2));
  EXPECT_FALSE(registry.isAssigned("pass_receive", 1));
  // どのセッションにも居ないロボット
  EXPECT_FALSE(registry.isAssigned("pass_receive", 4));
  // 登録されていない名前
  EXPECT_FALSE(registry.isAssigned("waiter", 1));
}
}  // namespace crane

auto main(int argc, char ** argv) -> int
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
