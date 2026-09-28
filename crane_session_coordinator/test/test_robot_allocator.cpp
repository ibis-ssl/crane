// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_session_coordinator/robot_allocator.hpp>
#include <filesystem>
#include <fstream>
#include <memory>

namespace crane
{
// waiter の適性は robot->id そのもの（小さいほど良い）なので、
// 候補 ID の組み合わせだけで適性差を決められる。
class RobotAllocatorHysteresis : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_robot_allocator");
    wm = std::make_shared<WorldModelWrapper>(*node, false);
    crane_msgs::msg::WorldModel msg;
    msg.field_info.x = 12;
    msg.field_info.y = 9;
    msg.penalty_area_size.x = 1.8;
    msg.penalty_area_size.y = 3.6;
    msg.goal_size.y = 1.8;
    msg.our_max_allowed_bots = 6;
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

    const auto config_path =
      std::filesystem::path(::testing::TempDir()) / "robot_allocator_hysteresis.yaml";
    std::ofstream(config_path) << "situations:\n"
                                  "  WAIT:\n"
                                  "    description: waiter only\n"
                                  "    sessions:\n"
                                  "      - name: waiter\n"
                                  "        max_robots: 1\n";
    allocator = std::make_unique<RobotAllocator>(
      std::make_shared<ConfigurationManager>(config_path), std::make_shared<SessionRegistry>(),
      node->get_logger());
  }

  auto allocateWaiter(std::vector<uint8_t> selectable) -> std::vector<uint8_t>
  {
    const auto results =
      allocator->allocate("WAIT", std::move(selectable), wm, *node, play_situation);
    EXPECT_EQ(results.results.size(), 1u);
    return results.results.empty() ? std::vector<uint8_t>{}
                                   : results.results.front().selected_robots;
  }

  rclcpp::Node::SharedPtr node;
  WorldModelWrapper::SharedPtr wm;
  crane_msgs::msg::PlaySituation play_situation;
  std::unique_ptr<RobotAllocator> allocator;
};

TEST_F(RobotAllocatorHysteresis, KeepsPreviousRobotWhenSuitabilityGapIsSmall)
{
  ASSERT_EQ(allocateWaiter({1, 2}), std::vector<uint8_t>({1}));
  // 0 の方が適性は 1 良いが、継続ボーナス分に届かないので 1 のまま。
  EXPECT_EQ(allocateWaiter({0, 1}), std::vector<uint8_t>({1}));
}

TEST_F(RobotAllocatorHysteresis, SwitchesRobotWhenSuitabilityGapExceedsBonus)
{
  ASSERT_EQ(allocateWaiter({2, 5}), std::vector<uint8_t>({2}));
  // 適性差 2 は継続ボーナスを上回るので 0 へ切り替わる。
  EXPECT_EQ(allocateWaiter({0, 2}), std::vector<uint8_t>({0}));
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
