// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <crane_session_coordinator/robot_allocator.hpp>
#include <crane_sessions/attacker_skill_session.hpp>
#include <crane_sessions/pass_receiver_session.hpp>
#include <filesystem>
#include <memory>

namespace crane
{
class PassPlanExecution : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_pass_plan_execution");
    wm = std::make_shared<WorldModelWrapper>(*node, false);
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
    auto & plan = msg.game_analysis.pass_plan;
    plan.state = plan.STATE_PLANNING;
    plan.kicker_id = 1;
    plan.receiver_id = 2;
    plan.receive_point.x = 3;
    plan.receive_point.y = 2;
    plan.kick_speed = 3.2;
    plan.score = 1;
    wm->update(msg);
  }

  rclcpp::Node::SharedPtr node;
  WorldModelWrapper::SharedPtr wm;
  crane_msgs::msg::WorldModel msg;
};

TEST_F(PassPlanExecution, ReservesPairBeforeDefenders)
{
  auto config = std::make_shared<ConfigurationManager>(
    ament_index_cpp::get_package_share_directory("crane_session_coordinator"));
  auto registry = std::make_shared<SessionRegistry>();
  RobotAllocator allocator(config, registry, node->get_logger());
  const auto results =
    allocator.allocate("INPLAY", {0, 1, 2, 3, 4, 5}, wm, *node, msg.play_situation);
  bool kicker = false;
  bool receiver = false;
  for (const auto & result : results.results) {
    if (result.name == "attacker_skill") {
      EXPECT_EQ(result.selected_robots, std::vector<uint8_t>({1}));
      kicker = true;
    } else if (result.name == "pass_receive") {
      EXPECT_EQ(result.selected_robots, std::vector<uint8_t>({2}));
      receiver = true;
    } else {
      for (auto id : result.selected_robots) {
        EXPECT_NE(id, 1);
        EXPECT_NE(id, 2);
      }
    }
  }
  EXPECT_TRUE(kicker);
  EXPECT_TRUE(receiver);
}

TEST_F(PassPlanExecution, ReceiverMovesToPlannedPointBeforeKick)
{
  PassReceiverSession session(wm, *node);
  const auto [status, commands] = session.calculatePositionCommand({RobotIdentifier{true, 2}});
  ASSERT_EQ(commands.size(), 1u);
  ASSERT_EQ(commands.front().position_target_mode.size(), 1u);
  // ドリブラー位置補正分(約0.1m)を除いて、現位置ではなく計画受領点へ向かう。
  EXPECT_NEAR(commands.front().position_target_mode.front().target_x, 3, 0.2);
  EXPECT_NEAR(commands.front().position_target_mode.front().target_y, 2, 0.2);
  EXPECT_FLOAT_EQ(commands.front().kick_power, 0);
}

TEST_F(PassPlanExecution, AttackerUsesPlannedTargetAndSpeed)
{
  msg.goal_size.y = 0;  // シュート角度をなくしパスを選ばせる
  msg.robot_info_ours[1].pose.x = 0;
  msg.robot_info_ours[1].pose.y = 0;
  wm->update(msg);
  skills::Attacker attacker(1, wm);
  attacker.run();
  ASSERT_EQ(attacker.pass_receiver_id, 2);
  const Point target = attacker.kick_skill.getParameter<Point>("target");
  EXPECT_DOUBLE_EQ(target.x(), 3);
  EXPECT_DOUBLE_EQ(target.y(), 2);
  EXPECT_NEAR(attacker.kick_skill.getParameter<double>("target_kick_speed"), 3.2, 1e-6);
  EXPECT_FALSE(attacker.kick_skill.getParameter<bool>("chip_kick"));
}

TEST_F(PassPlanExecution, FlightKeepsKickerDespiteChangedRecommendation)
{
  msg.game_analysis.pass_plan.state = crane_msgs::msg::PassPlan::STATE_BALL_IN_FLIGHT;
  wm->update(msg);
  AttackerSkillSession session(wm, *node);
  auto analysis = msg.game_analysis;
  analysis.recommended_attacker_id = 3;
  session.setGameAnalysis(analysis);
  const auto suitability = session.getRobotSuitabilityFunc();
  EXPECT_LT(suitability(wm->getOurRobot(1)), suitability(wm->getOurRobot(3)));
  EXPECT_LT(suitability(wm->getOurRobot(1)), suitability(wm->getOurRobot(2)));
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
