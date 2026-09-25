// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <chrono>
#include <crane_robot_skills/center_stop_kick.hpp>
#include <memory>
#include <thread>

namespace crane
{
// CenterStopKick は rclcpp::Clock()（システム時刻）を直接読むので、待ち時間は実時間で sleep する
class CenterStopKickTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_center_stop_kick");
    wm = std::make_shared<WorldModelWrapper>(*node, false);
    msg.field_info.x = 12;
    msg.field_info.y = 9;
    msg.ball_info.detected = true;
    msg.ball_info.position.x = 1.0;
    crane_msgs::msg::RobotInfo robot;
    robot.id = 0;
    robot.available_vision = true;
    robot.available_feedback = true;
    // 目標 (0,0) に対するボール (1,0) の後方 approach_distance_(0.2m) のキック位置
    robot.pose.x = 1.2;
    msg.robot_info_ours.push_back(robot);
    wm->update(msg);
  }

  void setBallVelocityX(double vx)
  {
    msg.ball_info.velocity.x = vx;
    wm->update(msg);
  }

  static void sleepSec(double sec)
  {
    std::this_thread::sleep_for(std::chrono::duration<double>(sec));
  }

  rclcpp::Node::SharedPtr node;
  WorldModelWrapper::SharedPtr wm;
  crane_msgs::msg::WorldModel msg;
};

TEST_F(CenterStopKickTest, RetryWaitsForNewKickInsteadOfCompletingImmediately)
{
  using State = skills::CenterStopKickState;
  skills::CenterStopKick skill(0, wm);

  skill.run();
  ASSERT_EQ(skill.getCurrentState(), State::WAIT_BALL_STOP);
  sleepSec(1.05);  // ボール停止の継続（stop_time_threshold_ = 1.0s）
  skill.run();
  ASSERT_EQ(skill.getCurrentState(), State::POSITION_BEHIND_BALL);
  skill.run();
  ASSERT_EQ(skill.getCurrentState(), State::KICK_EXECUTE);

  // 1 回目のキック: 0.2s 以上経ってからボールが中心方向へ動き出す
  sleepSec(0.25);
  setBallVelocityX(-1.0);
  skill.run();
  ASSERT_EQ(skill.getCurrentState(), State::KICK_COMPLETE);

  // ボールは中心から外れたまま止まり、1s の結果確認の後でリトライに入る
  setBallVelocityX(0.0);
  sleepSec(1.05);
  skill.run();
  ASSERT_EQ(skill.getCurrentState(), State::WAIT_BALL_STOP);
  sleepSec(1.05);
  skill.run();
  ASSERT_EQ(skill.getCurrentState(), State::POSITION_BEHIND_BALL);
  skill.run();
  ASSERT_EQ(skill.getCurrentState(), State::KICK_EXECUTE);

  // 2 回目のキック直後はまだ 0.2s 経っていないので、ボールが動いていても完了にしない。
  // リトライでキックの状態が戻っていないと、1 回目のキック時刻で即 KICK_COMPLETE に進む
  setBallVelocityX(-1.0);
  skill.run();
  EXPECT_EQ(skill.getCurrentState(), State::KICK_EXECUTE);
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
