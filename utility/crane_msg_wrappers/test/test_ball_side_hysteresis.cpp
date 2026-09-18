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
auto makeWorldModel(double ball_y) -> crane_msgs::msg::WorldModel
{
  crane_msgs::msg::WorldModel msg;
  msg.field_info.x = 12.0;
  msg.field_info.y = 9.0;
  msg.penalty_area_size.x = 1.8;
  msg.penalty_area_size.y = 3.6;
  msg.goal_size.x = 0.18;
  msg.goal_size.y = 1.8;
  msg.ball_info.position.x = 0.02;
  msg.ball_info.position.y = ball_y;
  msg.ball_info.detected = true;
  return msg;
}

class BallSideHysteresisTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_ball_side_hysteresis");
    world_model = std::make_shared<WorldModelWrapper>(*node, false);
  }

  void feed(double ball_y) { world_model->update(makeWorldModel(ball_y)); }

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<WorldModelWrapper> world_model;
};

// ボールがセンターに静止しているとき、visionノイズ（実測 stdev 1.3mm）だけで
// 左右判定が反転してはいけない。反転すると second_threat_defender の守備目標が
// ゴール左角↔右角へ数mジャンプし、suitability関数も同時に振れてロール割り当てが
// 毎フレーム入れ替わる。
TEST_F(BallSideHysteresisTest, SideIsRetainedUnderVisionNoiseAtCenter)
{
  // 上側であることを確定させてから、センター付近のノイズだけを与える
  feed(1.0);
  ASSERT_DOUBLE_EQ(world_model->getBallSideSign(), 1.0);

  // 実測bag相当のノイズ幅（±4mm程度）で振動させる
  constexpr double NOISE[] = {0.0004, -0.0018, 0.0028, -0.0006, 0.0013, -0.0011, 0.0000, 0.0043};
  for (int i = 0; i < 100; i++) {
    feed(NOISE[i % 8]);
    EXPECT_DOUBLE_EQ(world_model->getBallSideSign(), 1.0) << "frame " << i;
  }

  // 下側から始めた場合も同様に保持されること
  feed(-1.0);
  ASSERT_DOUBLE_EQ(world_model->getBallSideSign(), -1.0);
  for (int i = 0; i < 100; i++) {
    feed(NOISE[i % 8]);
    EXPECT_DOUBLE_EQ(world_model->getBallSideSign(), -1.0) << "frame " << i;
  }
}

// デッドバンドを跨いでボールが実際に移動したときは、きちんと追従すること。
// （デッドバンドが追従そのものを殺していないことの確認）
TEST_F(BallSideHysteresisTest, SideFollowsBallCrossingDeadBand)
{
  feed(1.0);
  ASSERT_DOUBLE_EQ(world_model->getBallSideSign(), 1.0);

  feed(-0.3);
  EXPECT_DOUBLE_EQ(world_model->getBallSideSign(), -1.0);

  feed(0.3);
  EXPECT_DOUBLE_EQ(world_model->getBallSideSign(), 1.0);
}

// デッドバンド内の移動では切り替わらないこと
TEST_F(BallSideHysteresisTest, SideDoesNotFlipInsideDeadBand)
{
  feed(1.0);
  ASSERT_DOUBLE_EQ(world_model->getBallSideSign(), 1.0);

  feed(-0.15);
  EXPECT_DOUBLE_EQ(world_model->getBallSideSign(), 1.0);

  feed(0.15);
  EXPECT_DOUBLE_EQ(world_model->getBallSideSign(), 1.0);
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
