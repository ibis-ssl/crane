// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_basics/ball_info.hpp>
#include <crane_msgs/msg/ball_info.hpp>

namespace crane
{
// Ball構造体とBallInfoメッセージの変換テスト
TEST(BallMsgConversionTest, ToMsgConversion)
{
  Ball ball;
  ball.pos << 1.5, 2.5;
  ball.pos_z = 0.3;
  ball.vel << 0.5, -0.8;
  ball.vel_z = 1.2;
  ball.detected = true;
  ball.state = Ball::State::FLYING;
  ball.deceleration = 0.4;
  ball.gravity = -9.8;
  ball.air_resistance = 0.1;

  crane_msgs::msg::BallInfo msg;
  ball.toMsg(msg);

  // 位置・速度の確認
  EXPECT_DOUBLE_EQ(msg.position.x, 1.5);
  EXPECT_DOUBLE_EQ(msg.position.y, 2.5);
  EXPECT_DOUBLE_EQ(msg.position.z, 0.3);
  EXPECT_DOUBLE_EQ(msg.velocity.x, 0.5);
  EXPECT_DOUBLE_EQ(msg.velocity.y, -0.8);
  EXPECT_DOUBLE_EQ(msg.velocity.z, 1.2);
  EXPECT_FLOAT_EQ(msg.velocity_norm, ball.vel.norm());

  // 検出状態の確認
  EXPECT_EQ(msg.detected, true);

  // ボール状態の確認
  EXPECT_EQ(msg.state, 2); // FLYING

  // モデルパラメータの確認
  EXPECT_FLOAT_EQ(msg.deceleration, 0.4f);
  EXPECT_FLOAT_EQ(msg.gravity, -9.8f);
  EXPECT_FLOAT_EQ(msg.air_resistance, 0.1f);

}

TEST(BallMsgConversionTest, FromMsgConversion)
{
  crane_msgs::msg::BallInfo msg;
  msg.position.x = 3.0;
  msg.position.y = 4.0;
  msg.position.z = 0.1;
  msg.velocity.x = -1.0;
  msg.velocity.y = 1.5;
  msg.velocity.z = -0.5;
  msg.detected = false;
  msg.state = 1; // ROLLING
  msg.deceleration = 0.6f;
  msg.gravity = -9.82f;
  msg.air_resistance = 0.05f;

  Ball ball;
  ball.fromMsg(msg);

  // 位置・速度の確認
  EXPECT_DOUBLE_EQ(ball.pos.x(), 3.0);
  EXPECT_DOUBLE_EQ(ball.pos.y(), 4.0);
  EXPECT_DOUBLE_EQ(ball.pos_z, 0.1);
  EXPECT_DOUBLE_EQ(ball.vel.x(), -1.0);
  EXPECT_DOUBLE_EQ(ball.vel.y(), 1.5);
  EXPECT_DOUBLE_EQ(ball.vel_z, -0.5);

  // 検出状態の確認
  EXPECT_EQ(ball.detected, false);

  // ボール状態の確認
  EXPECT_EQ(ball.state, Ball::State::ROLLING);

  // モデルパラメータの確認（float→doubleの精度を考慮）
  EXPECT_NEAR(ball.deceleration, 0.6, 1e-6);
  EXPECT_NEAR(ball.gravity, -9.82, 1e-6);
  EXPECT_NEAR(ball.air_resistance, 0.05, 1e-6);
}

TEST(BallMsgConversionTest, RoundTripConversion)
{
  Ball original_ball;
  original_ball.pos << 2.0, -1.5;
  original_ball.pos_z = 0.05;
  original_ball.vel << 0.3, 0.7;
  original_ball.vel_z = 0.8;
  original_ball.detected = true;
  original_ball.state = Ball::State::STOPPED;
  original_ball.deceleration = 0.55;
  original_ball.gravity = -9.81;
  original_ball.air_resistance = 0.02;

  // Ball -> Msg -> Ball
  crane_msgs::msg::BallInfo msg;
  original_ball.toMsg(msg);
  
  Ball converted_ball;
  converted_ball.fromMsg(msg);

  // 元の値と変換後の値が同じかチェック
  EXPECT_DOUBLE_EQ(converted_ball.pos.x(), original_ball.pos.x());
  EXPECT_DOUBLE_EQ(converted_ball.pos.y(), original_ball.pos.y());
  EXPECT_DOUBLE_EQ(converted_ball.pos_z, original_ball.pos_z);
  EXPECT_DOUBLE_EQ(converted_ball.vel.x(), original_ball.vel.x());
  EXPECT_DOUBLE_EQ(converted_ball.vel.y(), original_ball.vel.y());
  EXPECT_DOUBLE_EQ(converted_ball.vel_z, original_ball.vel_z);
  EXPECT_EQ(converted_ball.detected, original_ball.detected);
  EXPECT_EQ(converted_ball.state, original_ball.state);
  EXPECT_NEAR(converted_ball.deceleration, original_ball.deceleration, 1e-6);
  EXPECT_NEAR(converted_ball.gravity, original_ball.gravity, 1e-6);
  EXPECT_NEAR(converted_ball.air_resistance, original_ball.air_resistance, 1e-6);
}

TEST(BallMsgConversionTest, AllStateConversions)
{
  // 全ての状態の変換をテスト
  Ball ball;
  crane_msgs::msg::BallInfo msg;

  // STOPPED
  ball.state = Ball::State::STOPPED;
  ball.toMsg(msg);
  EXPECT_EQ(msg.state, 0);
  
  ball.fromMsg(msg);
  EXPECT_EQ(ball.state, Ball::State::STOPPED);

  // ROLLING
  ball.state = Ball::State::ROLLING;
  ball.toMsg(msg);
  EXPECT_EQ(msg.state, 1);
  
  ball.fromMsg(msg);
  EXPECT_EQ(ball.state, Ball::State::ROLLING);

  // FLYING
  ball.state = Ball::State::FLYING;
  ball.toMsg(msg);
  EXPECT_EQ(msg.state, 2);
  
  ball.fromMsg(msg);
  EXPECT_EQ(ball.state, Ball::State::FLYING);

  // 無効な状態値
  msg.state = 99;
  ball.fromMsg(msg);
  EXPECT_EQ(ball.state, Ball::State::STOPPED); // デフォルト値
}
}  // namespace crane
