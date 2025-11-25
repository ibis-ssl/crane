// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_msgs/msg/ball_info.hpp>
#include <crane_physics/ball_info.hpp>
#include <crane_physics/ball_physics_model.hpp>

namespace crane
{
// Ball構造体とBallInfoメッセージの変換テスト
TEST(BallMsgConversionTest, ToMsgConversion)
{
  // BallPhysicsModelを作成
  BallPhysicsModel::Config config;
  config.deceleration = 0.4;
  config.gravity = -9.8;
  config.air_resistance = 0.1;
  auto physics_model = std::make_shared<BallPhysicsModel>(config);

  Ball ball(physics_model);
  ball.pos = Point(1.5, 2.5);
  ball.pos_z = 0.3;
  ball.vel = Point(0.5, -0.8);
  ball.vel_z = 1.2;
  ball.detected = true;
  ball.state = Ball::State::FLYING;

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
  EXPECT_EQ(msg.state, 2);  // FLYING

  // 物理モデルパラメータの確認
  EXPECT_DOUBLE_EQ(msg.physics_config.deceleration, 0.4);
  EXPECT_DOUBLE_EQ(msg.physics_config.gravity, -9.8);
  EXPECT_DOUBLE_EQ(msg.physics_config.air_resistance, 0.1);
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
  msg.state = 1;  // ROLLING
  msg.physics_config.deceleration = 0.6;
  msg.physics_config.gravity = -9.82;
  msg.physics_config.air_resistance = 0.05;

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

  // 物理モデルパラメータの確認
  auto config = ball.getPhysicsModel()->getConfig();
  EXPECT_NEAR(config.deceleration, 0.6, 1e-6);
  EXPECT_NEAR(config.gravity, -9.82, 1e-6);
  EXPECT_NEAR(config.air_resistance, 0.05, 1e-6);
}

TEST(BallMsgConversionTest, RoundTripConversion)
{
  // BallPhysicsModelを作成
  BallPhysicsModel::Config config;
  config.deceleration = 0.55;
  config.gravity = -9.81;
  config.air_resistance = 0.02;
  auto physics_model = std::make_shared<BallPhysicsModel>(config);

  Ball original_ball(physics_model);
  original_ball.pos = Point(2.0, -1.5);
  original_ball.pos_z = 0.05;
  original_ball.vel = Point(0.3, 0.7);
  original_ball.vel_z = 0.8;
  original_ball.detected = true;
  original_ball.state = Ball::State::STOPPED;

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

  // 物理モデルパラメータの比較
  auto converted_config = converted_ball.getPhysicsModel()->getConfig();
  auto original_config = original_ball.getPhysicsModel()->getConfig();
  EXPECT_NEAR(converted_config.deceleration, original_config.deceleration, 1e-6);
  EXPECT_NEAR(converted_config.gravity, original_config.gravity, 1e-6);
  EXPECT_NEAR(converted_config.air_resistance, original_config.air_resistance, 1e-6);
}

TEST(BallMsgConversionTest, AllStateConversions)
{
  // 全ての状態の変換をテスト
  Ball ball;
  ball.pos = Point(0.0, 0.0);
  ball.pos_z = 0.0;
  ball.vel = Point(0.0, 0.0);
  ball.vel_z = 0.0;
  ball.detected = false;
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
  EXPECT_EQ(ball.state, Ball::State::STOPPED);  // デフォルト値
}
}  // namespace crane
