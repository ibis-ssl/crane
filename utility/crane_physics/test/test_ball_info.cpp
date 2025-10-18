// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_physics/ball_info.hpp>

namespace crane
{
// Ball構造体のテスト
TEST(BallTest, MovementTests)
{
  Ball ball;
  ball.pos = Point(0.0, 0.0);
  ball.vel = Point(0.0, 0.0);
  ball.detected = true;

  // 停止状態のテスト
  EXPECT_FALSE(ball.isMoving());
  EXPECT_TRUE(ball.isStopped());

  // 移動中のテスト
  ball.vel = Point(1.0, 0.0);
  EXPECT_TRUE(ball.isMoving());
  EXPECT_FALSE(ball.isStopped());

  // カスタム閾値でのテスト
  ball.vel = Point(0.005, 0.0);
  EXPECT_FALSE(ball.isMoving(0.01));  // 0.01より小さいので停止とみなす
  EXPECT_TRUE(ball.isMoving(0.001));  // 0.001より大きいので移動中とみなす
}

TEST(BallTest, DirectionTests)
{
  Ball ball;
  ball.pos = Point(0.0, 0.0);
  ball.detected = true;

  // X方向に動いている場合
  ball.vel = Point(1.0, 0.0);

  // X正方向のポイントに向かっている
  Point target_point1;
  target_point1 = Point(5.0, 0.0);
  EXPECT_TRUE(ball.isMovingTowards(target_point1));
  EXPECT_FALSE(ball.isMovingAwayFrom(target_point1));

  // X負方向のポイントから離れている
  Point target_point2;
  target_point2 = Point(-5.0, 0.0);
  EXPECT_FALSE(ball.isMovingTowards(target_point2));
  EXPECT_TRUE(ball.isMovingAwayFrom(target_point2));

  // Y方向のポイント - 角度が60度以上なので向かっていない
  Point target_point3;
  target_point3 = Point(0.0, 5.0);
  EXPECT_FALSE(ball.isMovingTowards(target_point3));
  EXPECT_FALSE(ball.isMovingAwayFrom(target_point3));

  // 45度方向のポイント - 角度が60度未満なので向かっている
  Point target_point4;
  target_point4 = Point(5.0, 5.0);
  EXPECT_TRUE(ball.isMovingTowards(target_point4));
  EXPECT_FALSE(ball.isMovingAwayFrom(target_point4));

  // 閾値を30度に設定して再テスト
  EXPECT_FALSE(ball.isMovingTowards(target_point4, 30.0));
}

// Hysteresisクラスのテスト
TEST(HysteresisTest, StateTransition)
{
  Hysteresis hysteresis(0.3, 0.7);

  // 初期状態はLow
  EXPECT_FALSE(hysteresis.is_high);

  // 中間値 (0.3 < 0.5 < 0.7) を与えても状態は変わらない
  hysteresis.update(0.5);
  EXPECT_FALSE(hysteresis.is_high);

  // 上限値を超えるとHighになる
  hysteresis.update(0.8);
  EXPECT_TRUE(hysteresis.is_high);

  // 中間値 (0.3 < 0.5 < 0.7) を与えても状態は変わらない
  hysteresis.update(0.5);
  EXPECT_TRUE(hysteresis.is_high);

  // 下限値を下回るとLowになる
  hysteresis.update(0.2);
  EXPECT_FALSE(hysteresis.is_high);
}

// コールバックのテスト
TEST(HysteresisTest, Callbacks)
{
  Hysteresis hysteresis(0.3, 0.7);
  bool upper_called = false;
  bool lower_called = false;

  hysteresis.upper_callback = [&upper_called]() { upper_called = true; };
  hysteresis.lower_callback = [&lower_called]() { lower_called = true; };

  // 上限値を超えるとupper_callbackが呼ばれる
  hysteresis.update(0.8);
  EXPECT_TRUE(upper_called);
  EXPECT_FALSE(lower_called);

  // リセット
  upper_called = false;
  lower_called = false;

  // 上限値を超えてもすでにHighなので何も呼ばれない
  hysteresis.update(0.9);
  EXPECT_FALSE(upper_called);
  EXPECT_FALSE(lower_called);

  // 下限値を下回るとlower_callbackが呼ばれる
  hysteresis.update(0.2);
  EXPECT_FALSE(upper_called);
  EXPECT_TRUE(lower_called);
}
}  // namespace crane
