// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_basics/pid_controller.hpp>

namespace crane
{
// PIDControllerのテスト
TEST(PIDControllerTest, BasicControl)
{
  PIDController pid;
  pid.setGain(1.0, 0.5, 0.1);  // P=1.0, I=0.5, D=0.1

  // 初期エラー
  double out = pid.update(2.0, 0.1);
  // P項: 1.0 * 2.0 = 2.0
  // I項: 0.5 * 2.0 * 0.1 = 0.1
  // D項: 0.1 * (2.0 - 0.0) / 0.1 = 2.0
  EXPECT_DOUBLE_EQ(out, 4.1);

  // 2回目の更新（エラー減少）
  out = pid.update(1.0, 0.1);
  // P項: 1.0 * 1.0 = 1.0
  // I項: 0.5 * (2.0*0.1 + 1.0*0.1) = 0.15
  // D項: 0.1 * (1.0 - 2.0) / 0.1 = -1.0
  EXPECT_DOUBLE_EQ(out, 0.15);

  // 3回目の更新（エラー増加）
  out = pid.update(1.5, 0.1);
  // P項: 1.0 * 1.5 = 1.5
  // I項: 0.5 * (2.0*0.1 + 1.0*0.1 + 1.5*0.1) = 0.225
  // D項: 0.1 * (1.5 - 1.0) / 0.1 = 0.5
  EXPECT_DOUBLE_EQ(out, 2.225);
}

TEST(PIDControllerTest, IntegralClamp)
{
  PIDController pid;
  pid.setGain(1.0, 1.0, 0.0, 0.5);  // max_integral = 0.5

  // 積分項が制限を超えないケース
  double out = pid.update(0.2, 0.1);
  // P項: 1.0 * 0.2 = 0.2
  // I項: 1.0 * 0.2 * 0.1 = 0.02
  // D項: 0.0
  EXPECT_DOUBLE_EQ(out, 0.22);

  // 積分項が制限に達するケース
  out = pid.update(5.0, 0.1);
  // P項: 1.0 * 5.0 = 5.0
  // I項（制限前）: 1.0 * (0.2*0.1 + 5.0*0.1) = 0.52
  // I項（制限後）: 0.5
  // D項: 0.0
  EXPECT_DOUBLE_EQ(out, 5.5);

  // 積分項がすでに最大値なので変わらない
  out = pid.update(5.0, 0.1);
  // P項: 1.0 * 5.0 = 5.0
  // I項: 0.5 (クランプ済み)
  // D項: 0.0
  EXPECT_DOUBLE_EQ(out, 5.5);
}
}  // namespace crane
