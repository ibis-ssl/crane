// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_world_model_publisher/vision_ball_velocity_estimator.hpp>

namespace crane
{
namespace
{
constexpr double MAX_GAP_SEC = 0.1;
constexpr double MAX_SPEED = 6.5;
constexpr double FRAME_DT = 1.0 / 60.0;
constexpr double TOLERANCE = 1e-9;

auto key(uint32_t frame_number) -> VisionFrameKey
{
  return {0, frame_number, frame_number * FRAME_DT};
}

auto expectVelocity(const Eigen::Vector3d & actual, const Eigen::Vector3d & expected) -> void
{
  EXPECT_NEAR(actual.x(), expected.x(), TOLERANCE);
  EXPECT_NEAR(actual.y(), expected.y(), TOLERANCE);
  EXPECT_NEAR(actual.z(), expected.z(), TOLERANCE);
}

TEST(VisionBallVelocityEstimator, FirstObservationIsZero)
{
  VisionBallVelocityEstimator estimator(MAX_GAP_SEC, MAX_SPEED);
  expectVelocity(estimator.update(key(1), {1.0, 2.0, 0.0}, 10.0), Eigen::Vector3d::Zero());
}

TEST(VisionBallVelocityEstimator, ConsecutiveFramesGivePositionDifferenceOverReceiveTime)
{
  VisionBallVelocityEstimator estimator(MAX_GAP_SEC, MAX_SPEED);
  estimator.update(key(1), {0.0, 0.0, 0.0}, 10.0);
  // 受信時刻の差 0.02 秒で (0.04, -0.02) 動いた → (2, -1) m/s
  expectVelocity(estimator.update(key(2), {0.04, -0.02, 0.0}, 10.02), {2.0, -1.0, 0.0});
  expectVelocity(estimator.update(key(3), {0.10, -0.02, 0.0}, 10.04), {3.0, 0.0, 0.0});
}

TEST(VisionBallVelocityEstimator, SameFrameKeepsVelocityAndHistory)
{
  VisionBallVelocityEstimator estimator(MAX_GAP_SEC, MAX_SPEED);
  estimator.update(key(1), {0.0, 0.0, 0.0}, 10.0);
  estimator.update(key(2), {0.04, 0.0, 0.0}, 10.02);

  // 新しい Vision 入力が無い統合（同じ位置・同じ受信時刻）でも速度は 0 に戻らない
  expectVelocity(estimator.update(key(2), {0.04, 0.0, 0.0}, 10.02), {2.0, 0.0, 0.0});
  // UDP 経路の二重処理（同じフレームを少し後の受信時刻で再入力）でも速度は崩れない
  expectVelocity(estimator.update(key(2), {0.04, 0.0, 0.0}, 10.0201), {2.0, 0.0, 0.0});

  // 次のフレームの差分は、最初に受け付けた観測（受信時刻 10.02）を起点にする
  expectVelocity(estimator.update(key(3), {0.08, 0.0, 0.0}, 10.04), {2.0, 0.0, 0.0});
}

TEST(VisionBallVelocityEstimator, SameFrameNumberWithDifferentCaptureTimeIsNewFrame)
{
  // frame_number を送らない（常に 0 の）送信元でも、t_capture が違えば別フレームとして扱う
  VisionBallVelocityEstimator estimator(MAX_GAP_SEC, MAX_SPEED);
  estimator.update({0, 0, 1.00}, {0.0, 0.0, 0.0}, 10.0);
  expectVelocity(estimator.update({0, 0, 1.02}, {0.04, 0.0, 0.0}, 10.02), {2.0, 0.0, 0.0});
}

TEST(VisionBallVelocityEstimator, ResumeAfterTimeoutStartsFromZero)
{
  VisionBallVelocityEstimator estimator(MAX_GAP_SEC, MAX_SPEED);
  estimator.update(key(1), {0.0, 0.0, 0.0}, 10.0);
  estimator.update(key(2), {0.04, 0.0, 0.0}, 10.02);

  // 0.5 秒途切れた後の観測は初回扱い（古い位置からの平均速度を出さない）
  expectVelocity(estimator.update(key(32), {1.0, 0.0, 0.0}, 10.52), Eigen::Vector3d::Zero());
  // 再開後は再開時の観測を起点に差分を取る
  expectVelocity(estimator.update(key(33), {1.02, 0.0, 0.0}, 10.54), {1.0, 0.0, 0.0});
}

TEST(VisionBallVelocityEstimator, ClampsToMaxSpeedKeepingDirection)
{
  VisionBallVelocityEstimator estimator(MAX_GAP_SEC, MAX_SPEED);
  estimator.update(key(1), {0.0, 0.0, 0.0}, 10.0);
  // 0.02 秒で (0.6, 0.8) → 50 m/s を 6.5 m/s に縮める
  expectVelocity(estimator.update(key(2), {0.6, 0.8, 0.0}, 10.02), {3.9, 5.2, 0.0});
}
}  // namespace
}  // namespace crane
