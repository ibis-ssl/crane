// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "bag_events.hpp"

namespace cb = crane::bag;

namespace
{

/// フィールド 12 m × 9 m（ゴールラインは x = ±6 m）で、ボールをフィールド中央から
/// (x, y) へ動かした 2 フレームの bag を作る。goal_width = 0 は goal_size 未記録の古い bag。
cb::BagData ball_moved_to(double x, double y, double goal_width, bool on_positive_half = false)
{
  cb::BagData data;
  data.info.start_time_ns = 0;
  const double xs[] = {0.0, x};
  const double ys[] = {0.0, y};
  for (int i = 0; i < 2; ++i) {
    cb::TimestampedMsg<cb::WorldModel> f;
    f.timestamp_ns = i * 20'000'000;
    f.msg.ball_info.position = {xs[i], ys[i], 0.0};
    f.msg.field_info = {12.0, 9.0};
    f.msg.goal_size = {0.18, goal_width};
    f.msg.on_positive_half = on_positive_half;
    data.world_models.push_back(f);
  }
  return data;
}

std::vector<cb::Event> goals(const cb::BagData & data)
{
  return cb::detect_events(data, {cb::EVENT_GOAL});
}

}  // namespace

TEST(DetectGoals, UsesRecordedGoalWidth)
{
  // ゴール幅 1.8 m（半幅 0.9 m）。未記録時の半幅 0.5 m では入らない y = 0.7 もゴール
  EXPECT_EQ(goals(ball_moved_to(6.05, 0.7, 1.8)).size(), 1u);
  EXPECT_TRUE(goals(ball_moved_to(6.05, 1.0, 1.8)).empty());
}

TEST(DetectGoals, FallsBackToHalfMeterWithoutGoalSize)
{
  EXPECT_EQ(goals(ball_moved_to(6.05, 0.4, 0.0)).size(), 1u);
  EXPECT_TRUE(goals(ball_moved_to(6.05, 0.6, 0.0)).empty());
}

TEST(DetectGoals, OnPositiveHalfDecidesOurGoalSide)
{
  const auto ours = goals(ball_moved_to(6.05, 0.0, 1.8, true));
  ASSERT_EQ(ours.size(), 1u);
  EXPECT_EQ(ours[0].description.rfind("GOAL: OUR_GOAL", 0), 0u) << ours[0].description;

  const auto theirs = goals(ball_moved_to(6.05, 0.0, 1.8, false));
  ASSERT_EQ(theirs.size(), 1u);
  EXPECT_EQ(theirs[0].description.rfind("GOAL: THEIR_GOAL", 0), 0u) << theirs[0].description;
}

namespace
{

/// ボール速度 (vx, vy) だけを変えたフレームを 20 ms ごとに並べる。位置は (1, 2)
cb::BagData ball_velocities(const std::vector<std::pair<double, double>> & velocities)
{
  cb::BagData data;
  data.info.start_time_ns = 0;
  for (size_t i = 0; i < velocities.size(); ++i) {
    cb::TimestampedMsg<cb::WorldModel> f;
    f.timestamp_ns = static_cast<int64_t>(i) * 20'000'000;
    f.msg.ball_info.position = {1.0, 2.0, 0.0};
    f.msg.ball_info.velocity = {velocities[i].first, velocities[i].second};
    data.world_models.push_back(f);
  }
  return data;
}

}  // namespace

TEST(DetectBallSpeedSpikes, FiresOnceWhenPlanarSpeedCrossesThreshold)
{
  // 平面の速さ |(2,2)| = 2.83 は閾値 3.0 未満、|(2.2,-2.2)| = 3.11 は以上。
  // 上回ったままの間は出さず、いったん下回ってから再び上回ると次のイベントになる
  const auto events = cb::detect_events(
    ball_velocities({{2.0, 2.0}, {2.2, -2.2}, {-4.0, 0.0}, {2.0, 2.0}, {0.0, 3.0}}),
    {cb::EVENT_BALL_SPEED});
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].timestamp_ns, 20'000'000);
  EXPECT_EQ(events[0].description, "BALL_SPEED: 3.11m/s >= 3.0m/s at (1.00,2.00)");
  EXPECT_EQ(events[1].timestamp_ns, 80'000'000);
  EXPECT_EQ(events[1].description, "BALL_SPEED: 3.00m/s >= 3.0m/s at (1.00,2.00)");
}
