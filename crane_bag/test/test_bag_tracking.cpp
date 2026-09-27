// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "bag_tracking.hpp"

namespace cb = crane::bag;

namespace
{

constexpr int64_t kStartNs = 1'000'000'000;

/// bag 開始からの経過 [ms] ごとに、ボールだけのフレームを並べる
cb::BagData frames_at(const std::vector<int64_t> & elapsed_ms)
{
  cb::BagData data;
  data.info.start_time_ns = kStartNs;
  for (const auto ms : elapsed_ms) {
    cb::TimestampedMsg<cb::WorldModel> f;
    f.timestamp_ns = kStartNs + ms * 1'000'000;
    data.world_models.push_back(f);
  }
  return data;
}

cb::RobotInfo robot(uint8_t id, double x, double y)
{
  cb::RobotInfo r;
  r.id = id;
  r.pose = {x, y, 0.5};
  r.velocity = {3.0, 4.0};
  r.available_vision = true;
  return r;
}

template <typename StateT>
std::vector<double> times(const std::vector<StateT> & states)
{
  std::vector<double> ts;
  for (const auto & s : states) ts.push_back(s.t);
  return ts;
}

}  // namespace

TEST(TrackRobot, AdvancesIntervalOnFramesWithoutTheRobot)
{
  // 0 ms のフレームにはロボットがいない。間隔はロボットの有無に関わらず進むので、
  // 50 ms・150 ms ではなく 100 ms・200 ms が残る
  auto data = frames_at({0, 50, 100, 150, 200});
  for (size_t i = 1; i < data.world_models.size(); ++i) {
    data.world_models[i].msg.robot_info_ours.push_back(robot(2, 0.0, 0.0));
  }
  EXPECT_EQ(times(cb::track_robot(data, 2, true, 0.1)), (std::vector<double>{0.1, 0.2}));
}

TEST(TrackRobot, SkipsFramesGoingBackInTime)
{
  auto data = frames_at({0, 200, 100, 300});
  for (auto & f : data.world_models) f.msg.robot_info_ours.push_back(robot(2, 0.0, 0.0));
  EXPECT_EQ(times(cb::track_robot(data, 2, true, 0.0)), (std::vector<double>{0.0, 0.2, 0.3}));
}

TEST(TrackRobot, ReportsStateOfTheRequestedTeam)
{
  auto data = frames_at({0});
  auto & msg = data.world_models[0].msg;
  msg.ball_info.position = {1.0, 1.0, 0.0};
  msg.robot_info_ours.push_back(robot(2, 10.0, 10.0));
  msg.robot_info_theirs.push_back(robot(1, 0.0, 0.0));
  msg.robot_info_theirs.push_back(robot(2, 4.0, 5.0));

  const auto states = cb::track_robot(data, 2, false, 0.0);
  ASSERT_EQ(states.size(), 1u);
  const auto & s = states[0];
  EXPECT_EQ(s.robot_id, 2);
  EXPECT_DOUBLE_EQ(s.x, 4.0);
  EXPECT_DOUBLE_EQ(s.y, 5.0);
  EXPECT_DOUBLE_EQ(s.theta, 0.5);
  EXPECT_DOUBLE_EQ(s.vx, 3.0);
  EXPECT_DOUBLE_EQ(s.vy, 4.0);
  EXPECT_DOUBLE_EQ(s.speed, 5.0);
  EXPECT_DOUBLE_EQ(s.dist_to_ball, 5.0);
  EXPECT_TRUE(s.detected);
}

TEST(TrackBall, KeepsFramesExactlyOneIntervalApart)
{
  const auto data = frames_at({0, 50, 100, 150, 200});
  EXPECT_EQ(times(cb::track_ball(data, 0.1)), (std::vector<double>{0.0, 0.1, 0.2}));
}

TEST(TrackBall, SkipsFramesGoingBackInTime)
{
  const auto data = frames_at({0, 200, 100, 300});
  EXPECT_EQ(times(cb::track_ball(data, 0.0)), (std::vector<double>{0.0, 0.2, 0.3}));
}

TEST(TrackBall, ReportsPositionAndSpeed)
{
  auto data = frames_at({0});
  data.world_models[0].msg.ball_info.position = {1.5, -2.0, 0.0};
  data.world_models[0].msg.ball_info.velocity = {-3.0, 4.0};

  const auto states = cb::track_ball(data, 0.0);
  ASSERT_EQ(states.size(), 1u);
  EXPECT_DOUBLE_EQ(states[0].x, 1.5);
  EXPECT_DOUBLE_EQ(states[0].y, -2.0);
  EXPECT_DOUBLE_EQ(states[0].vx, -3.0);
  EXPECT_DOUBLE_EQ(states[0].vy, 4.0);
  EXPECT_DOUBLE_EQ(states[0].speed, 5.0);
}
