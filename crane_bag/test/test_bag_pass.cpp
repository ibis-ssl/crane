// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>

#include "bag_pass.hpp"

namespace cb = crane::bag;

namespace
{

constexpr double kDt = 0.02;  // 50Hz

/// 合成 WorldModel 時系列のビルダー。
/// record() 時点の状態をフレームとして積み、step() でボール物理（等減速転がり）を進める。
struct Sim
{
  cb::BagData data;
  double t = 0.0;
  double ball_x = 0.0, ball_y = 0.0, ball_z = 0.0;
  double vel_x = 0.0, vel_y = 0.0;
  double decel = 0.5;  // [m/s^2]
  std::vector<cb::RobotInfo> ours;
  std::vector<cb::RobotInfo> theirs;
  int32_t pass_target_id = -1;
  int32_t reserved_id = -1;
  cb::OngoingKickInfo ongoing;

  Sim() { data.info.start_time_ns = 0; }

  cb::RobotInfo & our(uint8_t id)
  {
    for (auto & r : ours) {
      if (r.id == id) return r;
    }
    cb::RobotInfo r;
    r.id = id;
    r.available_vision = true;
    ours.push_back(r);
    return ours.back();
  }

  cb::RobotInfo & their(uint8_t id)
  {
    for (auto & r : theirs) {
      if (r.id == id) return r;
    }
    cb::RobotInfo r;
    r.id = id;
    r.available_vision = true;
    theirs.push_back(r);
    return theirs.back();
  }

  void place_our(uint8_t id, double x, double y)
  {
    auto & r = our(id);
    r.pose.x = x;
    r.pose.y = y;
  }

  void place_their(uint8_t id, double x, double y)
  {
    auto & r = their(id);
    r.pose.x = x;
    r.pose.y = y;
  }

  void kick(double vx, double vy, int32_t kicker_id, bool friendly = true)
  {
    vel_x = vx;
    vel_y = vy;
    ongoing.present = true;
    ongoing.kicker_id = kicker_id;
    ongoing.is_kicker_friend = friendly;
    ongoing.origin = {ball_x, ball_y};
    ongoing.direction = std::atan2(vy, vx);
  }

  int64_t now_ns() const { return static_cast<int64_t>(t * 1e9); }

  void step(size_t n = 1)
  {
    for (size_t k = 0; k < n; ++k) {
      cb::TimestampedMsg<cb::WorldModel> f;
      f.timestamp_ns = now_ns();
      auto & wm = f.msg;
      wm.header_stamp_ns = f.timestamp_ns;
      wm.ball_info.position = {ball_x, ball_y, ball_z};
      wm.ball_info.velocity = {vel_x, vel_y};
      wm.field_info = {12.0, 9.0};
      wm.robot_info_ours = ours;
      wm.robot_info_theirs = theirs;
      wm.pass_target_id = pass_target_id;
      wm.recommended_pass_receiver_id = reserved_id;
      wm.ongoing_kick = ongoing;
      data.world_models.push_back(f);

      t += kDt;
      ball_x += vel_x * kDt;
      ball_y += vel_y * kDt;
      const double speed = std::sqrt(vel_x * vel_x + vel_y * vel_y);
      if (speed > 0.0) {
        const double new_speed = std::max(0.0, speed - decel * kDt);
        vel_x *= new_speed / speed;
        vel_y *= new_speed / speed;
      }
    }
  }
};

}  // namespace

TEST(BagPass, SuccessfulPassToIntendedReceiver)
{
  Sim sim;
  sim.place_our(1, 0.0, -0.1);  // キッカー
  sim.place_our(3, 2.0, 2.0);   // 受け手（45°方向 2.83m 先）
  sim.pass_target_id = 3;
  sim.reserved_id = 3;
  sim.step(10);               // 静止 0.2s
  sim.kick(2.121, 2.121, 1);  // |v| = 3.0, 45°
  sim.step(100);              // 2.0s 追跡

  auto events = cb::detect_pass_events(sim.data);
  ASSERT_EQ(events.size(), 1u);
  const auto & e = events[0];
  EXPECT_EQ(e.outcome, cb::PassOutcome::SUCCESS);
  EXPECT_EQ(e.kicker_id, 1);
  EXPECT_EQ(e.intended_receiver_id, 3);
  EXPECT_EQ(e.reserved_receiver_id, 3);
  EXPECT_EQ(e.first_toucher_id, 3);
  EXPECT_TRUE(e.first_toucher_ours);
  EXPECT_NEAR(e.kick_speed, 3.0, 0.15);
  EXPECT_NEAR(e.pass_distance, 2.83, 0.3);
  EXPECT_GT(e.forward_progress, 1.5);

  auto s = cb::summarize_passes(events);
  EXPECT_EQ(s.attempts, 1u);
  EXPECT_EQ(s.success, 1u);
  EXPECT_EQ(s.band_attempts[1], 1u);  // 1.5-4.0m 帯
  EXPECT_EQ(s.band_success[1], 1u);
}

TEST(BagPass, InterceptedByEnemyOnPassLine)
{
  Sim sim;
  sim.place_our(1, 0.0, -0.1);
  sim.place_our(3, 3.0, 3.0);
  sim.place_their(7, 1.0, 1.0);  // パスライン上の敵
  sim.pass_target_id = 3;
  sim.step(10);
  sim.kick(2.121, 2.121, 1);
  sim.step(150);

  auto events = cb::detect_pass_events(sim.data);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].outcome, cb::PassOutcome::INTERCEPTED);
  EXPECT_EQ(events[0].first_toucher_id, 7);
  EXPECT_FALSE(events[0].first_toucher_ours);
}

TEST(BagPass, WrongReceiverWhenAnotherFriendTouchesFirst)
{
  Sim sim;
  sim.place_our(1, 0.0, -0.1);
  sim.place_our(3, 3.0, 3.0);  // 意図受け手
  sim.place_our(4, 1.5, 1.5);  // ライン上の別味方
  sim.pass_target_id = 3;
  sim.step(10);
  sim.kick(2.121, 2.121, 1);
  sim.step(150);

  auto events = cb::detect_pass_events(sim.data);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].outcome, cb::PassOutcome::WRONG_RECEIVER);
  EXPECT_EQ(events[0].first_toucher_id, 4);
  EXPECT_TRUE(events[0].first_toucher_ours);
}

TEST(BagPass, OverrunWhenBallStopsShort)
{
  Sim sim;
  sim.decel = 1.5;
  sim.place_our(1, 0.0, -0.1);
  sim.place_our(3, 5.0, 5.0);  // 遠すぎる受け手
  sim.pass_target_id = 3;
  sim.step(10);
  sim.kick(1.6, 1.6, 1);  // |v|=2.26 → 最大到達 ~1.7m
  sim.step(200);

  auto events = cb::detect_pass_events(sim.data);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].outcome, cb::PassOutcome::OVERRUN);
  EXPECT_EQ(events[0].first_toucher_id, -1);
}

TEST(BagPass, OutOfPlayWhenBallLeavesField)
{
  Sim sim;
  sim.ball_y = 4.0;  // タッチライン際
  sim.place_our(1, 0.0, 3.9);
  sim.place_our(3, 2.0, 3.0);
  sim.pass_target_id = 3;
  sim.step(10);
  sim.kick(0.0, 3.0, 1);  // 場外方向
  sim.step(100);

  auto events = cb::detect_pass_events(sim.data);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].outcome, cb::PassOutcome::OUT_OF_PLAY);
}

TEST(BagPass, ShotTowardGoalIsExcluded)
{
  Sim sim;
  sim.ball_x = 2.0;
  sim.place_our(1, 1.9, 0.0);
  sim.place_our(3, 2.0, 2.0);  // 受け手は射線外
  sim.pass_target_id = 3;      // 分析器は継続的に pass_target を出している想定
  sim.step(10);
  sim.kick(4.0, 0.0, 1);  // ゴールマウス直撃方向
  sim.step(100);

  auto events = cb::detect_pass_events(sim.data);
  EXPECT_TRUE(events.empty());
}

TEST(BagPass, GoalwardKickWithReceiverOnRayIsPass)
{
  Sim sim;
  sim.place_our(1, 0.0, -0.1);
  sim.place_our(3, 2.5, 0.0);  // ゴール方向の射線上に受け手
  sim.pass_target_id = 3;
  sim.step(10);
  sim.kick(3.0, 0.0, 1);  // ゴール方向だが受け手が射線上
  sim.step(100);

  auto events = cb::detect_pass_events(sim.data);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].outcome, cb::PassOutcome::SUCCESS);
}

TEST(BagPass, NoPassTargetMeansNoAttempt)
{
  Sim sim;
  sim.place_our(1, 0.0, -0.1);
  sim.place_our(3, 2.0, 2.0);
  sim.pass_target_id = -1;
  sim.step(10);
  sim.kick(2.121, 2.121, 1);
  sim.step(100);

  auto events = cb::detect_pass_events(sim.data);
  EXPECT_TRUE(events.empty());
}

TEST(BagPass, EnemyKickIsIgnored)
{
  Sim sim;
  sim.place_their(5, 0.0, -0.1);
  sim.place_our(3, 2.0, 2.0);
  sim.pass_target_id = 3;  // 古い値が残っている想定
  sim.step(10);
  sim.kick(2.121, 2.121, 5, /*friendly=*/false);
  sim.step(100);

  auto events = cb::detect_pass_events(sim.data);
  EXPECT_TRUE(events.empty());
}

TEST(BagPass, FallbackAttributionWithoutOngoingKick)
{
  Sim sim;
  sim.place_our(1, 0.0, -0.1);  // ボール至近の味方 → キッカー帰属
  sim.place_our(3, 2.0, 2.0);
  sim.pass_target_id = 3;
  sim.step(10);
  sim.vel_x = 2.121;  // ongoing_kick なしで速度だけ立ち上げる
  sim.vel_y = 2.121;
  sim.step(100);

  auto events = cb::detect_pass_events(sim.data);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].kicker_id, 1);
  EXPECT_EQ(events[0].outcome, cb::PassOutcome::SUCCESS);
}

TEST(BagPass, StampBasedContactDetection)
{
  Sim sim;
  sim.place_our(1, 0.0, -0.1);
  sim.place_our(3, 2.0, 2.4);  // 射線から 0.28m 外（距離ベース接触は発火しない）
  sim.pass_target_id = 3;
  sim.step(10);
  sim.kick(2.121, 2.121, 1);
  // ボールが受け手最接近点付近に到達するまで進める
  while (sim.ball_x < 2.1) {
    sim.step(1);
  }
  // ドリブラーセンサー接触を模擬（header と同一クロックのスタンプ）
  sim.our(3).ball_contact_last_ns = sim.now_ns();
  sim.step(3);
  sim.our(3).ball_contact_last_ns = 0;
  sim.step(100);

  auto events = cb::detect_pass_events(sim.data);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].outcome, cb::PassOutcome::SUCCESS);
  EXPECT_EQ(events[0].first_toucher_id, 3);
}

TEST(BagPass, DistanceBandBoundaries)
{
  EXPECT_EQ(cb::pass_distance_band(0.5), 0u);
  EXPECT_EQ(cb::pass_distance_band(1.5), 1u);
  EXPECT_EQ(cb::pass_distance_band(4.0), 1u);
  EXPECT_EQ(cb::pass_distance_band(4.01), 2u);
}
