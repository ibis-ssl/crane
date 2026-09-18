// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_game_analyzer/metrics/pass_plan_metrics.hpp>
#include <crane_msg_wrappers/pass_plan.hpp>
#include <crane_msg_wrappers/pass_rating.hpp>
#include <crane_physics/ball_physics_model.hpp>
#include <deque>
#include <memory>

namespace crane
{
class PassPlanTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_pass_plan");
    wm = std::make_shared<WorldModelWrapper>(*node, false);
    msg.field_info.x = 12;
    msg.field_info.y = 9;
    msg.penalty_area_size.x = 1.8;
    msg.penalty_area_size.y = 3.6;
    msg.goal_size.y = 1.8;
    msg.on_positive_half = false;
    msg.play_situation.command.value = crane_msgs::msg::PlaySituation::INPLAY;
    msg.ball_info.detected = true;
    for (uint8_t id = 1; id <= 3; ++id) {
      crane_msgs::msg::RobotInfo robot;
      robot.id = id;
      robot.available_vision = true;
      robot.available_feedback = true;
      robot.pose.x = id == 1 ? 0 : 3;
      robot.pose.y = id == 1 ? 0 : (id == 2 ? 2 : -2);
      msg.robot_info_ours.push_back(robot);
    }
    analysis.recommended_attacker_id = 1;
    metric.setRecomputeInterval(0.0);
    metric.setMinPassScore(0.1);
  }

  void compute()
  {
    wm->update(msg);
    metrics::MetricContext ctx{wm.get(), &history, node->get_clock(), nullptr, analysis};
    metric.compute(ctx);
  }

  void startBall(const crane_msgs::msg::PassPlan & plan)
  {
    const Vector2 direction = Point(plan.receive_point.x, plan.receive_point.y).normalized();
    msg.ball_info.velocity.x = direction.x() * 2.0;
    msg.ball_info.velocity.y = direction.y() * 2.0;
    msg.ball_info.position.x = direction.x() * 0.1;
    msg.ball_info.position.y = direction.y() * 0.1;
  }

  rclcpp::Node::SharedPtr node;
  WorldModelWrapper::SharedPtr wm;
  crane_msgs::msg::WorldModel msg;
  crane_msgs::msg::GameAnalysis analysis;
  std::deque<crane_msgs::msg::BallInfo> history;
  metrics::PassPlanMetric metric;
};

TEST_F(PassPlanTest, UsesSameFlightForScoreAndFeasibility)
{
  compute();
  const auto & plan = analysis.pass_plan;
  ASSERT_EQ(plan.state, plan.STATE_PLANNING);
  EXPECT_TRUE(isUsablePassPlan(plan, *wm));
  PassRatingConfig config;
  config.straight_flight =
    StraightPassFlight{plan.kick_speed, wm->ball().getPhysicsModel()->getDeceleration()};
  const auto rating = ratePassCandidate(
    wm.get(), wm->ball().pos, Point(plan.receive_point.x, plan.receive_point.y), config);
  EXPECT_NEAR(plan.score, rating.score, 1e-5);
  EXPECT_EQ(analysis.recommended_pass_receiver_id, plan.receiver_id);
}

// 評価予算を使い切っても後続の受け手に届くか。
//
// 受け手3を原点寄りへ動かしてからテストする。既定配置の (3,-2) は、
// ボール減速度を実測値 0.36 m/s^2 に直したあとでは飛行 2.49 秒となり、
// パスラインから 3.33m 離れた敵でも 4.65m 到達できて本当に迎撃される。
// 以前この配置で計画が立っていたのは、減速度を 0.7 と過大に見積もっていて
// 飛行が 1.71 秒・敵到達 2.19m と出ていたためで、物理の方が誤っていた。
// このテストが固定したいのは予算配分であって特定のパス距離ではないので、
// 前提が成り立つ距離に縮める。
TEST_F(PassPlanTest, BudgetReachesLaterReceiverEvenWhenFirstIsBlocked)
{
  metric.setMaxCandidates(2);
  for (auto & robot : msg.robot_info_ours) {
    if (robot.id == 3) {
      robot.pose.x = 1.5;
      robot.pose.y = -1.0;
    }
  }
  crane_msgs::msg::RobotInfo enemy;
  enemy.id = 1;
  enemy.available_vision = true;
  enemy.pose = msg.robot_info_ours[1].pose;
  msg.robot_info_theirs.push_back(enemy);
  compute();
  ASSERT_EQ(analysis.pass_plan.state, analysis.pass_plan.STATE_PLANNING);
  EXPECT_EQ(analysis.pass_plan.receiver_id, 3);
}

// 計画の受け手以外の味方が先に触れてしまう受領点は採用しない。
//
// 実測では、キック較正を直したあとの失敗の最大要因がこれだった（15試行中4件）。
// ボールは計画どおりの地点に届く（受領点誤差 0.03〜0.75m）のに、そこへ来たのが
// 計画の受け手ではない、という形で契約が破れる。
// ここでは受け手2の現在位置に別の味方を重ねて、受け手2が選ばれなくなることを見る。
TEST_F(PassPlanTest, RejectsReceivePointThatAnotherFriendWouldReachFirst)
{
  compute();
  ASSERT_EQ(analysis.pass_plan.state, analysis.pass_plan.STATE_PLANNING);
  const int original_receiver = analysis.pass_plan.receiver_id;
  ASSERT_GE(original_receiver, 0);

  // 選ばれた受け手の真上に別の味方を置く。経路上・受領点のどちらでも
  // この味方が先着するので、この受け手を使う候補は全滅するはず。
  crane_msgs::msg::RobotInfo poacher;
  poacher.id = 4;
  poacher.available_vision = true;
  poacher.available_feedback = true;
  for (const auto & robot : msg.robot_info_ours) {
    if (robot.id == original_receiver) {
      poacher.pose = robot.pose;
    }
  }
  msg.robot_info_ours.push_back(poacher);
  compute();
  EXPECT_NE(analysis.pass_plan.receiver_id, original_receiver);
}

TEST_F(PassPlanTest, ReleasesPlanOutsideInplay)
{
  compute();
  ASSERT_EQ(analysis.pass_plan.state, analysis.pass_plan.STATE_PLANNING);
  msg.play_situation.command.value = crane_msgs::msg::PlaySituation::STOP;
  compute();
  EXPECT_EQ(analysis.pass_plan.state, analysis.pass_plan.STATE_INACTIVE);
  EXPECT_EQ(analysis.pass_plan.receiver_id, -1);
}

TEST_F(PassPlanTest, FreezesReceiverDespiteDelayedKickDetectionAndChangedAttacker)
{
  compute();
  const auto before = analysis.pass_plan;
  ASSERT_EQ(before.state, before.STATE_PLANNING);
  startBall(before);
  analysis.recommended_attacker_id = before.receiver_id;
  compute();
  ASSERT_EQ(analysis.pass_plan.state, before.STATE_BALL_IN_FLIGHT);
  crane_msgs::msg::Kick kick;
  kick.kicker_id = before.kicker_id;
  kick.is_kicker_friend = true;
  kick.direction = 0;  // 検出直後の未確定方向
  analysis.ongoing_kick.push_back(kick);
  compute();
  EXPECT_EQ(analysis.pass_plan.state, before.STATE_BALL_IN_FLIGHT);
  EXPECT_EQ(analysis.pass_plan.plan_id, before.plan_id);
  EXPECT_EQ(analysis.pass_plan.receiver_id, before.receiver_id);
  EXPECT_EQ(analysis.pass_plan.receive_point, before.receive_point);
  analysis.ongoing_kick.front().is_kicker_friend = false;
  compute();
  EXPECT_EQ(analysis.pass_plan.state, before.STATE_INACTIVE);
}

// 出し手の推薦が入れ替わっても、計画を保持している間は出し手を差し替えない。
//
// AttackerMetric のスコアは `10.0 / 到達距離` に二値条件の乗算が掛かる構造で、
// 実測では推薦が 1.0〜1.6 秒ごとに入れ替わる。そのたびに出し手を差し替えると
// plan_id が変わり受領点が跳ぶ（実測: PLANNING 継続 1.49 秒の途中で受け手が
// 1→10、受領点が 3.0m 移動）。受け手は先回りする先を決められない。
// 計画は契約なので、出し手がボールを保持している限り維持する。
TEST_F(PassPlanTest, KeepsKickerWhileHoldingPlanDespiteAttackerRecommendationChange)
{
  compute();
  const auto before = analysis.pass_plan;
  ASSERT_EQ(before.state, before.STATE_PLANNING);
  ASSERT_EQ(before.kicker_id, 1);

  // 推薦だけが別のロボットへ移る。ボールの位置は変えないので、
  // 出し手（ID 1、原点＝ボール上）が最もボールに近いまま。
  analysis.recommended_attacker_id = before.receiver_id;
  compute();
  EXPECT_EQ(analysis.pass_plan.kicker_id, before.kicker_id);
  EXPECT_EQ(analysis.pass_plan.plan_id, before.plan_id);
  EXPECT_EQ(analysis.pass_plan.receive_point, before.receive_point);
}

// ボールを手放したら推薦に追従する。保持は「近いままである限り」の条件付き。
TEST_F(PassPlanTest, FollowsAttackerRecommendationOnceKickerLosesBall)
{
  compute();
  const auto before = analysis.pass_plan;
  ASSERT_EQ(before.state, before.STATE_PLANNING);
  ASSERT_EQ(before.kicker_id, 1);

  // 出し手をボールから引き離し、別の味方がボール上に来る。
  for (auto & robot : msg.robot_info_ours) {
    if (robot.id == 1) {
      robot.pose.x = 4.0;
      robot.pose.y = 4.0;
    } else if (robot.id == 2) {
      robot.pose.x = 0.0;
      robot.pose.y = 0.0;
    }
  }
  analysis.recommended_attacker_id = 2;
  compute();
  EXPECT_EQ(analysis.pass_plan.kicker_id, 2);
}

// 出し手がボールを運んでいる間は計画を解除しない。
//
// Attacker はキック前にボールを運んで体勢を整えるので、その間ボール速度は
// すぐ 0.5 m/s を超える。ボールが動いているというだけで解除すると、
// 出し手が蹴る判断をするまさにその瞬間に計画が消え、パスではなくクリアになる。
TEST_F(PassPlanTest, KeepsPlanWhileKickerCarriesMovingBall)
{
  compute();
  const auto before = analysis.pass_plan;
  ASSERT_EQ(before.state, before.STATE_PLANNING);

  // 出し手(robot1, 原点)がボールを持ったまま動いている状況。
  // 受領点の方向とは関係ない向きに動かし、飛行判定には乗らないようにする。
  msg.ball_info.position.x = 0.05;
  msg.ball_info.position.y = 0.05;
  msg.ball_info.velocity.x = -1.5;
  msg.ball_info.velocity.y = 0.0;
  compute();
  EXPECT_EQ(analysis.pass_plan.state, before.STATE_PLANNING);
  EXPECT_EQ(analysis.pass_plan.receiver_id, before.receiver_id);
}

// 相手にキックされたら、ボールが出し手の近くにあっても計画を手放す。
TEST_F(PassPlanTest, ReleasesPlanWhenOpponentKicksNearbyBall)
{
  compute();
  const auto before = analysis.pass_plan;
  ASSERT_EQ(before.state, before.STATE_PLANNING);

  msg.ball_info.position.x = 0.05;
  msg.ball_info.position.y = 0.05;
  msg.ball_info.velocity.x = -1.5;
  msg.ball_info.velocity.y = 0.0;
  crane_msgs::msg::Kick kick;
  kick.kicker_id = 0;
  kick.is_kicker_friend = false;
  analysis.ongoing_kick.push_back(kick);
  compute();
  EXPECT_EQ(analysis.pass_plan.state, before.STATE_INACTIVE);
}

// ボールが出し手から離れていれば、味方のキック扱いでも運搬とはみなさない。
TEST_F(PassPlanTest, ReleasesPlanWhenMovingBallIsAwayFromKicker)
{
  compute();
  const auto before = analysis.pass_plan;
  ASSERT_EQ(before.state, before.STATE_PLANNING);

  // 出し手(robot1)は原点。ボールを 1m 離れた位置で受領点と無関係な向きに動かす。
  msg.ball_info.position.x = -1.0;
  msg.ball_info.position.y = 0.0;
  msg.ball_info.velocity.x = -1.5;
  msg.ball_info.velocity.y = 0.0;
  compute();
  EXPECT_EQ(analysis.pass_plan.state, before.STATE_INACTIVE);
}

TEST_F(PassPlanTest, RejectsUnavailableReceiverWithoutWaitingForRecompute)
{
  compute();
  const int receiver = analysis.pass_plan.receiver_id;
  ASSERT_GE(receiver, 0);
  metric.setRecomputeInterval(100.0);
  for (auto & robot : msg.robot_info_ours) {
    if (robot.id == receiver) {
      robot.available_vision = false;
      robot.available_feedback = false;
    }
  }
  compute();
  EXPECT_NE(analysis.pass_plan.receiver_id, receiver);
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
