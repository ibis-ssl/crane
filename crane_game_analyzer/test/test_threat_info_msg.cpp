// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_game_analyzer/threat_evaluator.hpp>
#include <memory>

namespace crane
{
namespace
{
auto makeRobot() -> std::shared_ptr<RobotInfo>
{
  auto robot = std::make_shared<RobotInfo>();
  robot->id = 7;
  robot->pose.pos = Point(-2.0, 0.5);
  robot->vel.linear = Point(0.3, -0.4);
  return robot;
}
}  // namespace

TEST(ThreatInfoMsgTest, BallThreatWithProtectionLine)
{
  BallThreat threat;
  threat.source_type = BallThreat::SourceType::PASS_RECEIVE;
  threat.source_position = Point(1.0, 2.0);
  threat.velocity = Vector2(3.0, 4.0);
  threat.threat_line = Segment{Point(1.0, 2.0), Point(-6.0, 0.0)};
  threat.protection_line = Segment{Point(-4.5, 0.2), Point(-4.5, 0.8)};

  const auto msg = ThreatEvaluator{}.toThreatInfoMsg(threat);

  EXPECT_EQ(msg.threat_type, crane_msgs::msg::ThreatInfo::THREAT_TYPE_BALL);
  EXPECT_EQ(msg.source_type, crane_msgs::msg::ThreatInfo::SOURCE_TYPE_PASS_RECEIVE);
  EXPECT_DOUBLE_EQ(msg.source_position.x, 1.0);
  EXPECT_DOUBLE_EQ(msg.source_position.y, 2.0);
  EXPECT_DOUBLE_EQ(msg.threat_line_start.x, 1.0);
  EXPECT_DOUBLE_EQ(msg.threat_line_start.y, 2.0);
  EXPECT_DOUBLE_EQ(msg.threat_line_end.x, -6.0);
  EXPECT_DOUBLE_EQ(msg.threat_line_end.y, 0.0);
  EXPECT_TRUE(msg.has_protection_line);
  EXPECT_DOUBLE_EQ(msg.protection_line_start.x, -4.5);
  EXPECT_DOUBLE_EQ(msg.protection_line_start.y, 0.2);
  EXPECT_DOUBLE_EQ(msg.protection_line_end.x, -4.5);
  EXPECT_DOUBLE_EQ(msg.protection_line_end.y, 0.8);
  EXPECT_DOUBLE_EQ(msg.velocity.x, 3.0);
  EXPECT_DOUBLE_EQ(msg.velocity.y, 4.0);
}

TEST(ThreatInfoMsgTest, BallThreatWithoutProtectionLine)
{
  BallThreat threat;
  threat.threat_line = Segment{Point(1.0, 2.0), Point(-6.0, 0.0)};

  const auto msg = ThreatEvaluator{}.toThreatInfoMsg(threat);

  EXPECT_FALSE(msg.has_protection_line);
  EXPECT_EQ(msg.source_type, crane_msgs::msg::ThreatInfo::SOURCE_TYPE_BALL);
}

TEST(ThreatInfoMsgTest, RobotThreatWithProtectionLine)
{
  RobotThreat threat;
  threat.robot = makeRobot();
  threat.threat_line = Segment{Point(-2.0, 0.5), Point(-6.0, 0.0)};
  threat.protection_line = Segment{Point(-4.8, 0.1), Point(-4.2, 0.3)};
  threat.threat_rating = 0.75;
  threat.rating_detail.score_redirect_angle = 0.1;
  threat.rating_detail.score_pen_area_border = 0.2;
  threat.rating_detail.score_facing_goal = 0.3;
  threat.rating_detail.score_ball_access = 0.4;

  const auto msg = ThreatEvaluator{}.toThreatInfoMsg(threat);

  EXPECT_EQ(msg.threat_type, crane_msgs::msg::ThreatInfo::THREAT_TYPE_ROBOT);
  EXPECT_EQ(msg.source_robot_id, 7);
  // ロボット脅威の起点はロボットの位置（BallThreat は source_position）
  EXPECT_DOUBLE_EQ(msg.source_position.x, -2.0);
  EXPECT_DOUBLE_EQ(msg.source_position.y, 0.5);
  EXPECT_DOUBLE_EQ(msg.threat_line_start.x, -2.0);
  EXPECT_DOUBLE_EQ(msg.threat_line_start.y, 0.5);
  EXPECT_DOUBLE_EQ(msg.threat_line_end.x, -6.0);
  EXPECT_DOUBLE_EQ(msg.threat_line_end.y, 0.0);
  EXPECT_TRUE(msg.has_protection_line);
  EXPECT_DOUBLE_EQ(msg.protection_line_start.x, -4.8);
  EXPECT_DOUBLE_EQ(msg.protection_line_start.y, 0.1);
  EXPECT_DOUBLE_EQ(msg.protection_line_end.x, -4.2);
  EXPECT_DOUBLE_EQ(msg.protection_line_end.y, 0.3);
  EXPECT_DOUBLE_EQ(msg.velocity.x, 0.3);
  EXPECT_DOUBLE_EQ(msg.velocity.y, -0.4);
  EXPECT_FLOAT_EQ(msg.threat_rating, 0.75F);
  EXPECT_FLOAT_EQ(msg.score_redirect_angle, 0.1F);
  EXPECT_FLOAT_EQ(msg.score_pen_area_border, 0.2F);
  EXPECT_FLOAT_EQ(msg.score_facing_goal, 0.3F);
  EXPECT_FLOAT_EQ(msg.score_ball_access, 0.4F);
}

TEST(ThreatInfoMsgTest, RobotThreatWithoutProtectionLine)
{
  RobotThreat threat;
  threat.robot = makeRobot();
  threat.threat_line = Segment{Point(-2.0, 0.5), Point(-6.0, 0.0)};

  const auto msg = ThreatEvaluator{}.toThreatInfoMsg(threat);

  EXPECT_FALSE(msg.has_protection_line);
}

}  // namespace crane
