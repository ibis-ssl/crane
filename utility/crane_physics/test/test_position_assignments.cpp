// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_physics/position_assignments.hpp>

namespace crane
{
// getOptimalAssignments関数のテスト
class PositionAssignmentsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // テスト用のセットアップ
  }

  PositionAssignmentsTest() = default;
};

TEST_F(PositionAssignmentsTest, EmptyRobotList)
{
  // 空のロボットリストの場合
  std::vector<Point> robot_positions;
  std::vector<Point> targets = {Point(1.0, 1.0), Point(2.0, 2.0)};

  auto result = getOptimalAssignments(robot_positions, targets);

  EXPECT_TRUE(result.empty());
}

TEST_F(PositionAssignmentsTest, SingleRobot)
{
  // ロボットが1台のみの場合（特別処理）
  std::vector<Point> robot_positions = {Point(0.0, 0.0)};
  std::vector<Point> targets = {Point(1.0, 1.0), Point(2.0, 2.0), Point(3.0, 3.0)};

  auto result = getOptimalAssignments(robot_positions, targets);

  ASSERT_EQ(result.size(), 1);
  // 1台のロボットの場合、常にインデックス0が返される
  EXPECT_EQ(result[0], 0);
}

TEST_F(PositionAssignmentsTest, SimpleOptimalAssignment)
{
  // 最適解が明確な単純なケース
  // ロボット0は(0,0)、ロボット1は(2,0)
  // ターゲット0は(0,1)、ターゲット1は(2,1)
  // 最適な割り当ては: ロボット0→ターゲット0、ロボット1→ターゲット1
  std::vector<Point> robot_positions = {Point(0.0, 0.0), Point(2.0, 0.0)};
  std::vector<Point> targets = {Point(0.0, 1.0), Point(2.0, 1.0)};

  auto result = getOptimalAssignments(robot_positions, targets);

  ASSERT_EQ(result.size(), 2);
  EXPECT_EQ(result[0], 0);  // ロボット0はターゲット0に割り当て
  EXPECT_EQ(result[1], 1);  // ロボット1はターゲット1に割り当て
}

TEST_F(PositionAssignmentsTest, CrossedOptimalAssignment)
{
  // 最適解が交差するケース
  // ロボット0は(0,0)、ロボット1は(0,2)
  // ターゲット0は(1,2)、ターゲット1は(1,0)
  // 最適な割り当ては: ロボット0→ターゲット1、ロボット1→ターゲット0（交差）
  std::vector<Point> robot_positions = {Point(0.0, 0.0), Point(0.0, 2.0)};
  std::vector<Point> targets = {Point(1.0, 2.0), Point(1.0, 0.0)};

  auto result = getOptimalAssignments(robot_positions, targets);

  ASSERT_EQ(result.size(), 2);
  EXPECT_EQ(result[0], 1);  // ロボット0はターゲット1に割り当て
  EXPECT_EQ(result[1], 0);  // ロボット1はターゲット0に割り当て
}

TEST_F(PositionAssignmentsTest, MoreTargetsThanRobots)
{
  // ターゲット数がロボット数より多い場合
  std::vector<Point> robot_positions = {Point(0.0, 0.0), Point(1.0, 0.0)};
  std::vector<Point> targets = {Point(0.0, 1.0), Point(1.0, 1.0), Point(2.0, 1.0), Point(3.0, 1.0)};

  auto result = getOptimalAssignments(robot_positions, targets);

  ASSERT_EQ(result.size(), 2);
  // 最も近いターゲットに割り当てられることを確認
  EXPECT_EQ(result[0], 0);  // ロボット0はターゲット0に割り当て（距離1.0）
  EXPECT_EQ(result[1], 1);  // ロボット1はターゲット1に割り当て（距離1.0）
}

TEST_F(PositionAssignmentsTest, ThreeRobotsOptimalAssignment)
{
  // 3台のロボットの最適割り当て
  std::vector<Point> robot_positions = {Point(0.0, 0.0), Point(1.0, 0.0), Point(2.0, 0.0)};
  std::vector<Point> targets = {Point(2.1, 0.0), Point(0.1, 0.0), Point(1.1, 0.0)};

  auto result = getOptimalAssignments(robot_positions, targets);

  ASSERT_EQ(result.size(), 3);
  // 各ロボットは最も近いターゲットに割り当てられる
  EXPECT_EQ(result[0], 1);  // ロボット0はターゲット1に割り当て
  EXPECT_EQ(result[1], 2);  // ロボット1はターゲット2に割り当て
  EXPECT_EQ(result[2], 0);  // ロボット2はターゲット0に割り当て
}

TEST_F(PositionAssignmentsTest, VerifyTotalDistanceIsMinimized)
{
  // 総距離が最小化されることを確認
  std::vector<Point> robot_positions = {Point(0.0, 0.0), Point(10.0, 0.0)};
  std::vector<Point> targets = {Point(9.0, 0.0), Point(1.0, 0.0)};

  auto result = getOptimalAssignments(robot_positions, targets);

  ASSERT_EQ(result.size(), 2);

  // 各ロボットからターゲットまでの距離を計算
  double total_distance = 0.0;
  for (size_t i = 0; i < robot_positions.size(); ++i) {
    total_distance += bg::distance(robot_positions[i], targets[result[i]]);
  }

  // 最適な割り当て: ロボット0→ターゲット1（距離1.0）、ロボット1→ターゲット0（距離1.0）
  // 総距離 = 2.0
  EXPECT_NEAR(total_distance, 2.0, 0.01);

  // 代替の割り当て: ロボット0→ターゲット0（距離9.0）、ロボット1→ターゲット1（距離9.0）
  // 総距離 = 18.0（最適ではない）
}

TEST_F(PositionAssignmentsTest, AllRobotsAtSamePosition)
{
  // すべてのロボットが同じ位置にいる場合
  std::vector<Point> robot_positions = {Point(0.0, 0.0), Point(0.0, 0.0), Point(0.0, 0.0)};
  std::vector<Point> targets = {Point(1.0, 0.0), Point(2.0, 0.0), Point(3.0, 0.0)};

  auto result = getOptimalAssignments(robot_positions, targets);

  ASSERT_EQ(result.size(), 3);
  // すべてのロボットが異なるターゲットに割り当てられることを確認
  std::set<int> unique_targets(result.begin(), result.end());
  EXPECT_EQ(unique_targets.size(), 3);
}

TEST_F(PositionAssignmentsTest, AllTargetsAtSamePosition)
{
  // すべてのターゲットが同じ位置にある場合
  std::vector<Point> robot_positions = {Point(0.0, 0.0), Point(1.0, 0.0)};
  std::vector<Point> targets = {Point(5.0, 5.0), Point(5.0, 5.0), Point(5.0, 5.0)};

  auto result = getOptimalAssignments(robot_positions, targets);

  ASSERT_EQ(result.size(), 2);
  // すべてのロボットが異なるターゲットに割り当てられることを確認
  EXPECT_NE(result[0], result[1]);
}

TEST_F(PositionAssignmentsTest, LargeScaleAssignment)
{
  // 大規模な割り当て（6台のロボット）
  std::vector<Point> robot_positions;
  std::vector<Point> targets;

  // 円周上にロボットを配置
  for (int i = 0; i < 6; ++i) {
    double angle = i * M_PI / 3.0;
    robot_positions.push_back(Point(std::cos(angle), std::sin(angle)));
  }

  // ターゲットは少し回転した位置に配置
  for (int i = 0; i < 6; ++i) {
    double angle = i * M_PI / 3.0 + M_PI / 12.0;
    targets.push_back(Point(std::cos(angle), std::sin(angle)));
  }

  auto result = getOptimalAssignments(robot_positions, targets);

  ASSERT_EQ(result.size(), 6);

  // すべてのロボットが異なるターゲットに割り当てられることを確認
  std::set<int> unique_targets(result.begin(), result.end());
  EXPECT_EQ(unique_targets.size(), 6);

  // 総距離が妥当な範囲にあることを確認
  double total_distance = 0.0;
  for (size_t i = 0; i < robot_positions.size(); ++i) {
    total_distance += bg::distance(robot_positions[i], targets[result[i]]);
  }
  // 各ロボットは隣のターゲットに移動するはずなので、総距離は小さい
  EXPECT_LT(total_distance, 2.0);
}

TEST_F(PositionAssignmentsTest, NegativeCoordinates)
{
  // 負の座標を含むケース
  std::vector<Point> robot_positions = {Point(-2.0, -1.0), Point(-1.0, -2.0)};
  std::vector<Point> targets = {Point(-1.5, -2.5), Point(-2.5, -1.5)};

  auto result = getOptimalAssignments(robot_positions, targets);

  ASSERT_EQ(result.size(), 2);

  // 総距離を計算して最適性を確認
  double total_distance = 0.0;
  for (size_t i = 0; i < robot_positions.size(); ++i) {
    total_distance += bg::distance(robot_positions[i], targets[result[i]]);
  }

  // 最適な割り当てでの総距離を確認
  EXPECT_LT(total_distance, 3.0);
}

}  // namespace crane
