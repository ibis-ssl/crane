// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_geometry/geometry_operations.hpp>
#include <crane_robot_skills/marker.hpp>

namespace crane::skills
{
void Marker::initialize()
{
  setParameter("marking_robot_id", 0);
  setParameter("mark_distance", 0.5);
  setParameter("max_mark_distance", 1.5);
  setParameter("mark_mode", std::string("save_goal"));
}

Status Marker::update()
{
  auto marked_robot = world_model()->getTheirRobot(getParameter<int>("marking_robot_id"));
  if (!marked_robot->available()) {
    // マーク対象が存在しないためペナルティエリア正面中央（守備ライン上）で待機
    constexpr double OFFSET_X = 0.2;
    constexpr double OFFSET_Y = 0.2;
    auto [p1, p2, p3, p4] = world_model()->getPenaltyAreaCorners(OFFSET_X, OFFSET_Y);
    command->setTargetPosition((p2 + p3) / 2.0, 0.1).lookAtBall();
    return Status::RUNNING;
  }
  auto enemy_pos = marked_robot->pose.pos;
  std::string mode = getParameter<std::string>("mark_mode");
  double mark_distance = getParameter<double>("mark_distance");
  double max_mark_distance = getParameter<double>("max_mark_distance");

  if (mode == "penalty_area") {
    // 守備ライン（ペナルティエリア外周+オフセット）上の守備位置に配置
    constexpr double OFFSET_X = 0.2;
    constexpr double OFFSET_Y = 0.2;

    Segment enemy_to_goal{enemy_pos, world_model()->getOurGoalCenter()};
    auto [p1, p2, p3, p4] = world_model()->getPenaltyAreaCorners(OFFSET_X, OFFSET_Y);
    Segment seg1{p1, p2}, seg2{p2, p3}, seg3{p3, p4};

    // 敵→ゴール線と守備ラインの交点を求める
    std::optional<Point> target_on_line_opt;
    for (const auto & seg : {seg1, seg2, seg3}) {
      auto intersections = getIntersections(seg, enemy_to_goal);
      if (!intersections.empty()) {
        target_on_line_opt = intersections[0];
        break;
      }
    }

    if (target_on_line_opt) {
      Point target_on_line = *target_on_line_opt;

      // 守備ラインの最近傍点を求める（3セグメント中で最短）
      auto cp1 = getClosestPointAndDistance(robot()->pose.pos, seg1);
      auto cp2 = getClosestPointAndDistance(robot()->pose.pos, seg2);
      auto cp3 = getClosestPointAndDistance(robot()->pose.pos, seg3);

      Point nearest_on_line = cp1.closest_point;
      double min_dist = cp1.distance;
      if (cp2.distance < min_dist) {
        nearest_on_line = cp2.closest_point;
        min_dist = cp2.distance;
      }
      if (cp3.distance < min_dist) {
        nearest_on_line = cp3.closest_point;
        min_dist = cp3.distance;
      }

      // ラインから離れているときは最近傍点優先、近いときはtarget_on_line優先
      double pull_ratio = std::clamp(1.0 - min_dist * 2.0, 0.0, 1.0);
      Point target = nearest_on_line * (1.0 - pull_ratio) + target_on_line * pull_ratio;
      command->setTargetPosition(target, 0.1).lookAtBall();
      return Status::RUNNING;
    }
    // 交点なし: save_goalモードにフォールバック
    mode = "save_goal";
  }

  // 基準点（ゴール中心 or ボール位置）
  Point reference;
  if (mode == "save_goal") {
    reference = world_model()->getOurGoalCenter();
  } else if (mode == "intercept_pass") {
    reference = world_model()->ball().pos;
  } else {
    throw std::runtime_error("unknown mark mode");
  }

  // マーキングセグメントを定義: 敵からmark_distance〜max_mark_distanceの範囲
  auto dir = (reference - enemy_pos).normalized();
  Point near_end = enemy_pos + dir * mark_distance;     // 敵に近い端（理想位置）
  Point far_end = enemy_pos + dir * max_mark_distance;  // 遠い端
  Segment marking_segment{near_end, far_end};

  // 自ロボットからセグメントへの最近傍点と距離
  auto cp = getClosestPointAndDistance(robot()->pose.pos, marking_segment);

  // セグメントから離れているときは最近傍点を優先、近いときはnear_end（敵寄り）を優先
  double pull_ratio = std::clamp(1.0 - cp.distance * 2.0, 0.0, 1.0);
  Point target = cp.closest_point * (1.0 - pull_ratio) + near_end * pull_ratio;

  command->setTargetPosition(target, 0.1).lookAtBall();
  return Status::RUNNING;
}
}  // namespace crane::skills
