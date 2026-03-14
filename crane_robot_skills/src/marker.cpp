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
  auto enemy_pos = marked_robot->pose.pos;
  std::string mode = getParameter<std::string>("mark_mode");
  double mark_distance = getParameter<double>("mark_distance");
  double max_mark_distance = getParameter<double>("max_mark_distance");

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
