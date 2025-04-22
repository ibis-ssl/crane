// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/formation_planner.hpp>

namespace crane
{
std::vector<Point> FormationPlanner::getWingFormationPoints(int robot_num)
{
  std::vector<Point> formation_points;

  double half_width = world_model->field_size.y() / 2.0 - 1.0;

  // フィールドの横幅いっぱいに広がるようにy座標を計算
  double y_step = 0;
  if (robot_num > 1) {
    y_step = (2 * half_width) / (robot_num - 1);
  }

  // 真ん中のロボットのインデックス
  int middle_index = robot_num / 2;

  // 真ん中のロボットのx座標を0.6にするため、
  // 真ん中のインデックスが偶数か奇数かで、x座標の配置パターンを決定
  bool start_with_x06 = (middle_index % 2 == 0);

  // ロボットごとに位置を設定
  for (int i = 0; i < robot_num; i++) {
    // y座標はフィールド端から端まで均等に分布
    double y = -half_width + i * y_step;

    // x座標を交互に設定（真ん中が0.6になるようにパターンを調整）
    double x;
    if (start_with_x06) {
      // 最初のロボットがx=0.6から始まるパターン
      x = (i % 2 == 0) ? 0.6 : 1.5;
    } else {
      // 最初のロボットがx=1.5から始まるパターン
      x = (i % 2 == 0) ? 1.5 : 0.6;
    }

    formation_points.emplace_back(x, y);
  }

  // フィールドの向きに応じてx座標を反転
  if (world_model->getOurGoalCenter().x() < 0.0) {
    for (auto & point : formation_points) {
      point.x() *= -1.0;
    }
  }

  return formation_points;
}

std::vector<Point> FormationPlanner::getIbisFormationPoints(int robot_num)
{
  std::vector<Point> formation_points;

  double y_offset = 0.3 * (robot_num / 2);
  double x = world_model->field_size.x() / 4.0;

  // iの頭
  formation_points.emplace_back(x, -y_offset);

  for (int i = 1; i < robot_num; i++) {
    formation_points.emplace_back(x, -y_offset + (i + 2) * 0.3);
  }

  // フィールドの向きに応じてx座標を反転
  if (world_model->getOurGoalCenter().x() < 0.0) {
    for (auto & point : formation_points) {
      point.x() *= -1.0;
    }
  }

  return formation_points;
}

std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
FormationPlanner::calculateRobotCommand(
  const std::vector<RobotIdentifier> & robots, PlannerContext &)
{
  std::vector<Point> robot_points;
  for (auto robot_id : robots) {
    robot_points.emplace_back(world_model->getRobot(robot_id)->pose.pos);
  }

  auto formation_points = [&]() {
    switch (formation_type) {
      case FormationType::WING:
        return getWingFormationPoints(robots.size());
      case FormationType::IBIS:
        return getIbisFormationPoints(robots.size());
      default:
        throw std::runtime_error("Unknown formation type");
    }
  }();

  auto solution = getOptimalAssignments(robot_points, formation_points);

  double target_theta = (world_model->getOurGoalCenter().x() > 0.0) ? M_PI : 0.0;
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  for (auto robot_id = robots.begin(); robot_id != robots.end(); ++robot_id) {
    int index = std::distance(robots.begin(), robot_id);
    Point target_point = formation_points[solution[index]];

    auto command =
      std::make_shared<crane::RobotCommandWrapper>("formation_planner", robot_id->id, world_model);

    command->setTargetPosition(target_point);
    command->setTargetTheta(target_theta);
    command->setMaxVelocity(1.0);

    robot_commands.emplace_back(command->getMsg());
  }
  return {PlannerBase::Status::RUNNING, robot_commands};
}

auto FormationPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
  -> std::vector<uint8_t>
{
  return this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      // choose id smaller first
      return 15. - static_cast<double>(-robot->id);
    },
    prev_roles, context);
}
}  // namespace crane
