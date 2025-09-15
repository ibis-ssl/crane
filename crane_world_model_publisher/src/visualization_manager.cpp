// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/visualization_manager.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <sstream>

namespace crane
{
struct SvgRobotBuilder
{
  SvgRobotBuilder() : corner_angle(std::acos(center_to_dribbler / radius)) {}

  std::string getSvgString() const
  {
    std::ostringstream path;
    path << "<path d=\"";
    path << "M " << (robot_position.x() + botRightX(theta)) * 1000. << " "
         << (robot_position.y() + botRightY(theta)) * -1000.;
    path << " A " << radius * 1000. << " " << radius * 1000. << " 0 1 0 "
         << (robot_position.x() + botLeftX(theta)) * 1000. << " "
         << (robot_position.y() + botLeftY(theta)) * -1000.;
    path << " Z";
    path << "\" fill=\"" << fill_color << "\" fill-opacity=\"" << fill_opacity;
    path << "\" stroke=\"" << stroke_color << "\" stroke-opacity=\"" << stroke_opacity;
    path << "\" stroke-width=\"" << stroke_width << "\"/>";
    return path.str();
  }

  SvgRobotBuilder & position(Point p, double theta)
  {
    this->robot_position = p;
    this->theta = theta;
    return *this;
  }

  SvgRobotBuilder & position(double x, double y, double theta)
  {
    return position(Point(x, y), theta);
  }

  SvgRobotBuilder & fill(const std::string & color, double opacity = 1.0)
  {
    fill_color = color;
    fill_opacity = opacity;
    return *this;
  }

  SvgRobotBuilder & stroke(const std::string & color, double opacity = 1.0)
  {
    stroke_color = color;
    stroke_opacity = opacity;
    return *this;
  }

  SvgRobotBuilder & strokeWidth(double width)
  {
    stroke_width = width;
    return *this;
  }

private:
  Point robot_position;
  double theta = 0.0;
  std::string fill_color = "none";
  double fill_opacity = 1.0;
  std::string stroke_color = "black";
  double stroke_opacity = 1.0;
  double stroke_width = 1.0;

  double botRightX(double orientation) const
  {
    return radius * std::cos(orientation + corner_angle);
  }
  double botRightY(double orientation) const
  {
    return radius * std::sin(orientation + corner_angle);
  }
  double botLeftX(double orientation) const
  {
    return radius * std::cos(orientation - corner_angle);
  }
  double botLeftY(double orientation) const
  {
    return radius * std::sin(orientation - corner_angle);
  }
  const double radius = 0.085;
  const double center_to_dribbler = 0.055;
  const double corner_angle;
};

VisualizationManager::VisualizationManager(rclcpp::Node & node) : node_(node)
{
  geometry_builder = std::make_shared<crane::VisualizerMessageBuilder>("world_model/geometry");
  vision_builder = std::make_shared<crane::VisualizerMessageBuilder>("world_model/vision");
  tracked_builder = std::make_shared<crane::VisualizerMessageBuilder>("world_model/tracked");
  referee_builder = std::make_shared<crane::VisualizerMessageBuilder>("world_model/referee");
  trajectory_builder = std::make_shared<crane::VisualizerMessageBuilder>("world_model/trajectory");
  slack_builder = std::make_shared<crane::VisualizerMessageBuilder>("world_model/slack");
  pass_score_builder = std::make_shared<crane::VisualizerMessageBuilder>("world_model/pass_score");
  debug_builder = std::make_shared<crane::VisualizerMessageBuilder>("world_model/debug");
  performance_builder =
    std::make_shared<crane::VisualizerMessageBuilder>("world_model/performance");
  performance_builder = std::make_shared<crane::VisualizerMessageBuilder>("world_model/kick_event");

  crane::CraneVisualizerBuffer::activate(node_);

  RCLCPP_INFO(node_.get_logger(), "VisualizationManager initialized with direct builders");
}

auto VisualizationManager::visualizeGeometry(
  const SSL_GeometryData & geometry_data, bool half_court_mode) -> void
{
  drawFieldGeometry(geometry_data, half_court_mode);
}

auto VisualizationManager::visualizeDetection(
  const SSL_DetectionFrame & detection, bool half_court_mode) -> void
{
  drawVisionDetections(detection, half_court_mode);
}

auto VisualizationManager::visualizeTrackedData(const WorldModelWrapper::SharedPtr & world_model)
  -> void
{
  drawTrackedObjects(world_model);
}

auto VisualizationManager::visualizeReferee(
  const robocup_ssl_msgs::msg::Referee & msg, double field_width, double field_height) -> void
{
  drawRefereeInfo(msg, field_width, field_height);
}

auto VisualizationManager::visualizeTrajectories(const WorldModelWrapper::SharedPtr & world_model)
  -> void
{
  drawRobotTrajectories(world_model);
  drawBallTrajectory(world_model);
}

auto VisualizationManager::visualizeSlackAnalysis(const WorldModelWrapper::SharedPtr & world_model)
  -> void
{
  drawSlackTimes(world_model);
}

auto VisualizationManager::visualizePassScoring(const WorldModelWrapper::SharedPtr & world_model)
  -> void
{
  // パススコア可視化の実装（将来的に実装）
  // 実装予定: パス評価結果の描画
}

auto VisualizationManager::visualizeDebugInfo(
  const std::string & category, const std::string & info) -> void
{
  debug_builder->text()
    .text(category + ": " + info)
    .position(0, 0)
    .fontSize(12)
    .fill("white")
    .build();
  debug_builder->flush();
}

auto VisualizationManager::visualizePerformanceMetrics(
  const std::string & component, double processing_time_ms) -> void
{
  std::string text = component + ": " + std::to_string(processing_time_ms) + "ms";
  performance_builder->text().text(text).position(0, 0).fontSize(10).fill("yellow").build();
  performance_builder->flush();
}

// Private methods implementation
auto VisualizationManager::drawFieldGeometry(
  const SSL_GeometryData & geometry_data, bool half_court_mode) -> void
{
  if (!geometry_data.has_field()) {
    return;
  }

  const auto & field = geometry_data.field();
  double field_width = field.field_width() / 1000.0;    // mm to m
  double field_height = field.field_length() / 1000.0;  // mm to m

  // フィールドラインの描画
  geometry_builder->line()
    .start(-field_height / 2, -field_width / 2)
    .end(field_height / 2, -field_width / 2)
    .stroke("white")
    .strokeWidth(10)
    .build();
  geometry_builder->line()
    .start(-field_height / 2, field_width / 2)
    .end(field_height / 2, field_width / 2)
    .stroke("white")
    .strokeWidth(10)
    .build();
  geometry_builder->line()
    .start(-field_height / 2, -field_width / 2)
    .end(-field_height / 2, field_width / 2)
    .stroke("white")
    .strokeWidth(10)
    .build();
  geometry_builder->line()
    .start(field_height / 2, -field_width / 2)
    .end(field_height / 2, field_width / 2)
    .stroke("white")
    .strokeWidth(10)
    .build();

  // センターライン
  geometry_builder->line()
    .start(0, -field_width / 2)
    .end(0, field_width / 2)
    .stroke("white")
    .strokeWidth(10)
    .build();

  // センターサークル
  geometry_builder->circle().center(0, 0).radius(0.5).stroke("white").strokeWidth(10).build();

  // ゴールエリアとペナルティエリアの描画
  if (field.has_goal_width() && field.has_goal_depth()) {
    double goal_width = field.goal_width() / 1000.0;
    double goal_depth = field.goal_depth() / 1000.0;

    // ゴール描画（U字型構造）
    // 左ゴール（後方の壁）
    geometry_builder->line()
      .start(-field_height / 2 - goal_depth, -goal_width / 2)
      .end(-field_height / 2 - goal_depth, goal_width / 2)
      .stroke("white")
      .strokeWidth(10)
      .build();
    // 左ゴール（左側の壁）
    geometry_builder->line()
      .start(-field_height / 2, -goal_width / 2)
      .end(-field_height / 2 - goal_depth, -goal_width / 2)
      .stroke("white")
      .strokeWidth(10)
      .build();
    // 左ゴール（右側の壁）
    geometry_builder->line()
      .start(-field_height / 2, goal_width / 2)
      .end(-field_height / 2 - goal_depth, goal_width / 2)
      .stroke("white")
      .strokeWidth(10)
      .build();

    // 右ゴール（後方の壁）
    geometry_builder->line()
      .start(field_height / 2 + goal_depth, -goal_width / 2)
      .end(field_height / 2 + goal_depth, goal_width / 2)
      .stroke("white")
      .strokeWidth(10)
      .build();
    // 右ゴール（左側の壁）
    geometry_builder->line()
      .start(field_height / 2, -goal_width / 2)
      .end(field_height / 2 + goal_depth, -goal_width / 2)
      .stroke("white")
      .strokeWidth(10)
      .build();
    // 右ゴール（右側の壁）
    geometry_builder->line()
      .start(field_height / 2, goal_width / 2)
      .end(field_height / 2 + goal_depth, goal_width / 2)
      .stroke("white")
      .strokeWidth(10)
      .build();
  }

  // ペナルティエリアの描画
  for (const auto & line : field.field_lines()) {
    if (line.name() == "LeftPenaltyStretch" || line.name() == "RightPenaltyStretch") {
      // ペナルティエリア横線（ゴールライン平行）
      double p1_x = line.p1().x() / 1000.0;
      double p1_y = line.p1().y() / 1000.0;
      double p2_x = line.p2().x() / 1000.0;
      double p2_y = line.p2().y() / 1000.0;
      geometry_builder->line()
        .start(p1_x, p1_y)
        .end(p2_x, p2_y)
        .stroke("white")
        .strokeWidth(10)
        .build();
    } else if (
      line.name() == "LeftFieldLeftPenaltyStretch" ||
      line.name() == "LeftFieldRightPenaltyStretch" ||
      line.name() == "RightFieldLeftPenaltyStretch" ||
      line.name() == "RightFieldRightPenaltyStretch") {
      // ペナルティエリア縦線（サイドライン平行）
      double p1_x = line.p1().x() / 1000.0;
      double p1_y = line.p1().y() / 1000.0;
      double p2_x = line.p2().x() / 1000.0;
      double p2_y = line.p2().y() / 1000.0;
      geometry_builder->line()
        .start(p1_x, p1_y)
        .end(p2_x, p2_y)
        .stroke("white")
        .strokeWidth(10)
        .build();
    }
  }

  // フィールドライン詳細（コーナーアーク、ペナルティマーク等）
  for (const auto & arc : field.field_arcs()) {
    double center_x = arc.center().x() / 1000.0;
    double center_y = arc.center().y() / 1000.0;
    double radius = arc.radius() / 1000.0;
    geometry_builder->circle()
      .center(center_x, center_y)
      .radius(radius)
      .stroke("white")
      .strokeWidth(10)
      .build();
  }

  geometry_builder->flush();
}

auto VisualizationManager::drawVisionDetections(
  const SSL_DetectionFrame & detection, bool half_court_mode) -> void
{
  // ロボット検出の描画（青チーム）
  for (const auto & robot : detection.robots_blue()) {
    double x = robot.x() / 1000.0;
    double y = robot.y() / 1000.0;
    double theta = robot.orientation();

    // ロボット本体（SvgRobotBuilderを使用）
    SvgRobotBuilder robot_shape;
    robot_shape.position(x, y, theta).fill("white", 0.0).stroke("white", 1.0).strokeWidth(20);
    vision_builder->add(robot_shape.getSvgString());

    // ロボットID
    if (robot.has_robot_id()) {
      vision_builder->text()
        .text(std::to_string(robot.robot_id()))
        .position(x, y + 0.15)
        .fontSize(50)
        .fill("white")
        .build();
    }
  }

  // ロボット検出の描画（黄チーム）
  for (const auto & robot : detection.robots_yellow()) {
    double x = robot.x() / 1000.0;
    double y = robot.y() / 1000.0;
    double theta = robot.orientation();

    // ロボット本体
    SvgRobotBuilder robot_shape;
    robot_shape.position(x, y, theta).fill("white", 0.0).stroke("white", 1.0).strokeWidth(20);
    vision_builder->add(robot_shape.getSvgString());

    // ロボットID
    if (robot.has_robot_id()) {
      vision_builder->text()
        .text(std::to_string(robot.robot_id()))
        .position(x, y + 0.15)
        .fontSize(50)
        .fill("white")
        .build();
    }
  }

  vision_builder->flush();
}

auto VisualizationManager::drawTrackedObjects(const WorldModelWrapper::SharedPtr & world_model)
  -> void
{
  // トラッキング済みボール（強調表示 + 回転スコープ）
  const auto ball = world_model->ball();
  const double ball_radius = 0.043;  // SSL official ball radius
  tracked_builder->circle()
    .center(ball.pos.x(), ball.pos.y())
    .radius(ball_radius)
    .stroke("orange")
    .strokeWidth(5)
    .fill("orange", 1.0)
    .build();
  // ベースの円（オレンジ枠 + 薄い塗り）
  tracked_builder->circle()
    .center(ball.pos.x(), ball.pos.y())
    .radius(0.5)
    .stroke("orange")
    .strokeWidth(5)
    .fill("orange", 0.15)
    .build();
  // 回転するスコープ飾り（クロスヘア）

  if (world_model->ball().detected) {
    const double t = node_.now().seconds();
    const double omega = 2.0;  // rad/s
    const double base_angle = std::fmod(t * omega, 2.0 * M_PI);
    auto add_tick = [&](
                      double angle, double r1, double r2, const std::string & color, double width,
                      double alpha) {
      const double dx1 = std::cos(angle) * r1;
      const double dy1 = std::sin(angle) * r1;
      const double dx2 = std::cos(angle) * r2;
      const double dy2 = std::sin(angle) * r2;
      tracked_builder->line()
        .start(ball.pos.x() + dx1, ball.pos.y() + dy1)
        .end(ball.pos.x() + dx2, ball.pos.y() + dy2)
        .stroke(color, alpha)
        .strokeWidth(width)
        .build();
    };

    // 十字のティック（長め）
    for (int i = 0; i < 4; ++i) {
      const double ang = base_angle + (i * M_PI_2);
      add_tick(ang, 0.45, 0.55, "orange", 10, 0.9);
    }
  }
  // 速度ベクトル
  if (ball.vel.norm() > 0.1) {                  // 0.1 m/s 以上で表示
    Point vel_end = ball.pos + ball.vel * 0.5;  // 0.5秒後の位置
    tracked_builder->line()
      .start(ball.pos.x(), ball.pos.y())
      .end(vel_end.x(), vel_end.y())
      .stroke("orange")
      .strokeWidth(12)
      .build();
  }

  // トラッキング済みロボット（味方）
  for (const auto & robot : world_model->ours().getAvailableRobots()) {
    const Point & pos = robot->pose.pos;

    // ロボット本体（SvgRobotBuilderを使用、トラッキング済みは太い線）
    SvgRobotBuilder robot_shape;
    robot_shape.position(pos.x(), pos.y(), robot->pose.theta)
      .fill("green", 1.0)
      .stroke("black", 1.0)
      .strokeWidth(10);
    tracked_builder->add(robot_shape.getSvgString());

    tracked_builder->text()
      .text(std::to_string(robot->id))
      .position(robot->pose.pos.x() - 0.05, robot->pose.pos.y() - 0.05)
      .fontSize(150)
      .fill("white")
      .build();

    // 速度ベクトル
    if (robot->vel.linear.norm() > 0.1) {
      double vel_end_x = pos.x() + robot->vel.linear.x() * 0.3;
      double vel_end_y = pos.y() + robot->vel.linear.y() * 0.3;
      tracked_builder->line()
        .start(pos.x(), pos.y())
        .end(vel_end_x, vel_end_y)
        .stroke("green")
        .strokeWidth(1)
        .build();
    }
  }

  // トラッキング済みロボット（敵）
  for (const auto & robot : world_model->theirs().getAvailableRobots()) {
    const Point & pos = robot->pose.pos;

    // ロボット本体（SvgRobotBuilderを使用）
    SvgRobotBuilder robot_shape;
    robot_shape.position(pos.x(), pos.y(), robot->pose.theta)
      .fill("red", 1.0)
      .stroke("black", 1.0)
      .strokeWidth(10);
    tracked_builder->add(robot_shape.getSvgString());

    tracked_builder->text()
      .text(std::to_string(robot->getID().id))
      .position(robot->pose.pos.x() - 0.05, robot->pose.pos.y() - 0.05)
      .fontSize(150)
      .fill("white")
      .build();
  }

  tracked_builder->flush();
}

auto VisualizationManager::drawRefereeInfo(
  const robocup_ssl_msgs::msg::Referee & msg, double field_width, double field_height) -> void
{
  // レフェリー状態の表示
  std::string command_name = "UNKNOWN";
  switch (msg.command) {
    case robocup_ssl_msgs::msg::Referee::COMMAND_HALT:
      command_name = "HALT";
      break;
    case robocup_ssl_msgs::msg::Referee::COMMAND_STOP:
      command_name = "STOP";
      break;
    case robocup_ssl_msgs::msg::Referee::COMMAND_NORMAL_START:
      command_name = "NORMAL_START";
      break;
    case robocup_ssl_msgs::msg::Referee::COMMAND_FORCE_START:
      command_name = "FORCE_START";
      break;
    case robocup_ssl_msgs::msg::Referee::COMMAND_PREPARE_KICKOFF_YELLOW:
      command_name = "PREPARE_KICKOFF_YELLOW";
      break;
    case robocup_ssl_msgs::msg::Referee::COMMAND_PREPARE_KICKOFF_BLUE:
      command_name = "PREPARE_KICKOFF_BLUE";
      break;
    case robocup_ssl_msgs::msg::Referee::COMMAND_PREPARE_PENALTY_YELLOW:
      command_name = "PREPARE_PENALTY_YELLOW";
      break;
    case robocup_ssl_msgs::msg::Referee::COMMAND_PREPARE_PENALTY_BLUE:
      command_name = "PREPARE_PENALTY_BLUE";
      break;
    case robocup_ssl_msgs::msg::Referee::COMMAND_DIRECT_FREE_YELLOW:
      command_name = "DIRECT_FREE_YELLOW";
      break;
    case robocup_ssl_msgs::msg::Referee::COMMAND_DIRECT_FREE_BLUE:
      command_name = "DIRECT_FREE_BLUE";
      break;
    default:
      command_name = "COMMAND_" + std::to_string(msg.command);
      break;
  }

  // レフェリー情報をフィールド上部に表示
  referee_builder->text()
    .text("Referee: " + command_name)
    .position(-field_height / 2, field_width / 2 + 0.5)
    .fontSize(100)
    .fill("white")
    .build();

  // ボール位置（指定されている場合）
  if (!msg.designated_position.empty()) {
    double ball_x = msg.designated_position.front().x / 1000.0;
    double ball_y = msg.designated_position.front().y / 1000.0;
    referee_builder->circle()
      .center(ball_x, ball_y)
      .radius(0.1)
      .stroke("white")
      .strokeWidth(2)
      .build();
    referee_builder->text()
      .text("Ball")
      .position(ball_x, ball_y + 0.2)
      .fontSize(12)
      .fill("white")
      .build();
  }

  referee_builder->flush();
}

auto VisualizationManager::drawRobotTrajectories(const WorldModelWrapper::SharedPtr & world_model)
  -> void
{
  // 実装予定: ロボット軌跡の描画
  // WorldModelPublisherComponentからの移行対象
}

auto VisualizationManager::drawBallTrajectory(const WorldModelWrapper::SharedPtr & world_model)
  -> void
{
  // 実装予定: ボール軌跡の描画
  // WorldModelPublisherComponentからの移行対象
}

auto VisualizationManager::drawSlackTimes(const WorldModelWrapper::SharedPtr & world_model) -> void
{
  // 実装予定: スラック時間分析結果の描画
  // WorldModelPublisherComponentのpostProcessWorldModelからの移行対象
}

auto VisualizationManager::visualizeTrajectoryHistory(const TrajectoryHistoryData & trajectory_data)
  -> void
{
  static constexpr int SAMPLING_NUM = 4;

  // 味方ロボットの履歴描画
  for (const auto & [robot_id, history] :
       trajectory_data.friend_history | ranges::views::enumerate) {
    if (history.size() > SAMPLING_NUM + 1 && history.front().detected) {
      for (int i = 0; i < 10; i++) {
        int start = static_cast<int>((history.size() / 10.) * i);
        int end = static_cast<int>((history.size() / 10.) * (i + 1));

        auto polyline_builder = trajectory_builder->polyline();
        for (int index = start; index < end; index += SAMPLING_NUM) {
          polyline_builder.addPoint(history.at(index).pose.x, history.at(index).pose.y);
        }
        if (i != 9) {
          polyline_builder.addPoint(history.at(end).pose.x, history.at(end).pose.y);
        }
        polyline_builder.stroke("green", start / static_cast<double>(history.size()))
          .strokeWidth(15)
          .build();
      }
    }
  }

  // 敵ロボットの履歴描画
  for (const auto & [robot_id, history] :
       trajectory_data.enemy_history | ranges::views::enumerate) {
    if (history.size() > SAMPLING_NUM + 1 && history.front().detected) {
      for (int i = 0; i < 10; i++) {
        int start = static_cast<int>((history.size() / 10.) * i);
        int end = static_cast<int>((history.size() / 10.) * (i + 1));

        auto polyline_builder = trajectory_builder->polyline();
        for (int index = start; index < end; index += SAMPLING_NUM) {
          polyline_builder.addPoint(history.at(index).pose.x, history.at(index).pose.y);
        }
        if (i != 9) {
          polyline_builder.addPoint(history.at(end).pose.x, history.at(end).pose.y);
        }
        polyline_builder.stroke("red", start / static_cast<double>(history.size()))
          .strokeWidth(15)
          .build();
      }
    }
  }

  // ボール軌跡描画
  if (trajectory_data.ball_info_history.size() > SAMPLING_NUM + 1) {
    for (int i = 0; i < 10; i++) {
      int start = static_cast<int>((trajectory_data.ball_info_history.size() / 10.) * i);
      int end = static_cast<int>((trajectory_data.ball_info_history.size() / 10.) * (i + 1));

      auto polyline_builder = trajectory_builder->polyline();
      for (int index = start; index < end; index += SAMPLING_NUM) {
        polyline_builder.addPoint(
          trajectory_data.ball_info_history.at(index).position.x,
          trajectory_data.ball_info_history.at(index).position.y);
      }
      if (i != 9) {
        polyline_builder.addPoint(
          trajectory_data.ball_info_history.at(end).position.x,
          trajectory_data.ball_info_history.at(end).position.y);
      }
      polyline_builder
        .stroke("orange", start / static_cast<double>(trajectory_data.ball_info_history.size()))
        .strokeWidth(30)
        .build();
    }
  }

  trajectory_builder->flush();
}

}  // namespace crane
