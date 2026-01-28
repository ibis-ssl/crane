// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/visualization_manager.hpp"

#include <algorithm>
#include <cmath>
#include <range/v3/view/enumerate.hpp>
#include <ranges>
#include <sstream>

namespace
{
inline std::string speedToColor(double s)
{
  if (s < 0.2) return "#7f8c8d";  // slow: gray
  if (s < 1.0) return "#00d1ff";  // walking: cyan
  if (s < 2.0) return "#00ff5a";  // jog: lime
  if (s < 3.0) return "#ffd400";  // run: yellow
  return "#ff4d4d";               // sprint: red
}
}  // namespace

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
  placement_builder = std::make_shared<crane::VisualizerMessageBuilder>("world_model/placement");
  slack_builder = std::make_shared<crane::VisualizerMessageBuilder>("world_model/slack");
  pass_score_builder = std::make_shared<crane::VisualizerMessageBuilder>("world_model/pass_score");
  kick_event_builder = std::make_shared<crane::VisualizerMessageBuilder>("world_model/kick_event");

  crane::CraneVisualizerBuffer::activate(node_);

  RCLCPP_INFO(node_.get_logger(), "VisualizationManager initialized with direct builders");
}

// Private methods implementation
auto VisualizationManager::drawFieldGeometry(
  const robocup_ssl::SSL_GeometryData & geometry_data, [[maybe_unused]] bool half_court_mode)
  -> void
{
  if (!geometry_data.has_field()) {
    return;
  }

  const auto & field = geometry_data.field();
  double field_width = field.field_width() / 1000.0;    // mm to m
  double field_height = field.field_length() / 1000.0;  // mm to m

  // フィールドラインの描画（外周）
  geometry_builder->drawFieldRect(
    Point(-field_height / 2, -field_width / 2), Point(field_height / 2, field_width / 2));

  // センターライン
  geometry_builder->drawFieldLine(Point(0, -field_width / 2), Point(0, field_width / 2));

  // センターサークル
  geometry_builder->drawCircle(Point(0, 0), 0.5, "white", 10);

  // ゴールエリアとペナルティエリアの描画
  if (field.has_goal_width() && field.has_goal_depth()) {
    double goal_width = field.goal_width() / 1000.0;
    double goal_depth = field.goal_depth() / 1000.0;

    // ゴール描画（U字型構造）
    geometry_builder->drawGoal(Point(-field_height / 2, 0), goal_width, -goal_depth);  // 左ゴール
    geometry_builder->drawGoal(Point(field_height / 2, 0), goal_width, goal_depth);    // 右ゴール
  }

  // ペナルティエリアの描画
  for (const auto & line : field.field_lines()) {
    if (
      line.name() == "LeftPenaltyStretch" || line.name() == "RightPenaltyStretch" ||
      line.name() == "LeftFieldLeftPenaltyStretch" ||
      line.name() == "LeftFieldRightPenaltyStretch" ||
      line.name() == "RightFieldLeftPenaltyStretch" ||
      line.name() == "RightFieldRightPenaltyStretch") {
      geometry_builder->drawFieldLine(
        Point(line.p1().x() / 1000.0, line.p1().y() / 1000.0),
        Point(line.p2().x() / 1000.0, line.p2().y() / 1000.0));
    }
  }

  // フィールドライン詳細（コーナーアーク、ペナルティマーク等）
  for (const auto & arc : field.field_arcs()) {
    double center_x = arc.center().x() / 1000.0;
    double center_y = arc.center().y() / 1000.0;
    double radius = arc.radius() / 1000.0;
    geometry_builder->drawCircle(Point(center_x, center_y), radius, "white", 10);
  }

  geometry_builder->flush();
}

auto VisualizationManager::drawVisionDetections(
  const robocup_ssl::SSL_DetectionFrame & detection, [[maybe_unused]] bool half_court_mode) -> void
{
  // ロボット検出の描画（青チーム）
  for (const auto & robot : detection.robots_blue()) {
    Point pos(robot.x() / 1000.0, robot.y() / 1000.0);
    double theta = robot.orientation();

    // ロボット本体とID
    if (robot.has_robot_id()) {
      vision_builder->drawRobotWithID(
        pos, theta, robot.robot_id(), "white", 0.0, "white", 1.0, 20, 50, "white", 0.0, 0.15);
    } else {
      vision_builder->drawRobot(pos, theta, "white", 0.0, "white", 1.0, 20);
    }
  }

  // ロボット検出の描画（黄チーム）
  for (const auto & robot : detection.robots_yellow()) {
    Point pos(robot.x() / 1000.0, robot.y() / 1000.0);
    double theta = robot.orientation();

    // ロボット本体とID
    if (robot.has_robot_id()) {
      vision_builder->drawRobotWithID(
        pos, theta, robot.robot_id(), "white", 0.0, "white", 1.0, 20, 50, "white", 0.0, 0.15);
    } else {
      vision_builder->drawRobot(pos, theta, "white", 0.0, "white", 1.0, 20);
    }
  }

  vision_builder->flush();
}

auto VisualizationManager::drawTrackedObjects(const WorldModelWrapper::SharedPtr & world_model)
  -> void
{
  // トラッキング済みボール（強調表示 + 回転スコープ）
  const auto & ball = world_model->ball();
  const double ball_radius = 0.043;  // SSL official ball radius
  tracked_builder->drawStyledCircle(ball.pos, ball_radius, "orange", 1.0, "orange", 1.0, 5);
  // ベースの円（オレンジ枠 + 薄い塗り）
  tracked_builder->drawStyledCircle(ball.pos, 0.5, "orange", 0.15, "orange", 1.0, 5);
  // 回転するスコープ飾り（クロスヘア）

  if (ball.detected) {
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

  auto draw_velocity_marker = [this](
                                const std::shared_ptr<RobotInfo> & robot, int trail_points = 5,
                                double min_speed = 0.05, double length_scale = 0.5,
                                double min_length = 0.10, double max_length = 1.5) {
    const double speed = robot->vel.linear.norm();
    if (speed <= min_speed) return;
    const std::string color = speedToColor(speed);
    const double len = std::clamp(speed * length_scale, min_length, max_length);
    const Point pos = robot->pose.pos;
    const Point vn = robot->vel.linear.normalized();
    const Point base = pos + vn * len;
    // ヘッド（V字）
    const Point v_perp = getVerticalVec(vn);
    const Point base_left = base + v_perp * 0.06 - vn * 0.03;
    const Point base_right = base - v_perp * 0.06 - vn * 0.03;
    tracked_builder->line().start(base).end(base_left).stroke(color, 0.5).strokeWidth(20).build();
    tracked_builder->line().start(base).end(base_right).stroke(color, 0.5).strokeWidth(20).build();
    // トレイル（減衰する点）
    for (int i = 1; i <= trail_points; ++i) {
      const double s = i / (static_cast<double>(trail_points) + 0.5);
      const Point p = pos + s * (base - pos);
      tracked_builder->drawStyledCircle(p, 0.02, color, s * 0.5, "none");
    }
  };

  auto draw_robot = [this](const std::shared_ptr<RobotInfo> & robot, bool is_friend) {
    // 味方のみ直近検出チェック（エラーでも表示はOK）
    if (is_friend) {
      const auto now = node_.now();
      const auto stamp = robot->vision_detection_stamp;
      const bool clock_ok = (stamp.get_clock_type() == now.get_clock_type());
      const bool has_recent_detection = clock_ok && (now - stamp).seconds() < 1.0;
      if (!has_recent_detection) return;
    }

    const Point & pos = robot->pose.pos;
    const double theta = std::isfinite(robot->pose.theta) ? robot->pose.theta : 0.0;

    // ロボット本体とID
    tracked_builder->drawRobotWithID(
      pos, theta, robot->id, is_friend ? "green" : "red", 1.0, "black", 1.0, 10);
  };

  // トラッキング済みロボット（味方）: エラー有無に関わらず、直近検出があれば描画
  for (const auto & robot : world_model->ours().robots) {
    draw_robot(robot, true);
    draw_velocity_marker(robot);
  }

  // トラッキング済みロボット（敵）
  for (const auto & robot : world_model->theirs().robotsWhere().available().get()) {
    draw_robot(robot, false);
    draw_velocity_marker(robot);
  }

  tracked_builder->flush();
}

auto VisualizationManager::drawRefereeInfo(
  const robocup_ssl_msgs::msg::Referee & msg, double field_width, double field_height,
  const std::string & command_text) -> void
{
  // レフェリー状態の表示: 文字列は PlaySituation の string 化を使用（フォールバックは行わない）
  std::string command_name =
    command_text.empty() ? ("COMMAND_" + std::to_string(msg.command.value)) : command_text;

  // レフェリー情報をフィールド上部に表示
  referee_builder->text()
    .text("Referee: " + command_name)
    .position(-field_width / 2, field_height / 2 + 0.5)
    .fontSize(500)
    .fill("white")
    .build();
  referee_builder->flush();
}

auto VisualizationManager::drawBallPlacement(const WorldModelWrapper::SharedPtr & world_model)
  -> void
{
  if (auto target = world_model->getBallPlacementTarget(); target) {
    const auto & ball = world_model->ball();
    placement_builder->drawCircle(target.value(), 0.5, "white", 5);
    Vector2 vertical = getVerticalVec((ball.pos - target.value()).normalized()) * 0.5;
    placement_builder->line()
      .start(ball.pos + vertical)
      .end(target.value() + vertical)
      .stroke("white")
      .strokeWidth(5)
      .build();
    placement_builder->line()
      .start(ball.pos - vertical)
      .end(target.value() - vertical)
      .stroke("white")
      .strokeWidth(5)
      .build();
    placement_builder->flush();
  } else {
    placement_builder->clearBuffer();
    placement_builder->flush();
  }
}

auto VisualizationManager::drawTrajectoryHistory(const TrajectoryHistoryData & trajectory_data)
  -> void
{
  static constexpr int SAMPLING_NUM = 4;

  auto draw_team_history = [&](const auto & histories, const std::string & color) {
    for (const auto & [robot_id, history] : histories | ranges::views::enumerate) {
      if (
        history.size() > SAMPLING_NUM + 1 &&
        (history.front().available_vision || history.front().available_feedback ||
         history.front().available_tracker)) {
        for (int i = 0; i < 10; i++) {
          int start = static_cast<int>((history.size() / 10.) * i);
          int end = static_cast<int>((history.size() / 10.) * (i + 1));

          std::vector<Point> points;
          for (int index = start; index < end; index += SAMPLING_NUM) {
            points.emplace_back(history.at(index).pose.x, history.at(index).pose.y);
          }
          if (i != 9) {
            points.emplace_back(history.at(end).pose.x, history.at(end).pose.y);
          }
          trajectory_builder->drawPolyline(
            points, color, 0.5 * start / static_cast<double>(history.size()), 15);
        }
      }
    }
  };

  // 味方・敵の履歴描画（共通処理）
  draw_team_history(trajectory_data.friend_history, "green");
  draw_team_history(trajectory_data.enemy_history, "red");

  // ボール軌跡描画
  if (trajectory_data.ball_info_history.size() > SAMPLING_NUM + 1) {
    for (int i = 0; i < 10; i++) {
      int start = static_cast<int>((trajectory_data.ball_info_history.size() / 10.) * i);
      int end = static_cast<int>((trajectory_data.ball_info_history.size() / 10.) * (i + 1));

      std::vector<Point> points;
      for (int index = start; index < end; index += SAMPLING_NUM) {
        points.emplace_back(
          trajectory_data.ball_info_history.at(index).position.x,
          trajectory_data.ball_info_history.at(index).position.y);
      }
      if (i != 9) {
        points.emplace_back(
          trajectory_data.ball_info_history.at(end).position.x,
          trajectory_data.ball_info_history.at(end).position.y);
      }
      trajectory_builder->drawPolyline(
        points, "orange", start / static_cast<double>(trajectory_data.ball_info_history.size()),
        30);
    }
  }

  trajectory_builder->flush();
}

}  // namespace crane
