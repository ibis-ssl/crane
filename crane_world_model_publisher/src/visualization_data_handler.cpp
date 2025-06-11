// Copyright 2023 Roots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "crane_world_model_publisher/visualization_data_handler.hpp"

#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <robocup_ssl_msgs/msg/robot_id.hpp>

namespace crane
{
using RobotId = robocup_ssl_msgs::msg::RobotId;

struct SvgRobotBuilder : public SvgPathBuilder
{
  explicit SvgRobotBuilder(const std::shared_ptr<VisualizerMessageBuilder> & builder)
  : SvgPathBuilder(builder), corner_angle(std::acos(center_to_dribbler / radius))
  {
  }

  auto getSvgString() const -> std::string override
  {
    SvgPathBuilder path_builder(builder);
    path_builder.definition
      .moveTo(robot_position.x() + botRightX(theta), robot_position.y() + botRightY(theta))
      .arcTo(
        {radius, radius}, 0, true, true,
        {robot_position.x() + botLeftX(theta), robot_position.y() + botLeftY(theta)})
      .lineTo(robot_position.x() + botRightX(theta), robot_position.y() + botRightY(theta));

    return path_builder.fill(fill_color, fill_opacity)
      .stroke(stroke_color, stroke_opacity)
      .strokeWidth(stroke_width)
      .getSvgString();
  }

  auto position(Point p, double theta) -> SvgRobotBuilder &
  {
    this->robot_position = p;
    this->theta = theta;
    return *this;
  }

  auto position(double x, double y, double theta) -> SvgRobotBuilder &
  {
    return position(Point(x, y), theta);
  }

private:
  Point robot_position;

  double theta;

  auto botRightX(double orientation) const -> double
  {
    return radius * std::cos(orientation + corner_angle);
  }
  auto botRightY(double orientation) const -> double
  {
    return radius * std::sin(orientation + corner_angle);
  }
  auto botLeftX(double orientation) const -> double
  {
    return radius * std::cos(orientation - corner_angle);
  }
  auto botLeftY(double orientation) const -> double
  {
    return radius * std::sin(orientation - corner_angle);
  }
  const double radius = 0.085;
  const double center_to_dribbler = 0.055;
  const double corner_angle;
};

VisualizationDataHandler::VisualizationDataHandler(rclcpp::Node & node)
: visualizer_geometry(std::make_shared<VisualizerMessageBuilder>("world_model/geometry")),
  visualizer_vision(std::make_shared<VisualizerMessageBuilder>("world_model/vision")),
  visualizer_tracked(std::make_shared<VisualizerMessageBuilder>("world_model/tracked")),
  visualizer_referee(std::make_shared<VisualizerMessageBuilder>("world_model/referee"))
{
  CraneVisualizerBuffer::activate(node);
}

auto VisualizationDataHandler::publish_vis_geometry(
  const SSL_GeometryData & geometry_data, const bool half_court_practice_mode) -> void
{
  // geometryを描画情報に変換してpublishする
  const double SCALE = half_court_practice_mode ? 0.0005 : 0.001;
  for (const auto & field_line : geometry_data.field().field_lines()) {
    visualizer_geometry->line()
      .start(field_line.p1().x() * SCALE, field_line.p1().y() * SCALE)
      .end(field_line.p2().x() * SCALE, field_line.p2().y() * SCALE)
      .stroke("white")
      .strokeWidth(20)
      .build();
  }

  for (const auto & field_arc : geometry_data.field().field_arcs()) {
    visualizer_geometry->circle()
      .center(field_arc.center().x() * SCALE, field_arc.center().y() * SCALE)
      .radius(field_arc.radius() * 0.001)
      .stroke("white")
      .strokeWidth(20)
      .build();
  }

  // ペナルティマーク
  // Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_penalty_mark
  visualizer_geometry->circle()
    .center(-geometry_data.field().field_length() * SCALE / 2.0 + 8.0, 0.0)
    .radius(0.006)
    .fill("white")
    .build();

  // ペナルティマーク
  // Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_penalty_mark
  visualizer_geometry->circle()
    .center(-geometry_data.field().field_length() * SCALE / 2.0 + 8.0, 0.0)
    .radius(0.006)
    .fill("white")
    .build();

  visualizer_geometry->circle()
    .center(geometry_data.field().field_length() * SCALE / 2.0 - 8.0, 0.0)
    .radius(0.006)
    .fill("white")
    .build();

  // フィールドの枠
  if (half_court_practice_mode) {
    visualizer_geometry->rect()
      .top_left(
        -(geometry_data.field().field_width() + geometry_data.field().boundary_width() * 2) *
          0.001 / 2.0,
        -(geometry_data.field().field_length() * 0.5 + geometry_data.field().boundary_width() * 2) *
          0.001 / 2.0)
      .size(
        (geometry_data.field().field_width() + geometry_data.field().boundary_width() * 2) * 0.001,
        (geometry_data.field().field_length() * 0.5 + geometry_data.field().boundary_width() * 2) *
          0.001)
      .stroke("black")
      .strokeWidth(30)
      .build();
  } else {
    visualizer_geometry->rect()
      .top_left(
        -(geometry_data.field().field_length() + geometry_data.field().boundary_width() * 2) *
          0.001 / 2.0,
        -(geometry_data.field().field_width() + geometry_data.field().boundary_width() * 2) *
          0.001 / 2.0)
      .size(
        (geometry_data.field().field_length() + geometry_data.field().boundary_width() * 2) * 0.001,
        (geometry_data.field().field_width() + geometry_data.field().boundary_width() * 2) * 0.001)
      .stroke("black")
      .strokeWidth(30)
      .build();
  }

  // ゴール(Positive)
  visualizer_geometry->rect()
    .top_left(
      half_court_practice_mode ? geometry_data.field().field_width() * 0.001
                               : geometry_data.field().field_length() * 0.001 / 2.0,
      -geometry_data.field().goal_width() * 0.001 / 2)
    .size(geometry_data.field().goal_depth() * 0.001, geometry_data.field().goal_width() * 0.001)
    .stroke("white")
    .strokeWidth(20)
    .build();
  // ゴール(Negative)
  visualizer_geometry->rect()
    .top_left(
      half_court_practice_mode
        ? -geometry_data.field().field_width() * 0.001 - geometry_data.field().goal_depth() * 0.001
        : -geometry_data.field().field_length() * 0.001 / 2.0 -
            geometry_data.field().goal_depth() * 0.001,
      -geometry_data.field().goal_width() * 0.001 / 2)
    .size(geometry_data.field().goal_depth() * 0.001, geometry_data.field().goal_width() * 0.001)
    .stroke("white")
    .strokeWidth(20)
    .build();

  visualizer_geometry->flush();
  CraneVisualizerBuffer::publish();
}

auto VisualizationDataHandler::publish_vis_detection(
  const SSL_DetectionFrame & detection, const bool half_court_practice_mode) -> void
{
  for (const auto & ball : detection.balls()) {
    visualizer_vision->circle()
      .center(ball.x() * 0.001, ball.y() * 0.001)
      .radius(0.0215 + ball.z() * 0.001)
      .stroke("black", 0.5)
      .strokeWidth(5)
      .build();
  }

  for (const auto & robot : detection.robots_yellow()) {
    visualizer_vision->circle()
      .center(robot.x() * 0.001, robot.y() * 0.001)
      .radius(0.09)
      .stroke("yellow", 0.5)
      .fill("yellow", robot.confidence())
      .strokeWidth(5)
      .build();
  }

  for (const auto & robot : detection.robots_blue()) {
    visualizer_vision->circle()
      .center(robot.x() * 0.001, robot.y() * 0.001)
      .radius(0.09)
      .stroke("blue", 0.5)
      .fill("blue", robot.confidence())
      .strokeWidth(5)
      .build();
  }
  visualizer_vision->flush();
  CraneVisualizerBuffer::publish();
}

auto VisualizationDataHandler::publish_vis_tracked(const WorldModelWrapper::SharedPtr & world_model)
  -> void
{
  const double VELOCITY_ALPHA = 0.5;
  // tracked_frameを描画情報に変換してpublishする

  auto ball = world_model->ball();
  visualizer_tracked->circle()
    .center(ball.pos)
    .radius(0.0215)
    .stroke("black")
    .fill("orange")
    .strokeWidth(10)
    .build();

  // ボールは小さいのでボールの周りを大きな円で囲う
  if (ball.detected) {
    visualizer_tracked->circle()
      .center(ball.pos)
      .radius(0.5)
      .stroke("crimson", 0.5)
      .fill("none")
      .strokeWidth(20)
      .build();
  }

  ball_x = ball.pos.x();
  ball_y = ball.pos.y();

  // 速度を描画
  visualizer_tracked->line()
    .start(ball.pos)
    .end(ball.pos + ball.vel)
    .stroke("gold", VELOCITY_ALPHA)
    .strokeWidth(20)
    .build();

  auto now = rclcpp::Clock(RCL_ROS_TIME).now();
  const double corner_angle = std::acos(0.055 / 0.085);
  for (const auto & robot : world_model->ours().getAvailableRobots()) {
    SvgRobotBuilder builder(visualizer_tracked);
    builder.position(robot->pose.pos, robot->pose.theta)
      .stroke("black")
      .strokeWidth(10)
      .fill(world_model->isYellow() ? "yellow" : "dodgerblue")
      .build();

    visualizer_tracked->text()
      .position(robot->pose.pos.x(), robot->pose.pos.y() + 0.05)
      .text(std::to_string(robot->id))
      .fill("black")
      .fontSize(100)
      .textAnchor("middle")
      .build();

    // ボールセンサ
    if (
      now.get_clock_type() == robot->ball_sensor_stamp.get_clock_type() &&
      std::abs((now - robot->ball_sensor_stamp).seconds()) < 0.01 && robot->ball_sensor) {
      visualizer_tracked->line()
        .start(robot->kicker_center() + getVerticalVec(getNormVec(robot->pose.theta)) * 0.055)
        .end(robot->kicker_center() - getVerticalVec(getNormVec(robot->pose.theta)) * 0.055)
        .stroke("red")
        .strokeWidth(15)
        .build();
    }
  }

  for (const auto & robot : world_model->theirs().getAvailableRobots()) {
    SvgRobotBuilder builder(visualizer_tracked);
    builder.position(robot->pose.pos, robot->pose.theta)
      .stroke("black")
      .strokeWidth(10)
      .fill(world_model->isYellow() ? "dodgerblue" : "yellow")
      .build();

    visualizer_tracked->text()
      .position(robot->pose.pos.x(), robot->pose.pos.y() + 0.05)
      .text(std::to_string(robot->id))
      .fill("black")
      .fontSize(100)
      .textAnchor("middle")
      .build();
  }
  visualizer_tracked->flush();
  CraneVisualizerBuffer::publish();
}

auto parse_stage = [](const auto & ref_stage) -> std::string {
  std::string output = "STAGE";

  if (ref_stage == Referee::STAGE_NORMAL_FIRST_HALF_PRE) {
    output = "FIRST HALF PRE";
  } else if (ref_stage == Referee::STAGE_NORMAL_FIRST_HALF) {
    output = "FIRST HALF";
  } else if (ref_stage == Referee::STAGE_NORMAL_HALF_TIME) {
    output = "HALF TIME";
  } else if (ref_stage == Referee::STAGE_NORMAL_SECOND_HALF_PRE) {
    output = "SECOND HALF PRE";
  } else if (ref_stage == Referee::STAGE_NORMAL_SECOND_HALF) {
    output = "SECOND HALF";
  } else if (ref_stage == Referee::STAGE_EXTRA_TIME_BREAK) {
    output = "EX TIME BREAK";
  } else if (ref_stage == Referee::STAGE_EXTRA_FIRST_HALF_PRE) {
    output = "EX FIRST HALF PRE";
  } else if (ref_stage == Referee::STAGE_EXTRA_FIRST_HALF) {
    output = "EX FIRST HALF";
  } else if (ref_stage == Referee::STAGE_EXTRA_HALF_TIME) {
    output = "EX HALF TIME";
  } else if (ref_stage == Referee::STAGE_EXTRA_SECOND_HALF_PRE) {
    output = "EX SECOND HALF PRE";
  } else if (ref_stage == Referee::STAGE_EXTRA_SECOND_HALF) {
    output = "EX SECOND HALF";
  } else if (ref_stage == Referee::STAGE_PENALTY_SHOOTOUT_BREAK) {
    output = "PENALTY SHOOTOUT BREAK";
  } else if (ref_stage == Referee::STAGE_PENALTY_SHOOTOUT) {
    output = "PENALTY SHOOTOUT";
  } else if (ref_stage == Referee::STAGE_POST_GAME) {
    output = "POST_GAME";
  }

  return output;
};

auto parse_command = [](
                       const Referee referee, std::string blue_color = "blue",
                       std::string yellow_color = "yellow") -> std::string {
  std::string output = "COMMAND";
  std::string text_color = "white";

  if (referee.designated_position.size() > 0) {
    double placement_pos_x = referee.designated_position[0].x * 0.001;
    double placement_pos_y = referee.designated_position[0].y * 0.001;

    output +=
      " (x: " + std::to_string(placement_pos_x) + ", y: " + std::to_string(placement_pos_y) + ")";
  }

  switch (referee.command) {
    case Referee::COMMAND_HALT:
      output = "HALT";
      break;
    case Referee::COMMAND_STOP:
      output = "STOP";
      break;
    case Referee::COMMAND_NORMAL_START:
      output = "NORMAL START";
      break;
    case Referee::COMMAND_FORCE_START:
      output = "FORCE START";
      break;
    case Referee::COMMAND_PREPARE_KICKOFF_YELLOW:
      output = "PREPARE KICKOFF YELLOW";
      text_color = yellow_color;
      break;
    case Referee::COMMAND_PREPARE_KICKOFF_BLUE:
      output = "PREPARE KICKOFF BLUE";
      text_color = blue_color;
      break;
    case Referee::COMMAND_PREPARE_PENALTY_YELLOW:
      output = "PREPARE PENALTY YELLOW";
      text_color = yellow_color;
      break;
    case Referee::COMMAND_PREPARE_PENALTY_BLUE:
      output = "PREPARE PENALTY BLUE";
      text_color = blue_color;
      break;
    case Referee::COMMAND_DIRECT_FREE_YELLOW:
      output = "DIRECT FREE YELLOW";
      text_color = yellow_color;
      break;
    case Referee::COMMAND_DIRECT_FREE_BLUE:
      output = "DIRECT FREE BLUE";
      text_color = blue_color;
      break;
    case Referee::COMMAND_INDIRECT_FREE_YELLOW:
      output = "INDIRECT FREE YELLOW";
      text_color = yellow_color;
      break;
    case Referee::COMMAND_INDIRECT_FREE_BLUE:
      output = "INDIRECT FREE BLUE";
      text_color = blue_color;
      break;
    case Referee::COMMAND_TIMEOUT_YELLOW:
      output = "TIMEOUT YELLOW";
      text_color = yellow_color;
      break;
    case Referee::COMMAND_TIMEOUT_BLUE:
      output = "TIMEOUT BLUE";
      text_color = blue_color;
      break;
    case Referee::COMMAND_BALL_PLACEMENT_YELLOW:
      output = "BALL PLACEMENT YELLOW";
      text_color = yellow_color;
      break;
    case Referee::COMMAND_BALL_PLACEMENT_BLUE:
      output = "BALL PLACEMENT BLUE";
      text_color = blue_color;
      break;
    default:
      output = "UNKNOWN COMMAND";
      break;
  }
  return output;
};

auto VisualizationDataHandler::publish_vis_referee(
  const Referee & msg, double field_width, double field_height) -> void
{
  // レフェリー情報を描画オブジェクトに変換してpublishする
  const double ANCHOR_X = -field_width / 2 - 0.5;
  const double ANCHOR_Y = -field_height / 2 - 0.5;

  const double TEXT_HEIGHT = 200;

  const double STAGE_COMMAND_X = ANCHOR_X;
  const double TIMER_X = STAGE_COMMAND_X + 2.0;
  const double BOTS_X = TIMER_X + 2.0;
  const double CARDS_X = BOTS_X + 2.0;
  const double YELLOW_CARD_TIMES_X = CARDS_X + 1.0;
  const double TIMEOUT_X = YELLOW_CARD_TIMES_X + 1.0;
  const double FIRST_LINE_Y = ANCHOR_Y;
  const double SECOND_LINE_Y = FIRST_LINE_Y - 0.3;
  const std::string COLOR_TEXT_BLUE = "deepskyblue";
  const std::string COLOR_TEXT_YELLOW = "yellow";
  const std::string COLOR_TEXT_WARNING = "red";

  // STAGEとCOMMANDを表示
  visualizer_referee->text()
    .position(STAGE_COMMAND_X, SECOND_LINE_Y)
    .text(parse_stage(msg.stage))
    .fill("white")
    .fontSize(TEXT_HEIGHT)
    .build();

  visualizer_referee->text()
    .position(STAGE_COMMAND_X, FIRST_LINE_Y)
    .text(parse_command(msg))
    .fill("white")
    .fontSize(TEXT_HEIGHT)
    .build();

  // 残り時間とACT_TIMEを表示
  if (msg.stage_time_left.size() > 0) {
    /*
    def parse_stage_time_left(ref_stage_time_left):
    # レフェリーステージの残り時間(usec)を文字列に変換する
    return "STAGE: " + _microseconds_to_text(ref_stage_time_left)
     */
    auto parse_stage_time_left = [](const int ref_stage_time_left) {
      auto parse_microseconds_to_text = [](const auto & microseconds) {
        auto [minutes, seconds] = std::div(std::ceil(microseconds * 1e-6), 60);
        return std::to_string(minutes) + " : " + std::to_string(seconds);
      };
      return "STAGE: " + parse_microseconds_to_text(ref_stage_time_left);
    };
    visualizer_referee->text()
      .position(TIMER_X, SECOND_LINE_Y)
      .text(parse_stage_time_left(msg.stage_time_left.front()))
      .fill("white")
      .fontSize(TEXT_HEIGHT)
      .build();
  }

  if (msg.current_action_time_remaining.size() > 0) {
    auto parse_action_time_remaining = [](const int ref_action_time_remaining) {
      auto parse_microseconds_to_text = [](const auto & microseconds) {
        auto [minutes, seconds] = std::div(std::ceil(microseconds * 1e-6), 60);
        return std::to_string(minutes) + " : " + std::to_string(seconds);
      };
      std::string text = "0:00";
      if (ref_action_time_remaining > 0) {
        text = parse_microseconds_to_text(ref_action_time_remaining);
      }
      return "ACT: " + text;
    };
    visualizer_referee->text()
      .position(TIMER_X, FIRST_LINE_Y)
      .text(parse_action_time_remaining(msg.current_action_time_remaining.front()))
      .fill("white")
      .fontSize(TEXT_HEIGHT)
      .build();
  }

  // ロボット数
  visualizer_referee->text()
    .position(BOTS_X, SECOND_LINE_Y)
    .text("BLUE BOTS: " + std::to_string(msg.blue.max_allowed_bots[0]))
    .fill(COLOR_TEXT_BLUE)
    .fontSize(TEXT_HEIGHT)
    .build();

  visualizer_referee->text()
    .position(BOTS_X, FIRST_LINE_Y)
    .text("YELLOW BOTS: " + std::to_string(msg.yellow.max_allowed_bots[0]))
    .fill(COLOR_TEXT_YELLOW)
    .fontSize(TEXT_HEIGHT)
    .build();

  // カード数
  visualizer_referee->text()
    .position(CARDS_X, SECOND_LINE_Y)
    .text(
      "R: " + std::to_string(msg.blue.red_cards) + ", Y: " + std::to_string(msg.blue.yellow_cards))
    .fill(COLOR_TEXT_BLUE)
    .fontSize(TEXT_HEIGHT)
    .build();

  visualizer_referee->text()
    .position(CARDS_X, FIRST_LINE_Y)
    .text(
      "R: " + std::to_string(msg.yellow.red_cards) +
      ", Y: " + std::to_string(msg.yellow.yellow_cards))
    .fill(COLOR_TEXT_YELLOW)
    .fontSize(TEXT_HEIGHT)
    .build();

  // イエローカードの時間
  auto parse_yellow_card_times = [](const auto & yellow_card_times) {
    auto parse_microseconds_to_text = [](const auto & microseconds) {
      auto [minutes, seconds] = std::div(std::ceil(microseconds * 1e-6), 60);
      return std::to_string(minutes) + " : " + std::to_string(seconds);
    };
    std::string text = "";
    for (size_t i = 0; i < yellow_card_times.size(); ++i) {
      text += parse_microseconds_to_text(yellow_card_times[i]);
      if (i != yellow_card_times.size() - 1) {
        text += "\n";
      }
    }
    return text;
  };
  visualizer_referee->text()
    .position(YELLOW_CARD_TIMES_X, SECOND_LINE_Y)
    .text(parse_yellow_card_times(msg.blue.yellow_card_times))
    .fill(COLOR_TEXT_BLUE)
    .fontSize(TEXT_HEIGHT)
    .build();

  visualizer_referee->text()
    .position(YELLOW_CARD_TIMES_X, FIRST_LINE_Y)
    .text(parse_yellow_card_times(msg.yellow.yellow_card_times))
    .fill(COLOR_TEXT_YELLOW)
    .fontSize(TEXT_HEIGHT)
    .build();

  // タイムアウト
  auto parse_timeouts = [](const auto & timeouts, const auto & timeout_time) {
    return "Timeouts: " + std::to_string(timeouts) + "\n" + std::to_string(timeout_time);
  };
  visualizer_referee->text()
    .position(TIMEOUT_X, SECOND_LINE_Y)
    .text(parse_timeouts(msg.blue.timeouts, msg.blue.timeout_time))
    .fill(COLOR_TEXT_BLUE)
    .fontSize(TEXT_HEIGHT)
    .build();

  visualizer_referee->text()
    .position(TIMEOUT_X, FIRST_LINE_Y)
    .text(parse_timeouts(msg.yellow.timeouts, msg.yellow.timeout_time))
    .fill(COLOR_TEXT_YELLOW)
    .fontSize(TEXT_HEIGHT)
    .build();

  // プレイスメント位置
  if (
    msg.command == Referee::COMMAND_BALL_PLACEMENT_BLUE ||
    msg.command == Referee::COMMAND_BALL_PLACEMENT_YELLOW) {
    if (not msg.designated_position.empty()) {
      visualizer_referee->line()
        .start(msg.designated_position.front().x / 1000., msg.designated_position.front().y / 1000.)
        .end(ball_x, ball_y)
        .stroke("aquamarine")
        .strokeWidth(10)
        .build();
    }
  }
  visualizer_referee->flush();
  CraneVisualizerBuffer::publish();
}
}  // namespace crane
