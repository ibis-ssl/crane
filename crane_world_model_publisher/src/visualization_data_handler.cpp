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
  SvgRobotBuilder() : corner_angle(std::acos(center_to_dribbler / radius)) {}

  std::string getSvgString() const override
  {
    SvgPathBuilder path_builder;
    path_builder.definition
      .moveTo(robot_position.x() + botRightX(theta), robot_position.y() + botRightY(theta))
      .arcTo(
        {radius, radius}, 0, true, true,
        {robot_position.x() + botLeftX(theta), robot_position.y() + botLeftY(theta)})
      .lineTo(robot_position.x() + botRightX(theta), robot_position.y() + botRightY(theta));

    path_builder.fill(fill_color, fill_opacity)
      .stroke(stroke_color, stroke_opacity)
      .strokeWidth(stroke_width);

    return path_builder.getSvgString();
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

private:
  Point robot_position;
  double theta;

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

VisualizationDataHandler::VisualizationDataHandler(rclcpp::Node & node)
: visualizer_geometry(
    std::make_shared<CraneVisualizerBuffer::MessageBuilder>("world_model/geometry")),
  visualizer_tracked(
    std::make_shared<CraneVisualizerBuffer::MessageBuilder>("world_model/tracked")),
  visualizer_referee(std::make_shared<CraneVisualizerBuffer::MessageBuilder>("world_model/referee"))
{
  CraneVisualizerBuffer::activate(node);
}

void VisualizationDataHandler::publish_vis_geometry(const SSL_GeometryData & geometry_data)
{
  // geometryを描画情報に変換してpublishする

  for (const auto & field_line : geometry_data.field().field_lines()) {
    SvgLineBuilder builder;
    builder.start(field_line.p1().x() * 0.001, field_line.p1().y() * 0.001)
      .end(field_line.p2().x() * 0.001, field_line.p2().y() * 0.001)
      .stroke("white")
      .strokeWidth(20);
    visualizer_geometry->add(builder.getSvgString());
  }

  for (const auto & field_arc : geometry_data.field().field_arcs()) {
    SvgCircleBuilder builder;
    builder.center(field_arc.center().x() * 0.001, field_arc.center().y() * 0.001)
      .radius(field_arc.radius() * 0.001)
      .stroke("white")
      .strokeWidth(20);
    visualizer_geometry->add(builder.getSvgString());
  }

  // ペナルティマーク
  // Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_penalty_mark
  SvgCircleBuilder builder;
  builder.center(-geometry_data.field().field_length() * 0.001 / 2.0 + 8.0, 0.0)
    .radius(0.006)
    .fill("white");
  visualizer_geometry->add(builder.getSvgString());

  builder.center(geometry_data.field().field_length() * 0.001 / 2.0 - 8.0, 0.0)
    .radius(0.006)
    .fill("white");
  visualizer_geometry->add(builder.getSvgString());

  // フィールドの枠
  SvgRectBuilder rect_builder;
  rect_builder
    .top_left(
      -(geometry_data.field().field_length() + geometry_data.field().boundary_width() * 2) * 0.001 /
        2.0,
      -(geometry_data.field().field_width() + geometry_data.field().boundary_width() * 2) * 0.001 /
        2.0)
    .size(
      (geometry_data.field().field_length() + geometry_data.field().boundary_width() * 2) * 0.001,
      (geometry_data.field().field_width() + geometry_data.field().boundary_width() * 2) * 0.001)
    .stroke("black")
    .strokeWidth(30);
  visualizer_geometry->add(rect_builder.getSvgString());
  visualizer_geometry->flush();
  CraneVisualizerBuffer::publish();
}

void VisualizationDataHandler::publish_vis_tracked(const WorldModelWrapper::SharedPtr & world_model)
{
  const double VELOCITY_ALPHA = 0.5;
  // tracked_frameを描画情報に変換してpublishする

  auto ball = world_model->ball;
  SvgCircleBuilder builder;
  builder.center(ball.pos).radius(0.0215).stroke("black").fill("orange").strokeWidth(10);
  visualizer_tracked->add(builder.getSvgString());

  // ボールは小さいのでボールの周りを大きな円で囲う
  builder.center(ball.pos).radius(0.5).stroke("crimson", 0.7).fill("none").strokeWidth(10);
  visualizer_tracked->add(builder.getSvgString());

  ball_x = ball.pos.x();
  ball_y = ball.pos.y();

  // 速度を描画
  SvgLineBuilder line_builder;
  line_builder.start(ball.pos)
    .end(ball.pos + ball.vel)
    .stroke("gold", VELOCITY_ALPHA)
    .strokeWidth(20);
  visualizer_tracked->add(line_builder.getSvgString());

  auto now = rclcpp::Clock().now();
  const double corner_angle = std::acos(0.055 / 0.085);
  for (const auto & robot : world_model->ours.getAvailableRobots()) {
    SvgRobotBuilder builder;
    builder.position(robot->pose.pos, robot->pose.theta).stroke("black").strokeWidth(10);
    if (world_model->isYellow()) {
      builder.fill("yellow");
    } else {
      builder.fill("dodgerblue");
    }
    visualizer_tracked->add(builder.getSvgString());

    SvgTextBuilder text_id_builder;
    text_id_builder.position(robot->pose.pos.x(), robot->pose.pos.y() + 0.05)
      .text(std::to_string(robot->id))
      .fill("black")
      .fontSize(100)
      .textAnchor("middle");
    visualizer_tracked->add(text_id_builder.getSvgString());

    // ボールセンサ
    if ((now - robot->ball_sensor_stamp).seconds() < 1.0 && robot->ball_sensor) {
      SvgLineBuilder ball_sensor_line;
      ball_sensor_line
        .start(robot->kicker_center() + getVerticalVec(getNormVec(robot->pose.theta)) * 0.055)
        .end(robot->kicker_center() - getVerticalVec(getNormVec(robot->pose.theta)) * 0.055)
        .stroke("red")
        .strokeWidth(15);
      visualizer_tracked->add(ball_sensor_line.getSvgString());
    }
  }

  for (const auto & robot : world_model->theirs.getAvailableRobots()) {
    SvgRobotBuilder builder;
    builder.position(robot->pose.pos, robot->pose.theta).stroke("black").strokeWidth(10);
    if (world_model->isYellow()) {
      builder.fill("dodgerblue");
    } else {
      builder.fill("yellow");
    }
    visualizer_tracked->add(builder.getSvgString());

    SvgTextBuilder text_id_builder;
    text_id_builder.position(robot->pose.pos.x(), robot->pose.pos.y() + 0.05)
      .text(std::to_string(robot->id))
      .fill("black")
      .fontSize(100)
      .textAnchor("middle");
    visualizer_tracked->add(text_id_builder.getSvgString());
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

void VisualizationDataHandler::publish_vis_referee(
  const Referee & msg, double field_width, double field_height)
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
  SvgTextBuilder text_builder;
  text_builder.position(STAGE_COMMAND_X, SECOND_LINE_Y)
    .text(parse_stage(msg.stage))
    .fill("white")
    .fontSize(TEXT_HEIGHT);
  visualizer_referee->add(text_builder.getSvgString());

  text_builder.position(STAGE_COMMAND_X, FIRST_LINE_Y)
    .text(parse_command(msg))
    .fill("white")
    .fontSize(TEXT_HEIGHT);
  visualizer_referee->add(text_builder.getSvgString());

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
    text_builder.position(TIMER_X, SECOND_LINE_Y)
      .text(parse_stage_time_left(msg.stage_time_left.front()))
      .fill("white")
      .fontSize(TEXT_HEIGHT);
    visualizer_referee->add(text_builder.getSvgString());
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
    text_builder.position(TIMER_X, FIRST_LINE_Y)
      .text(parse_action_time_remaining(msg.current_action_time_remaining.front()))
      .fill("white")
      .fontSize(TEXT_HEIGHT);
    visualizer_referee->add(text_builder.getSvgString());
  }

  // ロボット数
  text_builder.position(BOTS_X, SECOND_LINE_Y)
    .text("BLUE BOTS: " + std::to_string(msg.blue.max_allowed_bots[0]))
    .fill(COLOR_TEXT_BLUE)
    .fontSize(TEXT_HEIGHT);
  visualizer_referee->add(text_builder.getSvgString());

  text_builder.position(BOTS_X, FIRST_LINE_Y)
    .text("YELLOW BOTS: " + std::to_string(msg.yellow.max_allowed_bots[0]))
    .fill(COLOR_TEXT_YELLOW)
    .fontSize(TEXT_HEIGHT);
  visualizer_referee->add(text_builder.getSvgString());

  // カード数
  text_builder.position(CARDS_X, SECOND_LINE_Y)
    .text(
      "R: " + std::to_string(msg.blue.red_cards) + ", Y: " + std::to_string(msg.blue.yellow_cards))
    .fill(COLOR_TEXT_BLUE)
    .fontSize(TEXT_HEIGHT);
  visualizer_referee->add(text_builder.getSvgString());

  text_builder.position(CARDS_X, FIRST_LINE_Y)
    .text(
      "R: " + std::to_string(msg.yellow.red_cards) +
      ", Y: " + std::to_string(msg.yellow.yellow_cards))
    .fill(COLOR_TEXT_YELLOW)
    .fontSize(TEXT_HEIGHT);
  visualizer_referee->add(text_builder.getSvgString());

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
  text_builder.position(YELLOW_CARD_TIMES_X, SECOND_LINE_Y)
    .text(parse_yellow_card_times(msg.blue.yellow_card_times))
    .fill(COLOR_TEXT_BLUE)
    .fontSize(TEXT_HEIGHT);
  visualizer_referee->add(text_builder.getSvgString());

  text_builder.position(YELLOW_CARD_TIMES_X, FIRST_LINE_Y)
    .text(parse_yellow_card_times(msg.yellow.yellow_card_times))
    .fill(COLOR_TEXT_YELLOW)
    .fontSize(TEXT_HEIGHT);
  visualizer_referee->add(text_builder.getSvgString());

  // タイムアウト
  auto parse_timeouts = [](const auto & timeouts, const auto & timeout_time) {
    return "Timeouts: " + std::to_string(timeouts) + "\n" + std::to_string(timeout_time);
  };
  text_builder.position(TIMEOUT_X, SECOND_LINE_Y)
    .text(parse_timeouts(msg.blue.timeouts, msg.blue.timeout_time))
    .fill(COLOR_TEXT_BLUE)
    .fontSize(TEXT_HEIGHT);
  visualizer_referee->add(text_builder.getSvgString());

  text_builder.position(TIMEOUT_X, FIRST_LINE_Y)
    .text(parse_timeouts(msg.yellow.timeouts, msg.yellow.timeout_time))
    .fill(COLOR_TEXT_YELLOW)
    .fontSize(TEXT_HEIGHT);
  visualizer_referee->add(text_builder.getSvgString());

  // プレイスメント位置
  if (
    msg.command == Referee::COMMAND_BALL_PLACEMENT_BLUE ||
    msg.command == Referee::COMMAND_BALL_PLACEMENT_YELLOW) {
    if (not msg.designated_position.empty()) {
      SvgLineBuilder line_builder;
      line_builder
        .start(msg.designated_position.front().x / 1000., msg.designated_position.front().y / 1000.)
        .end(ball_x, ball_y)
        .stroke("aquamarine")
        .strokeWidth(10);
      visualizer_referee->add(line_builder.getSvgString());
    }
  }
  visualizer_referee->flush();
  CraneVisualizerBuffer::publish();
}
}  // namespace crane
