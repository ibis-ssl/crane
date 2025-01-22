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
#include <crane_visualization_interfaces/msg/shape_arc.hpp>
#include <crane_visualization_interfaces/msg/shape_circle.hpp>
#include <crane_visualization_interfaces/msg/shape_line.hpp>
#include <crane_visualization_interfaces/msg/shape_point.hpp>
#include <crane_visualization_interfaces/msg/shape_rectangle.hpp>
#include <crane_visualization_interfaces/msg/shape_robot.hpp>
#include <crane_visualization_interfaces/msg/shape_tube.hpp>
#include <robocup_ssl_msgs/msg/robot_id.hpp>

namespace crane
{
using RobotId = robocup_ssl_msgs::msg::RobotId;

VisualizationDataHandler::VisualizationDataHandler(rclcpp::Node & node)
: visualizer(std::make_shared<CraneVisualizerBuffer::MessageBuilder>("world_model"))
{
  CraneVisualizerBuffer::activate(node);
  sub_referee_ = node.create_subscription<Referee>(
    "referee", 10,
    std::bind(&VisualizationDataHandler::publish_vis_referee, this, std::placeholders::_1));
}

void VisualizationDataHandler::publish_vis_geometry(const SSL_GeometryData & geometry_data)
{
  // geometryを描画情報に変換してpublishする

  for (const auto & field_line : geometry_data.field().field_lines()) {
    SvgLineBuilder builder;
    builder.start(field_line.p1().x() * 0.001, field_line.p1().y() * 0.001)
      .end(field_line.p2().x() * 0.001, field_line.p2().y() * 0.001)
      .stroke("white")
      .strokeWidth(2);
    visualizer->add(builder.getSvgString());
  }

  for (const auto & field_arc : geometry_data.field().field_arcs()) {
    SvgCircleBuilder builder;
    builder.center(field_arc.center().x() * 0.001, field_arc.center().y() * 0.001)
      .radius(field_arc.radius() * 0.001)
      .stroke("white")
      .strokeWidth(2);
    visualizer->add(builder.getSvgString());
  }

  // ペナルティマーク
  // Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_penalty_mark
  SvgCircleBuilder builder;
  builder.center(-geometry_data.field().field_length() * 0.001 / 2.0 + 8.0, 0.0)
    .radius(0.006)
    .fill("white");
  visualizer->add(builder.getSvgString());

  builder.center(geometry_data.field().field_length() * 0.001 / 2.0 - 8.0, 0.0)
    .radius(0.006)
    .fill("white");
  visualizer->add(builder.getSvgString());

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
    .strokeWidth(3);
  visualizer->add(rect_builder.getSvgString());
  visualizer->flush();
  //  CraneVisualizerBuffer::publish();
}

void VisualizationDataHandler::publish_vis_tracked(const TrackedFrame & tracked_frame)
{
  const double VELOCITY_ALPHA = 0.5;
  // tracked_frameを描画情報に変換してpublishする

  for (const auto & ball : tracked_frame.balls()) {
    if (!ball.has_visibility() || ball.visibility() < 0.5) {
      continue;
    }
    SvgCircleBuilder builder;
    builder.center(ball.pos().x(), ball.pos().y())
      .radius(0.0215)
      .stroke("black")
      .fill("orange")
      .strokeWidth(1);
    visualizer->add(builder.getSvgString());

    // ボールは小さいのでボールの周りを大きな円で囲う
    builder.center(ball.pos().x(), ball.pos().y())
      .radius(0.5)
      .stroke("crimson", 0.7)
      .fill("none")
      .strokeWidth(1);
    visualizer->add(builder.getSvgString());

    ball_x = ball.pos().x();
    ball_y = ball.pos().y();

    // 速度を描画
    if (ball.has_vel()) {
      const double vel_norm = std::hypot(ball.vel().x(), ball.vel().y());
      SvgLineBuilder line_builder;
      line_builder.start(ball.pos().x(), ball.pos().y())
        .end(ball.pos().x() + ball.vel().x(), ball.pos().y() + ball.vel().y())
        .stroke("gold", VELOCITY_ALPHA)
        .strokeWidth(2);
      visualizer->add(line_builder.getSvgString());
    }
  }

  for (const auto & robot : tracked_frame.robots()) {
    if (not robot.has_visibility() || robot.visibility() < 0.5) {
      continue;
    }
    SvgPolygonBuilder builder;
    double robot_x = robot.pos().x();
    double robot_y = robot.pos().y();
    double robot_theta = robot.orientation();
    double robot_radius = 0.09;
    builder
      .addPoint(
        robot_x + robot_radius * std::cos(robot_theta),
        robot_y + robot_radius * std::sin(robot_theta))
      .addPoint(
        robot_x + robot_radius * std::cos(robot_theta + 2.0 * M_PI / 3.0),
        robot_y + robot_radius * std::sin(robot_theta + 2.0 * M_PI / 3.0))
      .addPoint(
        robot_x + robot_radius * std::cos(robot_theta + 4.0 * M_PI / 3.0),
        robot_y + robot_radius * std::sin(robot_theta + 4.0 * M_PI / 3.0))
      .stroke("black")
      .strokeWidth(1);
    if (robot.robot_id().team() == RobotId::TEAM_COLOR_BLUE) {
      builder.fill("dodgerblue");
    } else {
      builder.fill("yellow");
    }
    visualizer->add(builder.getSvgString());

    // 速度を描画
    //    if (robot.has_vel() && robot.hans_vel_angular()) {
    //      const double vel_norm = std::hypot(robot.vel().x(), robot.vel().y());
    //      VisLine robot_vel;
    //      // 直進速度
    //      robot_vel.color.name = "gold";
    //      robot_vel.color.alpha = VELOCITY_ALPHA;
    //      robot_vel.size = 2;
    //      robot_vel.p1.x = robot.pos().x();
    //      robot_vel.p1.y = robot.pos().y();
    //      robot_vel.p2.x = robot.pos().x() + robot.vel().x();
    //      robot_vel.p2.y = robot.pos().y() + robot.vel().y();
    //      robot_vel.caption = std::to_string(vel_norm);
    //      vis_objects.lines.push_back(robot_vel);
    //
    //      // 角速度
    //      const double vel_angular_norm = std::fabs(robot.vel_angular[0]);
    //      robot_vel.color.name = "crimson";
    //      robot_vel.color.alpha = VELOCITY_ALPHA;
    //      robot_vel.p1.x = robot.pos().x();
    //      robot_vel.p1.y = robot.pos().y();
    //      robot_vel.p2.x = robot.pos().x() + robot.vel_angular();
    //      robot_vel.p2.y = robot.pos().y();
    //      robot_vel.caption = std::to_string(vel_angular_norm);
    //      vis_objects.lines.push_back(robot_vel);
    //    }
  }
  visualizer->flush();
  //  CraneVisualizerBuffer::publish();
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

void VisualizationDataHandler::publish_vis_referee(const Referee::SharedPtr msg)
{
  // レフェリー情報を描画オブジェクトに変換してpublishする
  const double MARGIN_X = 0.02;
  const double TEXT_HEIGHT = 0.05;
  const double STAGE_COMMAND_WIDTH = 0.15;
  const double STAGE_COMMAND_X = 0.0 + MARGIN_X;
  const double TIMER_WIDTH = 0.15;
  const double TIMER_X = STAGE_COMMAND_X + STAGE_COMMAND_WIDTH + MARGIN_X;
  const double BOTS_WIDTH = 0.2;
  const double BOTS_X = TIMER_X + TIMER_WIDTH + MARGIN_X;
  const double CARDS_WIDTH = 0.1;
  const double CARDS_X = BOTS_X + BOTS_WIDTH + MARGIN_X;
  const double YELLOW_CARD_TIMES_WIDTH = 0.1;
  const double YELLOW_CARD_TIMES_X = CARDS_X + CARDS_WIDTH + MARGIN_X;
  const double TIMEOUT_WIDTH = 0.2;
  const double TIMEOUT_X = YELLOW_CARD_TIMES_X + YELLOW_CARD_TIMES_WIDTH + MARGIN_X;
  const std::string COLOR_TEXT_BLUE = "deepskyblue";
  const std::string COLOR_TEXT_YELLOW = "yellow";
  const std::string COLOR_TEXT_WARNING = "red";

  // STAGEとCOMMANDを表示
  SvgTextBuilder text_builder;
  text_builder.position(STAGE_COMMAND_X, 0.0)
    .text(parse_stage(msg->stage))
    .stroke("white")
    .fontSize(TEXT_HEIGHT);
  visualizer->add(text_builder.getSvgString());

  text_builder.position(STAGE_COMMAND_X, TEXT_HEIGHT)
    .text(parse_command(*msg))
    .stroke("white")
    .fontSize(TEXT_HEIGHT);
  visualizer->add(text_builder.getSvgString());

  // 残り時間とACT_TIMEを表示
  if (msg->stage_time_left.size() > 0) {
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
    text_builder.position(TIMER_X, 0.0)
      .text(parse_stage_time_left(msg->stage_time_left.front()))
      .stroke("white")
      .fontSize(TEXT_HEIGHT);
    visualizer->add(text_builder.getSvgString());
  }

  if (msg->current_action_time_remaining.size() > 0) {
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
    text_builder.position(TIMER_X, TEXT_HEIGHT)
      .text(parse_action_time_remaining(msg->current_action_time_remaining.front()))
      .stroke("white")
      .fontSize(TEXT_HEIGHT);
    visualizer->add(text_builder.getSvgString());
  }

  // ロボット数
  text_builder.position(BOTS_X, 0.0)
    .text("BLUE BOTS: " + std::to_string(msg->blue.max_allowed_bots[0]))
    .stroke(COLOR_TEXT_BLUE)
    .fontSize(TEXT_HEIGHT);
  visualizer->add(text_builder.getSvgString());

  text_builder.position(BOTS_X, TEXT_HEIGHT)
    .text("YELLOW BOTS: " + std::to_string(msg->yellow.max_allowed_bots[0]))
    .stroke(COLOR_TEXT_YELLOW)
    .fontSize(TEXT_HEIGHT);
  visualizer->add(text_builder.getSvgString());

  // カード数
  text_builder.position(CARDS_X, 0.0)
    .text(
      "R: " + std::to_string(msg->blue.red_cards) +
      ", Y: " + std::to_string(msg->blue.yellow_cards))
    .stroke(COLOR_TEXT_BLUE)
    .fontSize(TEXT_HEIGHT);
  visualizer->add(text_builder.getSvgString());

  text_builder.position(CARDS_X, TEXT_HEIGHT)
    .text(
      "R: " + std::to_string(msg->yellow.red_cards) +
      ", Y: " + std::to_string(msg->yellow.yellow_cards))
    .stroke(COLOR_TEXT_YELLOW)
    .fontSize(TEXT_HEIGHT);
  visualizer->add(text_builder.getSvgString());

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
  text_builder.position(YELLOW_CARD_TIMES_X, 0.0)
    .text(parse_yellow_card_times(msg->blue.yellow_card_times))
    .stroke(COLOR_TEXT_BLUE)
    .fontSize(TEXT_HEIGHT);
  visualizer->add(text_builder.getSvgString());

  text_builder.position(YELLOW_CARD_TIMES_X, TEXT_HEIGHT)
    .text(parse_yellow_card_times(msg->yellow.yellow_card_times))
    .stroke(COLOR_TEXT_YELLOW)
    .fontSize(TEXT_HEIGHT);
  visualizer->add(text_builder.getSvgString());

  // タイムアウト
  auto parse_timeouts = [](const auto & timeouts, const auto & timeout_time) {
    return "Timeouts: " + std::to_string(timeouts) + "\n" + std::to_string(timeout_time);
  };
  text_builder.position(TIMEOUT_X, 0.0)
    .text(parse_timeouts(msg->blue.timeouts, msg->blue.timeout_time))
    .stroke(COLOR_TEXT_BLUE)
    .fontSize(TEXT_HEIGHT);
  visualizer->add(text_builder.getSvgString());

  text_builder.position(TIMEOUT_X, TEXT_HEIGHT)
    .text(parse_timeouts(msg->yellow.timeouts, msg->yellow.timeout_time))
    .stroke(COLOR_TEXT_YELLOW)
    .fontSize(TEXT_HEIGHT);
  visualizer->add(text_builder.getSvgString());

  // プレイスメント位置
  if (
    msg->command == Referee::COMMAND_BALL_PLACEMENT_BLUE ||
    msg->command == Referee::COMMAND_BALL_PLACEMENT_YELLOW) {
    if (not msg->designated_position.empty()) {
      SvgLineBuilder line_builder;
      line_builder
        .start(
          msg->designated_position.front().x / 1000., msg->designated_position.front().y / 1000.)
        .end(ball_x, ball_y)
        .stroke("aquamarine")
        .strokeWidth(1);
      visualizer->add(line_builder.getSvgString());
    }
  }
  visualizer->flush();
  //  CraneVisualizerBuffer::publish();
}
}  // namespace crane
