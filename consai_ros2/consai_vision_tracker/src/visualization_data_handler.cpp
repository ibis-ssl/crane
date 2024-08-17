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

#include "consai_vision_tracker/visualization_data_handler.hpp"

#include <consai_visualizer_msgs/msg/shape_arc.hpp>
#include <consai_visualizer_msgs/msg/shape_circle.hpp>
#include <consai_visualizer_msgs/msg/shape_line.hpp>
#include <consai_visualizer_msgs/msg/shape_point.hpp>
#include <consai_visualizer_msgs/msg/shape_rectangle.hpp>
#include <consai_visualizer_msgs/msg/shape_robot.hpp>
#include <robocup_ssl_msgs/msg/robot_id.hpp>

namespace consai_vision_tracker
{
using VisColor = consai_visualizer_msgs::msg::Color;
using VisArc = consai_visualizer_msgs::msg::ShapeArc;
using VisAnnotation = consai_visualizer_msgs::msg::ShapeAnnotation;
using VisCircle = consai_visualizer_msgs::msg::ShapeCircle;
using VisLine = consai_visualizer_msgs::msg::ShapeLine;
using VisPoint = consai_visualizer_msgs::msg::ShapePoint;
using VisRect = consai_visualizer_msgs::msg::ShapeRectangle;
using VisRobot = consai_visualizer_msgs::msg::ShapeRobot;
using VisText = consai_visualizer_msgs::msg::ShapeText;
using RobotId = robocup_ssl_msgs::msg::RobotId;

VisualizationDataHandler::VisualizationDataHandler(rclcpp::Node & node)
{
  pub_vis_objects_ =
    node.create_publisher<VisualizerObjects>("visualizer_objects", rclcpp::SensorDataQoS());
  sub_referee_ = node.create_subscription<Referee>(
    "referee", 10,
    std::bind(&VisualizationDataHandler::publish_vis_referee, this, std::placeholders::_1));
}

void VisualizationDataHandler::publish_vis_detection(const DetectionFrame::SharedPtr msg)
{
  // detectionを描画情報に変換してpublishする
  auto vis_objects = std::make_unique<VisualizerObjects>();
  bool has_object = false;
  const auto cam_id = std::to_string(msg->camera_id);

  vis_objects->layer = "vision";
  vis_objects->sub_layer = "detection_cam" + cam_id;
  vis_objects->z_order = 1;

  VisCircle vis_ball;
  vis_ball.line_color.name = "black";
  vis_ball.fill_color.name = "orange";
  vis_ball.fill_color.alpha = 0.7;
  vis_ball.line_size = 1;
  vis_ball.radius = 0.0215;
  //  vis_ball.caption = cam_id;

  for (const auto & ball : msg->balls) {
    vis_ball.center.x = ball.x * 0.001;
    vis_ball.center.y = ball.y * 0.001;
    vis_objects->circles.push_back(vis_ball);
    has_object = true;
  }

  VisRobot vis_robot;
  vis_robot.line_color.name = "black";
  vis_robot.fill_color.name = "dodgerblue";
  vis_robot.fill_color.alpha = 0.7;
  vis_robot.line_size = 1;
  //  vis_robot.caption = cam_id;
  for (const auto & robot : msg->robots_blue) {
    if (robot.robot_id.size() <= 0) {
      continue;
    }
    vis_robot.id = robot.robot_id[0];
    vis_robot.x = robot.x * 0.001;
    vis_robot.y = robot.y * 0.001;
    if (robot.orientation.size() > 0) {
      vis_robot.theta = robot.orientation[0];
    }
    vis_objects->robots.push_back(vis_robot);
    has_object = true;
  }

  vis_robot.line_color.name = "black";
  vis_robot.fill_color.name = "yellow";
  for (const auto & robot : msg->robots_yellow) {
    if (robot.robot_id.size() <= 0) {
      continue;
    }
    vis_robot.id = robot.robot_id[0];
    vis_robot.x = robot.x * 0.001;
    vis_robot.y = robot.y * 0.001;
    if (robot.orientation.size() > 0) {
      vis_robot.theta = robot.orientation[0];
    }
    vis_objects->robots.push_back(vis_robot);
    has_object = true;
  }

  if (has_object) {
    pub_vis_objects_->publish(std::move(vis_objects));
  }
}

void VisualizationDataHandler::publish_vis_geometry(const GeometryData::SharedPtr msg)
{
  // geometryを描画情報に変換してpublishする
  auto vis_objects = std::make_unique<VisualizerObjects>();

  vis_objects->layer = "vision";
  vis_objects->sub_layer = "geometry";
  vis_objects->z_order = 0;

  for (const auto & field_line : msg->field.field_lines) {
    VisLine line;

    line.color.name = "white";
    line.size = 2;
    // 単位を[m]に変換
    line.p1.x = field_line.p1.x * 0.001;
    line.p1.y = field_line.p1.y * 0.001;
    line.p2.x = field_line.p2.x * 0.001;
    line.p2.y = field_line.p2.y * 0.001;
    //    line.caption = field_line.name;

    vis_objects->lines.push_back(line);
  }

  for (const auto & field_arc : msg->field.field_arcs) {
    VisArc arc;

    arc.color.name = "white";
    arc.size = 2;
    // 単位を[m]に変換
    arc.center.x = field_arc.center.x * 0.001;
    arc.center.y = field_arc.center.y * 0.001;
    arc.radius = field_arc.radius * 0.001;
    arc.start_angle = field_arc.a1;
    arc.end_angle = field_arc.a2;
    //    arc.caption = field_arc.name;

    vis_objects->arcs.push_back(arc);
  }

  // ペナルティマーク
  // Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_penalty_mark
  VisPoint point;
  point.color.name = "white";
  point.size = 6;
  point.x = -msg->field.field_length * 0.001 / 2.0 + 8.0;
  point.y = 0.0;
  //  point.caption = "penalty_mark_positive";
  vis_objects->points.push_back(point);

  point.x = -point.x;
  //  point.caption = "penalty_mark_negative";
  vis_objects->points.push_back(point);

  // フィールドの枠
  VisRect rect;
  rect.line_color.name = "black";
  rect.fill_color.alpha = 0.0;
  rect.line_size = 3;
  rect.center.x = 0.0;
  rect.center.y = 0.0;
  rect.width = (msg->field.field_length + msg->field.boundary_width * 2) * 0.001;
  rect.height = (msg->field.field_width + msg->field.boundary_width * 2) * 0.001;
  //  rect.caption = "wall";
  vis_objects->rects.push_back(rect);

  pub_vis_objects_->publish(std::move(vis_objects));
}

TrackedFrame::UniquePtr VisualizationDataHandler::publish_vis_tracked(TrackedFrame::UniquePtr msg)
{
  const double VELOCITY_ALPHA = 0.5;
  // tracked_frameを描画情報に変換してpublishする
  auto vis_objects = std::make_unique<VisualizerObjects>();
  vis_objects->layer = "vision";
  vis_objects->sub_layer = "tracked";
  vis_objects->z_order = 10;  // 一番上に描画する

  VisCircle vis_ball;
  vis_ball.line_color.name = "black";
  vis_ball.fill_color.name = "orange";
  vis_ball.line_size = 1;
  vis_ball.radius = 0.0215;
  for (const auto & ball : msg->balls) {
    if (ball.visibility.size() <= 0 || ball.visibility[0] < 0.5) {
      continue;
    }
    vis_ball.center.x = ball.pos.x;
    vis_ball.center.y = ball.pos.y;
    vis_objects->circles.push_back(vis_ball);

    // ボールは小さいのでボールの周りを大きな円で囲う
    vis_ball.line_color.name = "crimson";
    vis_ball.fill_color.alpha = 0.0;
    vis_ball.line_size = 2;
    vis_ball.radius = 0.8;
    vis_ball.caption = "ball is here";
    vis_objects->circles.push_back(vis_ball);

    // 速度を描画
    if (ball.vel.size() > 0) {
      const double vel_norm = std::hypot(ball.vel[0].x, ball.vel[0].y);
      VisLine ball_vel;
      ball_vel.color.name = "gold";
      ball_vel.color.alpha = VELOCITY_ALPHA;
      ball_vel.size = 2;
      ball_vel.p1.x = ball.pos.x;
      ball_vel.p1.y = ball.pos.y;
      ball_vel.p2.x = ball.pos.x + ball.vel[0].x;
      ball_vel.p2.y = ball.pos.y + ball.vel[0].y;
      //      ball_vel.caption = std::to_string(vel_norm);
      vis_objects->lines.push_back(ball_vel);
    }
  }

  VisRobot vis_robot;
  vis_robot.line_color.name = "black";
  vis_robot.line_size = 1;
  for (const auto & robot : msg->robots) {
    if (robot.visibility.size() <= 0 || robot.visibility[0] < 0.5) {
      continue;
    }

    vis_robot.id = robot.robot_id.id;
    if (robot.robot_id.team_color == RobotId::TEAM_COLOR_BLUE) {
      vis_robot.fill_color.name = "dodgerblue";
    } else {
      vis_robot.fill_color.name = "yellow";
    }

    vis_robot.x = robot.pos.x;
    vis_robot.y = robot.pos.y;
    vis_robot.theta = robot.orientation;
    vis_objects->robots.push_back(vis_robot);

    // 速度を描画
    //    if (robot.vel.size() > 0 && robot.vel_angular.size() > 0) {
    //      const double vel_norm = std::hypot(robot.vel[0].x, robot.vel[0].y);
    //      VisLine robot_vel;
    //      // 直進速度
    //      robot_vel.color.name = "gold";
    //      robot_vel.color.alpha = VELOCITY_ALPHA;
    //      robot_vel.size = 2;
    //      robot_vel.p1.x = robot.pos.x;
    //      robot_vel.p1.y = robot.pos.y;
    //      robot_vel.p2.x = robot.pos.x + robot.vel[0].x;
    //      robot_vel.p2.y = robot.pos.y + robot.vel[0].y;
    //      robot_vel.caption = std::to_string(vel_norm);
    //      vis_objects->lines.push_back(robot_vel);
    //
    //      // 角速度
    //      const double vel_angular_norm = std::fabs(robot.vel_angular[0]);
    //      robot_vel.color.name = "crimson";
    //      robot_vel.color.alpha = VELOCITY_ALPHA;
    //      robot_vel.p1.x = robot.pos.x;
    //      robot_vel.p1.y = robot.pos.y;
    //      robot_vel.p2.x = robot.pos.x + robot.vel_angular[0];
    //      robot_vel.p2.y = robot.pos.y;
    //      robot_vel.caption = std::to_string(vel_angular_norm);
    //      vis_objects->lines.push_back(robot_vel);
    //    }
  }

  pub_vis_objects_->publish(std::move(vis_objects));

  return msg;
}

/*
def vis_info(referee: Referee, blue_bots: int, yellow_bots: int,
             placement_pos: State2D):
    # レフェリー情報を描画オブジェクトに変換する
    MARGIN_X = 0.02
    TEXT_HEIGHT = 0.05
    STAGE_COMMAND_WIDTH = 0.15
    STAGE_COMMAND_X = 0.0 + MARGIN_X
    TIMER_WIDTH = 0.15
    TIMER_X = STAGE_COMMAND_X + STAGE_COMMAND_WIDTH + MARGIN_X
    BOTS_WIDTH = 0.2
    BOTS_X = TIMER_X + TIMER_WIDTH + MARGIN_X
    CARDS_WIDTH = 0.1
    CARDS_X = BOTS_X + BOTS_WIDTH + MARGIN_X
    YELLOW_CARD_TIMES_WIDTH = 0.1
    YELLOW_CARD_TIMES_X = CARDS_X + CARDS_WIDTH + MARGIN_X
    TIMEOUT_WIDTH = 0.1
    TIMEOUT_X = YELLOW_CARD_TIMES_X + YELLOW_CARD_TIMES_WIDTH + MARGIN_X
    COLOR_TEXT_BLUE = 'deepskyblue'
    COLOR_TEXT_YELLOW = 'yellow'
    COLOR_TEXT_WARNING = 'red'

    vis_objects = Objects()
    vis_objects.layer = 'referee'
    vis_objects.sub_layer = 'info'
    vis_objects.z_order = 2

    # 左端にSTAGEとCOMMANDを表示
    vis_annotation = ShapeAnnotation()
    vis_annotation.text = parse_stage(referee.stage)
    vis_annotation.color.name = 'white'
    vis_annotation.normalized_x = STAGE_COMMAND_X
    vis_annotation.normalized_y = 0.0
    vis_annotation.normalized_width = STAGE_COMMAND_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    vis_annotation = ShapeAnnotation()
    vis_annotation.text, vis_annotation.color.name = parse_command(
        referee, COLOR_TEXT_BLUE, COLOR_TEXT_YELLOW)
    vis_annotation.normalized_x = STAGE_COMMAND_X
    vis_annotation.normalized_y = TEXT_HEIGHT
    vis_annotation.normalized_width = STAGE_COMMAND_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    # 残り時間とACT_TIMEを表示
    if referee.stage_time_left:
        vis_annotation = ShapeAnnotation()
        vis_annotation.text = parse_stage_time_left(referee.stage_time_left[0])
        vis_annotation.color.name = 'white'
        vis_annotation.normalized_x = TIMER_X
        vis_annotation.normalized_y = 0.0
        vis_annotation.normalized_width = TIMER_WIDTH
        vis_annotation.normalized_height = TEXT_HEIGHT
        vis_objects.annotations.append(vis_annotation)

    if referee.current_action_time_remaining:
        vis_annotation = ShapeAnnotation()
        vis_annotation.text = parse_action_time_remaining(referee.current_action_time_remaining[0])
        vis_annotation.color.name = 'white'
        vis_annotation.normalized_x = TIMER_X
        vis_annotation.normalized_y = TEXT_HEIGHT
        vis_annotation.normalized_width = TIMER_WIDTH
        vis_annotation.normalized_height = TEXT_HEIGHT
        vis_objects.annotations.append(vis_annotation)

    # ロボット数
    if referee.blue.max_allowed_bots:
        vis_annotation = ShapeAnnotation()
        vis_annotation.color.name = COLOR_TEXT_BLUE
        # 許可台数よりロボットが多い場合は色を変える
        if blue_bots > referee.blue.max_allowed_bots[0]:
            vis_annotation.color.name = COLOR_TEXT_WARNING
        vis_annotation.text = 'BLUE BOTS: {}/{}'.format(
            blue_bots, referee.blue.max_allowed_bots[0])
        vis_annotation.normalized_x = BOTS_X
        vis_annotation.normalized_y = 0.0
        vis_annotation.normalized_width = BOTS_WIDTH
        vis_annotation.normalized_height = TEXT_HEIGHT
        vis_objects.annotations.append(vis_annotation)

    if referee.yellow.max_allowed_bots:
        vis_annotation = ShapeAnnotation()
        vis_annotation.color.name = COLOR_TEXT_YELLOW
        # 許可台数よりロボットが多い場合は色を変える
        if yellow_bots > referee.yellow.max_allowed_bots[0]:
            vis_annotation.color.name = COLOR_TEXT_WARNING
        vis_annotation.text = 'YELLOW BOTS: {}/{}'.format(
            yellow_bots, referee.yellow.max_allowed_bots[0])
        vis_annotation.normalized_x = BOTS_X
        vis_annotation.normalized_y = TEXT_HEIGHT
        vis_annotation.normalized_width = BOTS_WIDTH
        vis_annotation.normalized_height = TEXT_HEIGHT
        vis_objects.annotations.append(vis_annotation)

    # カード数
    vis_annotation = ShapeAnnotation()
    vis_annotation.color.name = COLOR_TEXT_BLUE
    vis_annotation.text = 'R: {}, Y:{}'.format(
        referee.blue.red_cards,
        referee.blue.yellow_cards)
    vis_annotation.normalized_x = CARDS_X
    vis_annotation.normalized_y = 0.0
    vis_annotation.normalized_width = CARDS_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    vis_annotation = ShapeAnnotation()
    vis_annotation.color.name = COLOR_TEXT_YELLOW
    vis_annotation.text = 'R: {}, Y:{}'.format(
        referee.yellow.red_cards,
        referee.yellow.yellow_cards)
    vis_annotation.normalized_x = CARDS_X
    vis_annotation.normalized_y = TEXT_HEIGHT
    vis_annotation.normalized_width = CARDS_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    # イエローカードの時間
    vis_annotation = ShapeAnnotation()
    vis_annotation.color.name = COLOR_TEXT_BLUE
    vis_annotation.text = parse_yellow_card_times(referee.blue.yellow_card_times)
    vis_annotation.normalized_x = YELLOW_CARD_TIMES_X
    vis_annotation.normalized_y = 0.0
    vis_annotation.normalized_width = YELLOW_CARD_TIMES_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    vis_annotation = ShapeAnnotation()
    vis_annotation.color.name = COLOR_TEXT_YELLOW
    vis_annotation.text = parse_yellow_card_times(referee.yellow.yellow_card_times)
    vis_annotation.normalized_x = YELLOW_CARD_TIMES_X
    vis_annotation.normalized_y = TEXT_HEIGHT
    vis_annotation.normalized_width = YELLOW_CARD_TIMES_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    # タイムアウト
    vis_annotation = ShapeAnnotation()
    vis_annotation.color.name = COLOR_TEXT_BLUE
    vis_annotation.text = parse_timeouts(referee.blue.timeouts, referee.blue.timeout_time)
    vis_annotation.normalized_x = TIMEOUT_X
    vis_annotation.normalized_y = 0.0
    vis_annotation.normalized_width = TIMEOUT_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    vis_annotation = ShapeAnnotation()
    vis_annotation.color.name = COLOR_TEXT_YELLOW
    vis_annotation.text = parse_timeouts(referee.yellow.timeouts, referee.yellow.timeout_time)
    vis_annotation.normalized_x = TIMEOUT_X
    vis_annotation.normalized_y = TEXT_HEIGHT
    vis_annotation.normalized_width = TIMEOUT_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    # プレースメント位置
    if referee.command == Referee.COMMAND_BALL_PLACEMENT_BLUE or \
       referee.command == Referee.COMMAND_BALL_PLACEMENT_YELLOW:
        vis_circle = ShapeCircle()
        vis_circle.center.x = placement_pos.x
        vis_circle.center.y = placement_pos.y
        vis_circle.radius = 0.15
        vis_circle.line_color.name = 'aquamarine'
        vis_circle.fill_color.name = 'aquamarine'
        vis_circle.line_size = 1
        vis_circle.caption = 'placement pos'
        vis_objects.circles.append(vis_circle)

    return vis_objects
 */

/*
def parse_stage(ref_stage):
    # レフェリーステージを文字列に変換する
    output = "STAGE"

    if ref_stage == Referee.STAGE_NORMAL_FIRST_HALF_PRE:
        output = "FIRST HALF PRE"
    elif ref_stage == Referee.STAGE_NORMAL_FIRST_HALF:
        output = "FIRST HALF"
    elif ref_stage == Referee.STAGE_NORMAL_HALF_TIME:
        output = "HALF TIME"
    elif ref_stage == Referee.STAGE_NORMAL_SECOND_HALF_PRE:
        output = "SECOND HALF PRE"
    elif ref_stage == Referee.STAGE_NORMAL_SECOND_HALF:
        output = "SECOND HALF"
    elif ref_stage == Referee.STAGE_EXTRA_TIME_BREAK:
        output = "EX TIME BREAK"
    elif ref_stage == Referee.STAGE_EXTRA_FIRST_HALF_PRE:
        output = "EX FIRST HALF PRE"
    elif ref_stage == Referee.STAGE_EXTRA_FIRST_HALF:
        output = "EX FIRST HALF"
    elif ref_stage == Referee.STAGE_EXTRA_HALF_TIME:
        output = "EX HALF TIME"
    elif ref_stage == Referee.STAGE_EXTRA_SECOND_HALF_PRE:
        output = "EX SECOND HALF PRE"
    elif ref_stage == Referee.STAGE_EXTRA_SECOND_HALF:
        output = "EX SECOND HALF"
    elif ref_stage == Referee.STAGE_PENALTY_SHOOTOUT_BREAK:
        output = "PENALTY SHOOTOUT BREAK"
    elif ref_stage == Referee.STAGE_PENALTY_SHOOTOUT:
        output = "PENALTY SHOOTOUT"
    elif ref_stage == Referee.STAGE_POST_GAME:
        output = "POST_GAME"

    return output
 */

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

/*
def parse_command(referee: Referee,
                  blue_color: str = 'blue', yellow_color: str = 'yellow') -> (str, str):
    # レフェリーコマンドを文字列と文字色に変換する
    output = "COMMAND"
    text_color = 'white'

    if len(referee.designated_position) > 0:
        placement_pos_x = referee.designated_position[0].x * 0.001
        placement_pos_y = referee.designated_position[0].y * 0.001

    ref_command = referee.command

    if ref_command == Referee.COMMAND_HALT:
        output = "HALT"
    elif ref_command == Referee.COMMAND_STOP:
        output = "STOP"
    elif ref_command == Referee.COMMAND_NORMAL_START:
        output = "NORMAL START"
    elif ref_command == Referee.COMMAND_FORCE_START:
        output = "FORCE START"
    elif ref_command == Referee.COMMAND_PREPARE_KICKOFF_YELLOW:
        output = "PREPARE KICK OFF YELLOW"
        text_color = yellow_color
    elif ref_command == Referee.COMMAND_PREPARE_KICKOFF_BLUE:
        output = "PREPARE KICK OFF BLUE"
        text_color = blue_color
    elif ref_command == Referee.COMMAND_PREPARE_PENALTY_YELLOW:
        output = "PREPARE PENALTY YELLOW"
        text_color = yellow_color
    elif ref_command == Referee.COMMAND_PREPARE_PENALTY_BLUE:
        output = "PREPARE PENALTY BLUE"
        text_color = blue_color
    elif ref_command == Referee.COMMAND_DIRECT_FREE_YELLOW:
        output = "DIRECT FREE YELLOW"
        text_color = yellow_color
    elif ref_command == Referee.COMMAND_DIRECT_FREE_BLUE:
        output = "DIRECT FREE BLUE"
        text_color = blue_color
    elif ref_command == Referee.COMMAND_INDIRECT_FREE_YELLOW:
        output = "INDIRECT FREE YELLOW"
        text_color = yellow_color
    elif ref_command == Referee.COMMAND_INDIRECT_FREE_BLUE:
        output = "INDIRECT FREE BLUE"
        text_color = blue_color
    elif ref_command == Referee.COMMAND_TIMEOUT_YELLOW:
        output = "TIMEOUT YELLOW"
        text_color = yellow_color
    elif ref_command == Referee.COMMAND_TIMEOUT_BLUE:
        output = "TIMEOUT BLUE"
        text_color = blue_color
    elif ref_command == Referee.COMMAND_BALL_PLACEMENT_YELLOW:
        output = "BALL PLACEMENT YELLOW"
        output += "(x: {:.1f}, y: {:.1f})".format(placement_pos_x, placement_pos_y)
        text_color = yellow_color
    elif ref_command == Referee.COMMAND_BALL_PLACEMENT_BLUE:
        output = "BALL PLACEMENT BLUE"
        output += "(x: {:.1f}, y: {:.1f})".format(placement_pos_x, placement_pos_y)
        text_color = blue_color

    return (output, text_color)

 */

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
  // std::cout << "publish_vis_referee" << std::endl;
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
  const double TIMEOUT_WIDTH = 0.1;
  const double TIMEOUT_X = YELLOW_CARD_TIMES_X + YELLOW_CARD_TIMES_WIDTH + MARGIN_X;
  const std::string COLOR_TEXT_BLUE = "deepskyblue";
  const std::string COLOR_TEXT_YELLOW = "yellow";
  const std::string COLOR_TEXT_WARNING = "red";

  auto vis_objects = std::make_unique<VisualizerObjects>();
  vis_objects->layer = "referee";
  vis_objects->sub_layer = "info";
  vis_objects->z_order = 2;

  // STAGEとCOMMANDを表示
  VisAnnotation vis_annotation;
  vis_annotation.text = parse_stage(msg->stage);
  vis_annotation.color.name = "white";
  vis_annotation.normalized_x = STAGE_COMMAND_X;
  vis_annotation.normalized_y = 0.0;
  vis_annotation.normalized_width = STAGE_COMMAND_WIDTH;
  vis_annotation.normalized_height = TEXT_HEIGHT;
  vis_objects->annotations.push_back(vis_annotation);

  vis_annotation.text = parse_command(*msg);
  vis_annotation.color.name = "white";
  vis_annotation.normalized_x = STAGE_COMMAND_X;
  vis_annotation.normalized_y = TEXT_HEIGHT;
  vis_annotation.normalized_width = STAGE_COMMAND_WIDTH;
  vis_annotation.normalized_height = TEXT_HEIGHT;
  vis_objects->annotations.push_back(vis_annotation);

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
    vis_annotation.text = parse_stage_time_left(msg->stage_time_left.front());
    vis_annotation.color.name = "white";
    vis_annotation.normalized_x = TIMER_X;
    vis_annotation.normalized_y = 0.0;
    vis_annotation.normalized_width = TIMER_WIDTH;
    vis_annotation.normalized_height = TEXT_HEIGHT;
    vis_objects->annotations.push_back(vis_annotation);
  }

  if (msg->current_action_time_remaining.size() > 0) {
    /*
    def _microseconds_to_text(microseconds):
    minutes, seconds = divmod(math.ceil(microseconds * 1e-6), 60)  # ceilで小数点切り上げ
    return '{} : {:0=2}'.format(minutes, seconds)  # 秒はゼロで埋める

    def parse_action_time_remaining(ref_action_time_remaining):
    # アクション残り時間(usec)を文字列に変換する
    text = "0:00"
    if ref_action_time_remaining > 0:
        text = _microseconds_to_text(ref_action_time_remaining)
    return "ACT: " + text
    */

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
    vis_annotation.text = parse_action_time_remaining(msg->current_action_time_remaining.front());
    vis_annotation.color.name = "white";
    vis_annotation.normalized_x = TIMER_X;
    vis_annotation.normalized_y = TEXT_HEIGHT;
    vis_annotation.normalized_width = TIMER_WIDTH;
    vis_annotation.normalized_height = TEXT_HEIGHT;
    vis_objects->annotations.push_back(vis_annotation);
  }

  // ロボット数
  vis_annotation.color.name = COLOR_TEXT_BLUE;
  vis_annotation.text = "BLUE BOTS: " + std::to_string(msg->blue.max_allowed_bots[0]);
  vis_annotation.normalized_x = BOTS_X;
  vis_annotation.normalized_y = 0.0;
  vis_annotation.normalized_width = BOTS_WIDTH;
  vis_annotation.normalized_height = TEXT_HEIGHT;
  vis_objects->annotations.push_back(vis_annotation);

  vis_annotation.color.name = COLOR_TEXT_YELLOW;
  vis_annotation.text = "YELLOW BOTS: " + std::to_string(msg->yellow.max_allowed_bots[0]);
  vis_annotation.normalized_x = BOTS_X;
  vis_annotation.normalized_y = TEXT_HEIGHT;
  vis_annotation.normalized_width = BOTS_WIDTH;
  vis_annotation.normalized_height = TEXT_HEIGHT;
  vis_objects->annotations.push_back(vis_annotation);

  // カード数
  vis_annotation.color.name = COLOR_TEXT_BLUE;
  vis_annotation.text =
    "R: " + std::to_string(msg->blue.red_cards) + ", Y: " + std::to_string(msg->blue.yellow_cards);
  vis_annotation.normalized_x = CARDS_X;
  vis_annotation.normalized_y = 0.0;
  vis_annotation.normalized_width = CARDS_WIDTH;
  vis_annotation.normalized_height = TEXT_HEIGHT;
  vis_objects->annotations.push_back(vis_annotation);

  vis_annotation.color.name = COLOR_TEXT_YELLOW;
  vis_annotation.text = "R: " + std::to_string(msg->yellow.red_cards) +
                        ", Y: " + std::to_string(msg->yellow.yellow_cards);
  vis_annotation.normalized_x = CARDS_X;
  vis_annotation.normalized_y = TEXT_HEIGHT;
  vis_annotation.normalized_width = CARDS_WIDTH;
  vis_annotation.normalized_height = TEXT_HEIGHT;
  vis_objects->annotations.push_back(vis_annotation);

  // イエローカードの時間

  /*
  def parse_yellow_card_times(yellow_card_times):
    if len(yellow_card_times) == 0:
        return "NO CARDS"

    text = ""
    for i in range(len(yellow_card_times)):
        text += _microseconds_to_text(yellow_card_times[i])
        if i != len(yellow_card_times) - 1:
            text += "\n"

    return text
  */

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
  vis_annotation.color.name = COLOR_TEXT_BLUE;
  vis_annotation.text = parse_yellow_card_times(msg->blue.yellow_card_times);
  vis_annotation.normalized_x = YELLOW_CARD_TIMES_X;

  vis_annotation.normalized_y = 0.0;
  vis_annotation.normalized_width = YELLOW_CARD_TIMES_WIDTH;
  vis_annotation.normalized_height = TEXT_HEIGHT;
  vis_objects->annotations.push_back(vis_annotation);

  vis_annotation.color.name = COLOR_TEXT_YELLOW;
  vis_annotation.text = parse_yellow_card_times(msg->yellow.yellow_card_times);
  vis_annotation.normalized_x = YELLOW_CARD_TIMES_X;
  vis_annotation.normalized_y = TEXT_HEIGHT;
  vis_annotation.normalized_width = YELLOW_CARD_TIMES_WIDTH;
  vis_annotation.normalized_height = TEXT_HEIGHT;
  vis_objects->annotations.push_back(vis_annotation);

  // タイムアウト

  /*
  def parse_timeouts(timeouts, timeout_time):
    return 'Timeouts: {}\n {}'.format(
        timeouts, _microseconds_to_text(timeout_time))
  */

  auto parse_timeouts = [](const auto & timeouts, const auto & timeout_time) {
    return "Timeouts: " + std::to_string(timeouts) + "\n" + std::to_string(timeout_time);
  };
  vis_annotation.color.name = COLOR_TEXT_BLUE;
  vis_annotation.text = parse_timeouts(msg->blue.timeouts, msg->blue.timeout_time);
  vis_annotation.normalized_x = TIMEOUT_X;
  vis_annotation.normalized_y = 0.0;
  vis_annotation.normalized_width = TIMEOUT_WIDTH;
  vis_annotation.normalized_height = TEXT_HEIGHT;
  vis_objects->annotations.push_back(vis_annotation);

  vis_annotation.color.name = COLOR_TEXT_YELLOW;
  vis_annotation.text = parse_timeouts(msg->yellow.timeouts, msg->yellow.timeout_time);
  vis_annotation.normalized_x = TIMEOUT_X;
  vis_annotation.normalized_y = TEXT_HEIGHT;
  vis_annotation.normalized_width = TIMEOUT_WIDTH;
  vis_annotation.normalized_height = TEXT_HEIGHT;
  vis_objects->annotations.push_back(vis_annotation);

  // プレイスメント位置
  if (
    msg->command == Referee::COMMAND_BALL_PLACEMENT_BLUE ||
    msg->command == Referee::COMMAND_BALL_PLACEMENT_YELLOW) {
    if (not msg->designated_position.empty()) {
      VisCircle vis_circle;
      vis_circle.center.x = msg->designated_position.front().x;
      vis_circle.center.y = msg->designated_position.front().y;
      vis_circle.radius = 0.15;
      vis_circle.line_color.name = "aquamarine";
      vis_circle.fill_color.name = "aquamarine";
      vis_circle.line_size = 1;
      vis_circle.caption = "placement pos";
      vis_objects->circles.push_back(vis_circle);
    }
  }

  pub_vis_objects_->publish(std::move(vis_objects));
}
}  // namespace consai_vision_tracker
