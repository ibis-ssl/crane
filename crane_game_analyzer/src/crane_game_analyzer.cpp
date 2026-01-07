// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <cmath>
#include <format>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <vector>

#include "crane_game_analyzer/game_analyzer.hpp"

namespace crane
{

namespace
{
// 脅威度 (0.0-1.0) を RGB 色に変換（緑→黄→赤）
std::string threatToColor(double threat_rating)
{
  double t = std::clamp(threat_rating, 0.0, 1.0);
  int r, g;
  if (t < 0.5) {
    r = static_cast<int>(510 * t);           // 0 → 255
    g = 200 + static_cast<int>(55 * t * 2);  // 200 → 255
  } else {
    r = 255;
    g = static_cast<int>(255 * (1.0 - (t - 0.5) * 2));  // 255 → 0
  }
  return std::format("rgb({},{},0)", r, g);
}
}  // namespace

GameAnalyzerComponent::GameAnalyzerComponent(const rclcpp::NodeOptions & options)
: Node("crane_game_analyzer", options),
  visualizer(std::make_shared<VisualizerMessageBuilder>("game_analyzer"))
{
  RCLCPP_INFO(get_logger(), "GameAnalyzer is constructed.");

  // パラメータの設定
  declare_parameter("ball_idle.threshold_duration", 5.0);
  declare_parameter("ball_idle.move_distance_threshold_meter", 0.05);
  declare_parameter("robot_collision.velocity_threshold", 1.0);
  declare_parameter("robot_collision.distance_threshold", 0.2);
  declare_parameter("robot_collision.time_window", 0.5);
  declare_parameter("our_team_name", "ibis");
  declare_parameter("their_team_name", "opponent");

  // パラメータの読み込み
  config.ball_idle.threshold_duration =
    rclcpp::Duration::from_seconds(get_parameter("ball_idle.threshold_duration").as_double());
  config.ball_idle.move_distance_threshold_meter =
    get_parameter("ball_idle.move_distance_threshold_meter").as_double();
  config.robot_collision.velocity_threshold =
    get_parameter("robot_collision.velocity_threshold").as_double();
  config.robot_collision.distance_threshold =
    get_parameter("robot_collision.distance_threshold").as_double();
  config.robot_collision.time_window = get_parameter("robot_collision.time_window").as_double();
  our_team_name_ = get_parameter("our_team_name").as_string();
  their_team_name_ = get_parameter("their_team_name").as_string();

  RCLCPP_DEBUG(
    get_logger(), "  - Velocity threshold: %.2f m/s", config.robot_collision.velocity_threshold);
  RCLCPP_DEBUG(
    get_logger(), "  - Distance threshold: %.2f m", config.robot_collision.distance_threshold);
  RCLCPP_DEBUG(get_logger(), "  - Time window: %.2f s", config.robot_collision.time_window);

  CraneVisualizerBuffer::activate(*this);

  world_model = std::make_unique<WorldModelWrapper>(*this);

  // 脅威評価結果のパブリッシャー
  game_analysis_pub_ = create_publisher<crane_msgs::msg::GameAnalysis>("game_analysis", 10);

  // RONARイベント検出器とパブリッシャーの初期化
  ronar_event_detector_ = std::make_unique<RonarEventDetector>(get_clock());
  ronar_events_pub_ = create_publisher<crane_msgs::msg::RonarEvent>("ronar_events", 10);

  // イベントメモリの初期化
  event_memory_ = std::make_unique<EventMemory>(get_clock());

  // PlaySituation トピックの購読
  play_situation_sub_ = create_subscription<crane_msgs::msg::PlaySituation>(
    "/play_situation", 10,
    [this](const crane_msgs::msg::PlaySituation & msg) { onPlaySituationChanged(msg); });

  // GameEvent (autoref) トピックの購読
  game_event_sub_ = create_subscription<crane_msgs::msg::GameEvent>(
    "/game_event", 10, [this](const crane_msgs::msg::GameEvent & msg) { onGameEvent(msg); });

  world_model->addCallback([&]() {
    auto robot_collision_info = getRobotCollisionInfo();

    if (robot_collision_info) {
      //          robot_collision_info->attack_robot.robot_id
      RCLCPP_DEBUG(
        get_logger(), "Collision Detected : ( %d, %d ) , %f [m/s]",
        robot_collision_info->attack_robot.id, robot_collision_info->attacked_robot.id,
        robot_collision_info->relative_velocity);
    }

    // 脅威評価を実行
    auto analysis = evaluateThreats();
    game_analysis_pub_->publish(analysis);

    // RONARイベント検出を実行
    detectAndPublishRonarEvents();

    visualizer->flush();
    CraneVisualizerBuffer::publish();
  });
}

auto GameAnalyzerComponent::evaluateThreats() -> crane_msgs::msg::GameAnalysis
{
  crane_msgs::msg::GameAnalysis msg;

  // ボール脅威の計算
  auto ball_threat = threat_evaluator_.calculateBallThreat(*world_model);
  msg.ball_threat = threat_evaluator_.toThreatInfoMsg(ball_threat);

  // ロボット脅威の計算（優先度順）
  auto robot_threats = threat_evaluator_.calculateRobotThreats(*world_model, ball_threat);
  for (const auto & threat : robot_threats) {
    msg.robot_threats.push_back(threat_evaluator_.toThreatInfoMsg(threat));
  }

  // 推奨守備者数
  int available_robots = static_cast<int>(world_model->ours().getAvailableRobots().size());
  msg.recommended_num_defenders = static_cast<uint8_t>(
    threat_evaluator_.calculateRecommendedDefenders(ball_threat, robot_threats, available_robots));

  // 脅威の可視化
  // ボール脅威ライン（オレンジ）
  visualizer->line()
    .fromSegment(ball_threat.threat_line)
    .stroke("orange", 0.9)
    .strokeWidth(3)
    .build();

  // ボール脅威の防御ライン（シアン）
  if (ball_threat.protection_line) {
    visualizer->line()
      .fromSegment(*ball_threat.protection_line)
      .stroke("cyan", 0.8)
      .strokeWidth(2)
      .build();
    // 防御ライン端点のマーカー
    visualizer->drawStyledCircle(
      ball_threat.protection_line->first, 0.03, "cyan", 0.5, "cyan", 1.0, 2);
    visualizer->drawStyledCircle(
      ball_threat.protection_line->second, 0.03, "cyan", 0.5, "cyan", 1.0, 2);
  }

  // ロボット脅威（上位5つを可視化）
  int vis_count = 0;
  for (const auto & threat : robot_threats) {
    if (vis_count >= 5) break;

    // 脅威度に応じたグラデーション色
    std::string color = threatToColor(threat.threat_rating);

    // 線の太さも脅威度に応じて変化 (1.0 - 4.0)
    double line_width = 1.0 + threat.threat_rating * 3.0;

    // 不透明度も脅威度に応じて変化 (0.4 - 1.0)
    double opacity = 0.4 + threat.threat_rating * 0.6;

    // 脅威ライン
    visualizer->line()
      .fromSegment(threat.threat_line)
      .stroke(color, opacity)
      .strokeWidth(line_width)
      .build();

    // 脅威スコア表示
    std::string score_text = std::to_string(threat.threat_rating).substr(0, 4);
    visualizer->drawCenteredLabel(threat.robot->pose.pos + Vector2(0, 0.15), score_text, color, 30);

    // 順位表示
    std::string rank_text = "#" + std::to_string(vis_count + 1);
    visualizer->drawCenteredLabel(
      threat.robot->pose.pos + Vector2(-0.12, 0.15), rank_text, color, 20);

    // 防御ライン（存在する場合）
    if (threat.protection_line) {
      visualizer->line()
        .fromSegment(*threat.protection_line)
        .stroke("cyan", 0.6)
        .strokeWidth(1.5)
        .build();
    }

    vis_count++;
  }

  // 上位脅威のリダイレクト角度を可視化（上位2つのみ）
  if (!robot_threats.empty()) {
    for (size_t i = 0; i < std::min(size_t(2), robot_threats.size()); ++i) {
      const auto & threat = robot_threats[i];
      Point ball_pos = world_model->ball().pos;
      Point goal_center = world_model->getOurGoalCenter();
      Point threat_pos = threat.robot->pose.pos;

      // ボール→脅威→ゴール のリダイレクト角度を弧で表示
      Vector2 from_ball = (threat_pos - ball_pos).normalized();
      Vector2 to_goal = (goal_center - threat_pos).normalized();

      double angle1 = std::atan2(-from_ball.y(), -from_ball.x());
      double angle2 = std::atan2(to_goal.y(), to_goal.x());

      // 角度が大きすぎる場合はスキップ
      double angle_diff = std::abs(angle2 - angle1);
      if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
      if (angle_diff < M_PI) {
        std::string arc_color = threatToColor(threat.threat_rating);
        visualizer->arc(
          threat_pos, 0.15, std::min(angle1, angle2), std::max(angle1, angle2), arc_color, 1.5, 8);
      }
    }
  }

  // 推奨守備者数を画面左上に表示
  std::string def_text = "DEF:" + std::to_string(msg.recommended_num_defenders);
  visualizer->text().viewBoxPosition(3, -97).text(def_text).fill("cyan").fontSize(100).build();

  return msg;
}

auto GameAnalyzerComponent::detectAndPublishRonarEvents() -> void
{
  auto events = ronar_event_detector_->detect(*world_model);

  for (const auto & event : events) {
    // イベントをメモリに保存
    event_memory_->processEvent(event);

    // イベントをパブリッシュ
    ronar_events_pub_->publish(event);

    // イベントタイプに応じたログ出力
    std::string event_name;
    switch (event.event_type) {
      case crane_msgs::msg::RonarEvent::EVENT_POSSESSION_CHANGE:
        event_name = "POSSESSION_CHANGE";
        break;
      case crane_msgs::msg::RonarEvent::EVENT_GOAL:
        event_name = "GOAL";
        break;
      case crane_msgs::msg::RonarEvent::EVENT_SHOT:
        event_name = "SHOT";
        break;
      case crane_msgs::msg::RonarEvent::EVENT_FAST_SHOT:
        event_name = "FAST_SHOT";
        break;
      case crane_msgs::msg::RonarEvent::EVENT_BALL_OUT:
        event_name = "BALL_OUT";
        break;
      case crane_msgs::msg::RonarEvent::EVENT_SET_PLAY:
        event_name = "SET_PLAY";
        break;
      default:
        event_name = "UNKNOWN";
        break;
    }

    RCLCPP_INFO(
      get_logger(), "RONAR Event: %s at (%.2f, %.2f), speed=%.2f m/s", event_name.c_str(),
      event.position.x, event.position.y, event.ball_speed);
  }
}

auto GameAnalyzerComponent::onPlaySituationChanged(const crane_msgs::msg::PlaySituation & msg)
  -> void
{
  // チーム名の動的更新（空文字列でない場合のみ）
  if (!msg.our_team_info.name.empty()) {
    our_team_name_ = msg.our_team_info.name;
  }
  if (!msg.their_team_info.name.empty()) {
    their_team_name_ = msg.their_team_info.name;
  }

  uint8_t current = static_cast<uint8_t>(msg.command.value);

  // 初回は状態を保存するのみ
  if (!last_play_situation_command_) {
    last_play_situation_command_ = current;
    return;
  }

  uint8_t last = *last_play_situation_command_;

  // 状態が変化していない場合はスキップ
  if (current == last) {
    return;
  }

  std::optional<crane_msgs::msg::RonarEvent> event;

  // HALT (0)
  if (current == 0 && last != 0) {
    event = createPlaySituationEvent(crane_msgs::msg::RonarEvent::EVENT_HALT, msg);
    event->metadata_json =
      std::format(R"({{"previous_command": {}, "reason": "{}"}})", last, msg.reason_text);
  }
  // STOP (1, 60-66)
  else if (
    (current == 1 || (current >= 60 && current <= 66)) &&
    !(last == 1 || (last >= 60 && last <= 66))) {
    event = createPlaySituationEvent(crane_msgs::msg::RonarEvent::EVENT_STOP, msg);
    std::string stop_type = (current >= 60 && current <= 66) ? "STOP_PRE" : "STOP";
    int32_t next_cmd = msg.next_command_raw.empty() ? -1 : msg.next_command_raw[0].value;
    event->metadata_json = std::format(
      R"({{"stop_type": "{}", "command_value": {}, "next_command": {}, "reason": "{}"}})",
      stop_type, current, next_cmd, msg.reason_text);
  }
  // INPLAY (50-53)
  else if ((current >= 50 && current <= 53) && !(last >= 50 && last <= 53)) {
    event = createPlaySituationEvent(crane_msgs::msg::RonarEvent::EVENT_INPLAY_START, msg);
    std::string inplay_type;
    switch (current) {
      case 50:
        inplay_type = "INPLAY";
        break;
      case 51:
        inplay_type = "OUR_INPLAY";
        break;
      case 52:
        inplay_type = "THEIR_INPLAY";
        break;
      case 53:
        inplay_type = "AMBIGUOUS_INPLAY";
        break;
      default:
        inplay_type = "UNKNOWN";
        break;
    }
    event->metadata_json =
      std::format(R"({{"inplay_type": "{}", "previous_command": {}}})", inplay_type, last);
  }
  // TIMEOUT (17, 27)
  else if ((current == 17 || current == 27) && last != 17 && last != 27) {
    event = createPlaySituationEvent(crane_msgs::msg::RonarEvent::EVENT_TIMEOUT, msg);
    bool is_ours = (current == 17);
    const auto & team_info = is_ours ? msg.our_team_info : msg.their_team_info;
    event->metadata_json = std::format(
      R"({{"team": "{}", "timeouts_left": {}}})", is_ours ? "ours" : "theirs", team_info.timeouts);
  }
  // HALF_TIME (100)
  else if (current == 100 && last != 100) {
    event = createPlaySituationEvent(crane_msgs::msg::RonarEvent::EVENT_HALF_TIME, msg);
    event->metadata_json = std::format(
      R"({{"our_score": {}, "their_score": {}, "stage": "{}"}})", msg.our_team_info.score,
      msg.their_team_info.score, msg.stage.name);
  }
  // GAME_END (101)
  else if (current == 101 && last != 101) {
    event = createPlaySituationEvent(crane_msgs::msg::RonarEvent::EVENT_GAME_END, msg);
    std::string result;
    if (msg.our_team_info.score > msg.their_team_info.score) {
      result = "WIN";
    } else if (msg.our_team_info.score < msg.their_team_info.score) {
      result = "LOSE";
    } else {
      result = "DRAW";
    }
    event->metadata_json = std::format(
      R"({{"our_score": {}, "their_score": {}, "result": "{}"}})", msg.our_team_info.score,
      msg.their_team_info.score, result);
  }

  // イベントが生成された場合はパブリッシュ
  if (event) {
    event_memory_->processEvent(*event);
    ronar_events_pub_->publish(*event);

    // イベント名を取得してログ出力
    std::string event_name;
    switch (event->event_type) {
      case crane_msgs::msg::RonarEvent::EVENT_HALT:
        event_name = "HALT";
        break;
      case crane_msgs::msg::RonarEvent::EVENT_STOP:
        event_name = "STOP";
        break;
      case crane_msgs::msg::RonarEvent::EVENT_INPLAY_START:
        event_name = "INPLAY_START";
        break;
      case crane_msgs::msg::RonarEvent::EVENT_TIMEOUT:
        event_name = "TIMEOUT";
        break;
      case crane_msgs::msg::RonarEvent::EVENT_HALF_TIME:
        event_name = "HALF_TIME";
        break;
      case crane_msgs::msg::RonarEvent::EVENT_GAME_END:
        event_name = "GAME_END";
        break;
      default:
        event_name = "UNKNOWN";
        break;
    }

    RCLCPP_INFO(
      get_logger(), "PlaySituation Event: %s, metadata: %s", event_name.c_str(),
      event->metadata_json.c_str());
  }

  // 状態を更新
  last_play_situation_command_ = current;
}

auto GameAnalyzerComponent::createPlaySituationEvent(
  uint8_t event_type, const crane_msgs::msg::PlaySituation & msg) -> crane_msgs::msg::RonarEvent
{
  crane_msgs::msg::RonarEvent event;
  event.header.stamp = now();
  event.event_type = event_type;
  event.position.x = msg.placement_position.x;
  event.position.y = msg.placement_position.y;
  event.position.z = 0.0;
  event.ball_speed = 0.0f;
  event.confidence = 1.0f;
  event.has_primary_robot = false;
  event.has_secondary_robot = false;
  return event;
}

auto GameAnalyzerComponent::onGameEvent(const crane_msgs::msg::GameEvent & msg) -> void
{
  crane_msgs::msg::RonarEvent event;
  event.header.stamp = now();
  event.confidence = 1.0f;
  event.ball_speed = 0.0f;

  // イベントタイプに応じた変換
  if (msg.event_type == "GOAL" || msg.event_type == "POSSIBLE_GOAL") {
    event.event_type = crane_msgs::msg::RonarEvent::EVENT_GOAL;
  } else if (msg.event_type == "BALL_LEFT_FIELD_TOUCH_LINE" ||
             msg.event_type == "BALL_LEFT_FIELD_GOAL_LINE" ||
             msg.event_type == "AIMLESS_KICK" ||
             msg.event_type == "BOUNDARY_CROSSING") {
    event.event_type = crane_msgs::msg::RonarEvent::EVENT_BALL_OUT;
  } else {
    // その他のイベントはFOULとして処理
    event.event_type = crane_msgs::msg::RonarEvent::EVENT_FOUL;
  }

  // 位置情報（存在する場合）
  if (!msg.position_values.empty()) {
    event.position.x = msg.position_values[0].x;
    event.position.y = msg.position_values[0].y;
    event.position.z = 0.0;
  }

  // ロボット情報
  if (msg.robot_id > 0) {
    event.has_primary_robot = true;
    event.primary_robot_id = static_cast<uint8_t>(msg.robot_id);
    // チーム判定: world_model から自チームカラーを取得
    event.primary_robot_is_ours = (msg.team == (world_model->isYellow() ? "YELLOW" : "BLUE"));
  } else {
    event.has_primary_robot = false;
  }
  event.has_secondary_robot = false;

  // チーム名の変換（YELLOW/BLUE → チームキー）
  std::string team_name = "unknown";
  bool is_our_team_event = (msg.team == (world_model->isYellow() ? "YELLOW" : "BLUE"));
  if (is_our_team_event) {
    team_name = our_team_name_;
  } else if (msg.team == (world_model->isYellow() ? "BLUE" : "YELLOW")) {
    team_name = their_team_name_;
  }

  // メタデータ生成（チーム名と詳細情報を含む）
  std::string origin_str = msg.origin.empty() ? "" : msg.origin[0];
  std::ostringstream metadata;
  metadata << R"({"event_type": ")" << msg.event_type << R"(", "team": ")" << team_name
           << R"(", "origin": ")" << origin_str << R"(")";

  // 追加の浮動小数点値
  for (const auto & fval : msg.float_values) {
    metadata << R"(, ")" << fval.name << R"(": )" << fval.value;
  }

  // 追加の文字列値
  for (const auto & sval : msg.string_values) {
    metadata << R"(, ")" << sval.name << R"(": ")" << sval.value << R"(")";
  }

  // 追加の位置情報
  for (const auto & pval : msg.position_values) {
    metadata << R"(, ")" << pval.name << R"(": {"x": )" << pval.x << R"(, "y": )" << pval.y << "}";
  }

  metadata << "}";
  event.metadata_json = metadata.str();

  // イベントをパブリッシュとメモリ保存
  // 重複チェックは game_controller_component で実施済み
  event_memory_->processEvent(event);
  ronar_events_pub_->publish(event);

  // イベント名を取得してログ出力
  std::string event_name;
  switch (event.event_type) {
    case crane_msgs::msg::RonarEvent::EVENT_GOAL:
      event_name = "GOAL (autoref)";
      break;
    case crane_msgs::msg::RonarEvent::EVENT_BALL_OUT:
      event_name = "BALL_OUT (autoref)";
      break;
    case crane_msgs::msg::RonarEvent::EVENT_FOUL:
      event_name = "FOUL";
      break;
    default:
      event_name = "UNKNOWN";
      break;
  }

  RCLCPP_INFO(
    get_logger(), "Autoref Event: %s (%s), team=%s, robot=%d",
    event_name.c_str(), msg.event_type.c_str(), msg.team.c_str(), msg.robot_id);
}

}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crane::GameAnalyzerComponent)
