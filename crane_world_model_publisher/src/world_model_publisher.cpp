// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_basics/ddps.hpp>
#include <crane_basics/geometry_operations.hpp>
#include <crane_basics/time.hpp>
#include <crane_world_model_publisher/world_model_publisher.hpp>
#include <deque>
#include <robocup_ssl_msgs/msg/robot_id.hpp>

namespace crane
{
static auto parseStringToIntArray(const std::string & str) -> std::vector<uint8_t>
{
  std::vector<uint8_t> result;
  std::stringstream ss(str);
  int value;
  char comma;
  while (ss >> value) {
    result.push_back(static_cast<uint8_t>(value));
    // 次のカンマをスキップ（もしあれば）
    ss >> comma;
  }
  return result;
}

WorldModelPublisherComponent::WorldModelPublisherComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("world_model_publisher", options),
  data_provider(*this),
  pub_world_model(this, "/world_model", 1, 50., 70.)
{
  using std::chrono_literals::operator""ms;

  CraneVisualizerBuffer::activate(*this);
  traj_visualizer = std::make_unique<crane::VisualizerMessageBuilder>("world_model/trajectory");

  slack_visualizer = std::make_unique<crane::VisualizerMessageBuilder>("world_model/slack");

  pass_score_visualizer =
    std::make_unique<crane::VisualizerMessageBuilder>("world_model/pass_score");

  declare_parameter("position_history_size", 200);
  get_parameter<int>("position_history_size", history_size);

  // 練習用モードの設定
  bool half_court_practice_mode = false;
  bool half_court_is_positive_side = true;  // 使用している半面がポジティブ側かどうか
  declare_parameter("half_court_practice_mode", half_court_practice_mode);
  get_parameter("half_court_practice_mode", half_court_practice_mode);
  declare_parameter("half_court_is_positive_side", half_court_is_positive_side);
  get_parameter("half_court_is_positive_side", half_court_is_positive_side);

  // DataProviderにアフィン変換行列を渡す
  data_provider.setTransformInfo(half_court_practice_mode, half_court_is_positive_side);

  declare_parameter("robot_id_mask", std::string("1, 2, 3"));
  std::string robot_id_mask_str;
  get_parameter("robot_id_mask", robot_id_mask_str);
  data_provider.setRobotIDsMask(parseStringToIntArray(robot_id_mask_str));

  declare_parameter("robot_acc_for_prediction", 2.5);
  get_parameter("robot_acc_for_prediction", robot_acc_for_prediction);

  declare_parameter("robot_max_vel_for_prediction", 5.0);
  get_parameter("robot_max_vel_for_prediction", robot_max_vel_for_prediction);

  pub_process_time = create_publisher<std_msgs::msg::Float32>("~/process_time", 10);

  // 自動/world_modelサブスクライブはOFF
  wrapper = std::make_shared<WorldModelWrapper>(*this, false);

  using std::chrono::operator""ms;
  timer = rclcpp::create_timer(this, get_clock(), 16ms, [this]() {
    if (data_provider.available()) {
      publishWorldModel();
      publishVisualization(wrapper);
    }
  });
}

// updateHistory
auto WorldModelPublisherComponent::updateHistory(crane_msgs::msg::WorldModel & msg) -> void
{
  if (ball_info_history.size() >= history_size) {
    ball_info_history.pop_front();
  }
  ball_info_history.emplace_back(msg.ball_info);

  for (const auto & robot : msg.robot_info_ours) {
    if (robot.detected) {
      friend_history[robot.id].push_back(robot);
    }
    if (friend_history[robot.id].size() > history_size) {
      friend_history[robot.id].pop_front();
    }
  }

  for (const auto & robot : msg.robot_info_theirs) {
    if (robot.detected) {
      enemy_history[robot.id].push_back(robot);
    }
    if (enemy_history[robot.id].size() > history_size) {
      enemy_history[robot.id].pop_front();
    }
  }
}

auto WorldModelPublisherComponent::publishWorldModel() -> void
{
  auto msg = data_provider.getMsg();
  updateHistory(msg);

  wrapper->update(msg);
  updateBallContact();
  postProcessWorldModel(wrapper);

  pub_world_model.publish(wrapper->getMsg());
}

auto WorldModelPublisherComponent::publishVisualization(WorldModelWrapper::SharedPtr world_model)
  -> void
{
  constexpr int SAMPLING_NUM = 4;
  for (const auto & [robot_id, history] : friend_history | ranges::views::enumerate) {
    if (history.size() > SAMPLING_NUM + 1 && history.front().detected) {
      for (int i = 0; i < 10; i++) {
        int start = static_cast<int>((history.size() / 10.) * i);
        int end = static_cast<int>((history.size() / 10.) * (i + 1));

        auto builder = traj_visualizer->polyline();
        for (int index = start; index < end; index += SAMPLING_NUM) {
          builder.addPoint(history.at(index).pose.x, history.at(index).pose.y);
        }
        if (i != 9) {
          builder.addPoint(history.at(end).pose.x, history.at(end).pose.y);
        }
        builder
          .stroke(
            world_model->isYellow() ? "yellow" : "blue",
            start / static_cast<double>(history.size()))
          .strokeWidth(15)
          .build();
      }
    }
  }

  for (const auto & [robot_id, history] : enemy_history | ranges::views::enumerate) {
    if (history.size() > SAMPLING_NUM + 1 && history.front().detected) {
      for (int i = 0; i < 10; i++) {
        int start = static_cast<int>((history.size() / 10.) * i);
        int end = static_cast<int>((history.size() / 10.) * (i + 1));

        auto builder = traj_visualizer->polyline();
        for (int index = start; index < end; index += SAMPLING_NUM) {
          builder.addPoint(history.at(index).pose.x, history.at(index).pose.y);
        }
        if (i != 9) {
          builder.addPoint(history.at(end).pose.x, history.at(end).pose.y);
        }
        builder
          .stroke(
            world_model->isYellow() ? "blue" : "yellow",
            start / static_cast<double>(history.size()))
          .strokeWidth(15)
          .build();
      }
    }
  }

  if (ball_info_history.size() > SAMPLING_NUM + 1) {
    for (int i = 0; i < 10; i++) {
      int start = static_cast<int>((ball_info_history.size() / 10.) * i);
      int end = static_cast<int>((ball_info_history.size() / 10.) * (i + 1));

      auto builder = traj_visualizer->polyline();
      for (int index = start; index < end; index += SAMPLING_NUM) {
        builder.addPoint(
          ball_info_history.at(index).position.x, ball_info_history.at(index).position.y);
      }
      if (i != 9) {
        builder.addPoint(
          ball_info_history.at(end).position.x, ball_info_history.at(end).position.y);
      }
      builder.stroke("orange", start / static_cast<double>(ball_info_history.size()))
        .strokeWidth(30)
        .build();
    }
  }

  data_provider.vis_data_handler.flushTrackerVisualization(wrapper);
  traj_visualizer->flush();
  CraneVisualizerBuffer::publish();
}

auto WorldModelPublisherComponent::postProcessWorldModel(WorldModelWrapper::SharedPtr world_model)
  -> void
{
  kick_event_detector.update(*world_model, traj_visualizer);
  crane_msgs::msg::GameAnalysis game_analysis_msg;
  if (auto kick = kick_event_detector.getOnGoingKick(); kick.has_value()) {
    game_analysis_msg.ongoing_kick.push_back(*kick);
  }

  // ボールラインの長さを計算
  game_analysis_msg.ball_horizon = [&]() {
    auto future_ball = world_model->ball().getPositionAt(3.0);
    Segment ball_line{world_model->ball().pos, future_ball};
    auto robots = world_model->theirs().getAvailableRobots();
    auto ball_line_lengths =
      robots |
      ranges::views::transform(
        [&](const auto & robot) { return getClosestPointAndDistance(ball_line, robot->pose.pos); })
      // 距離が0.5m以下のものを抽出
      | ranges::views::filter([](const ClosestPoint & pair) { return pair.distance < 0.5; })
      // ball.posとの距離を計算
      | ranges::views::transform([&](const ClosestPoint & pair) -> double {
          return (pair.closest_point - world_model->ball().pos).norm();
        });
    return ranges::empty(ball_line_lengths) ? 10.0 : ranges::min(ball_line_lengths);
  }();

  for (const auto & robot : wrapper->ours().getAvailableRobots()) {
    auto [min_slack, max_slack] = world_model->getMinMaxSlackInterceptPointAndSlackTime(
      {robot}, 3.0, 0.1, 0.5, robot_acc_for_prediction, robot_max_vel_for_prediction,
      game_analysis_msg.ball_horizon);
    crane_msgs::msg::Slack slack_msg;
    slack_msg.id = robot->id;
    if (min_slack) {
      slack_msg.min.slack_time = min_slack->slack_time;
      slack_msg.min.x = min_slack->intercept_point.x();
      slack_msg.min.y = min_slack->intercept_point.y();

      slack_visualizer->text()
        .position(robot->pose.pos.x(), robot->pose.pos.y() - 0.3)
        .text("min slack: " + std::to_string(min_slack->slack_time))
        .fill("white")
        .fontSize(100)
        .build();
      slack_visualizer->line()
        .start(robot->pose.pos)
        .end(min_slack->intercept_point)
        .stroke("red", 0.5)
        .strokeWidth(5)
        .build();
    }
    if (max_slack) {
      slack_msg.max.slack_time = max_slack->slack_time;
      slack_msg.max.x = max_slack->intercept_point.x();
      slack_msg.max.y = max_slack->intercept_point.y();

      if (max_slack->slack_time > 0.) {
        slack_visualizer->text()
          .position(robot->pose.pos.x(), robot->pose.pos.y() - 0.5)
          .text("max slack: " + std::to_string(max_slack->slack_time))
          .fill("white")
          .fontSize(100)
          .build();
        slack_visualizer->line()
          .start(robot->pose.pos)
          .end(max_slack->intercept_point)
          .stroke("red", 0.5)
          .strokeWidth(5)
          .build();
      }
    }
    game_analysis_msg.our_slack.push_back(slack_msg);
  }

  for (const auto & robot : wrapper->theirs().getAvailableRobots()) {
    auto [min_slack, max_slack] = world_model->getMinMaxSlackInterceptPointAndSlackTime(
      {robot}, 3.0, 0.1, 0.5, robot_acc_for_prediction, robot_max_vel_for_prediction,
      game_analysis_msg.ball_horizon);
    crane_msgs::msg::Slack slack_msg;
    slack_msg.id = robot->id;
    if (min_slack) {
      slack_msg.min.slack_time = min_slack->slack_time;
      slack_msg.min.x = min_slack->intercept_point.x();
      slack_msg.min.y = min_slack->intercept_point.y();
    }
    if (max_slack) {
      slack_msg.max.slack_time = max_slack->slack_time;
      slack_msg.max.x = max_slack->intercept_point.x();
      slack_msg.max.y = max_slack->intercept_point.y();
    }
    game_analysis_msg.their_slack.push_back(slack_msg);
  }

  auto our_robots = world_model->ours().getAvailableRobots(true);
  const auto enemy_robots = world_model->theirs().getAvailableRobots();

  auto calc_score = [&](Point p) {
    Segment ball_to_target{world_model->ball().pos, p};
    double score = 1.0;
    // 0~4mで遠くなるほどスコアが高い
    score += std::clamp((p - world_model->ball().pos).norm() * 0.5, 0.0, 2.0);
    {
      // パス先のゴールチャンスが大きい場合はスコアを上げる(30度以上で最大0.5上昇)
      auto [best_angle, goal_angle_width] = world_model->getLargestGoalAngleRangeFromPoint(p);
      score += std::clamp(goal_angle_width / (M_PI / 12.), 0.0, 0.5);
    }
    {
      // パス先が自チームのゴールを脅かす場合はスコアを下げる(30度以上で最大0.5減少)
      auto [best_angle, goal_angle_width] =
        world_model->getLargestGoalAngleRangeFromPoint(p, world_model->getOurGoalPosts(), {});
      score -= std::clamp(goal_angle_width / (M_PI / 12.), 0.0, 0.5);
    }
    // 敵ゴールに近いときはスコアを上げる
    double normed_distance_to_their_goal =
      ((p - world_model->getTheirGoalCenter()).norm() - (world_model->fieldSize().x() * 0.5)) /
      (world_model->fieldSize().x() * 0.5);
    // マイナスのときはゴールに近い
    score *= (1.0 - normed_distance_to_their_goal * 0.5);
    if (auto nearest_enemy =
          world_model->getNearestRobotWithDistanceFromSegment(ball_to_target, enemy_robots);
        nearest_enemy) {
      // ボールから遠い敵がパスコースを塞いでいる場合は諦める
      if (
        nearest_enemy->robot->getDistance(world_model->ball().pos) > 1.0 &&
        nearest_enemy->distance < 0.4) {
        score = 0.0;
      }
      // パスラインに敵がいるときはスコアを下げる
      score *= 1.0 / (1.0 + nearest_enemy->distance);
    }

    if (world_model->point_checker.isPenaltyArea(p)) {
      score = 0.0;
    }
    return score;
  };

  constexpr double UNIT = 0.2;
  auto grid_points = getPoints(
    Point(0, 0), UNIT, UNIT, world_model->fieldSize().x() / UNIT,
    world_model->fieldSize().y() / UNIT);
  auto score_grid =
    grid_points |
    ranges::views::transform([&](const auto & p) { return std::make_pair(p, calc_score(p)); }) |
    ranges::to<std::vector>();

  ranges::for_each(score_grid, [&]([[maybe_unused]] const auto & pair) {
    //  pass_score_visualizer->circle().center(pair.first).
    //  radius(pair.second * 0.05).stroke("red").strokeWidth(2.).build();
  });
  pass_score_visualizer->flush();

  auto score_with_bots = our_robots | ranges::views::transform([&](const auto & robot) {
                           return std::make_pair(robot, calc_score(robot->pose.pos));
                         }) |
                         ranges::to<std::vector>();
  // larger score first
  ranges::sort(score_with_bots, [](const auto & a, const auto & b) { return a.second > b.second; });

  game_analysis_msg.pass_scores = score_with_bots | ranges::views::transform([](const auto & pair) {
                                    crane_msgs::msg::FloatWithID msg;
                                    return msg.set__id(pair.first->id).set__value(pair.second);
                                  }) |
                                  ranges::to<std::vector>();

  world_model->update(game_analysis_msg);
}

auto WorldModelPublisherComponent::updateBallContact() -> void
{
  auto now = rclcpp::Clock().now();

  // ローカルセンサーの情報でボール情報を更新
  auto friend_robots = wrapper->ours().getAvailableRobots();
  for (std::size_t i = 0; i < friend_robots.size(); i++) {
    auto robot = friend_robots[i];
    // ビジョンがボールを見失っているときに
    // ボールセンサが反応している間は、接触しているものとみなす。
    if (robot->getBallSensorAvailable(now) && not wrapper->ball().detected) {
      // ビジョンはボール見失っているけどロボットが保持しているので、
      // ロボットの座標にボールがあることにする
      wrapper->overwriteBallPos(robot->kicker_center());
    }
  }
}
}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::WorldModelPublisherComponent)
