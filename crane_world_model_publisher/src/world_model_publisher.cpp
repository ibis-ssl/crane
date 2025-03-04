// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_basics/geometry_operations.hpp>
#include <crane_basics/time.hpp>
#include <crane_world_model_publisher/world_model_publisher.hpp>
#include <deque>
#include <robocup_ssl_msgs/msg/robot_id.hpp>

namespace crane
{
WorldModelPublisherComponent::WorldModelPublisherComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("world_model_publisher", options), data_provider(*this)
{
  using std::chrono_literals::operator""ms;

  CraneVisualizerBuffer::activate(*this);
  visualizer =
    std::make_unique<crane::CraneVisualizerBuffer::MessageBuilder>("world_model/trajectory");

  declare_parameter("position_history_size", 200);
  get_parameter<int>("position_history_size", history_size);

  pub_process_time = create_publisher<std_msgs::msg::Float32>("~/process_time", 10);

  pub_world_model = create_publisher<crane_msgs::msg::WorldModel>("/world_model", 1);

  // 自動/world_modelサブスクライブはOFF
  wrapper = std::make_shared<WorldModelWrapper>(*this, false);

  using std::chrono::operator""ms;
  timer = rclcpp::create_timer(this, get_clock(), 16ms, [this]() {
    if (data_provider.available()) {
      publishWorldModel();
      publishVisualization();
    }
  });
}

// updateHistory
void WorldModelPublisherComponent::updateHistory(crane_msgs::msg::WorldModel & msg)
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

void WorldModelPublisherComponent::publishWorldModel()
{
  auto msg = data_provider.getMsg();
  updateHistory(msg);

  wrapper->update(msg);
  updateBallContact();
  postProcessWorldModel(wrapper);

  pub_world_model->publish(wrapper->getMsg());
}

void WorldModelPublisherComponent::publishVisualization()
{
  constexpr int SAMPLING_NUM = 4;
  for (const auto & [robot_id, history] : friend_history | ranges::views::enumerate) {
    if (history.size() > SAMPLING_NUM + 1 && history.front().detected) {
      for (int i = 0; i < 10; i++) {
        SvgPolyLineBuilder polyline_builder;
        int start = static_cast<int>((history.size() / 10.) * i);
        int end = static_cast<int>((history.size() / 10.) * (i + 1));
        for (int index = start; index < end; index += SAMPLING_NUM) {
          polyline_builder.addPoint(history.at(index).pose.x, history.at(index).pose.y);
        }
        if (i != 9) {
          polyline_builder.addPoint(history.at(end).pose.x, history.at(end).pose.y);
        }
        polyline_builder.stroke("yellow", start / static_cast<double>(history.size()))
          .strokeWidth(15);
        visualizer->add(polyline_builder.getSvgString());
      }
    }
  }

  for (const auto & [robot_id, history] : enemy_history | ranges::views::enumerate) {
    if (history.size() > SAMPLING_NUM + 1 && history.front().detected) {
      for (int i = 0; i < 10; i++) {
        SvgPolyLineBuilder polyline_builder;
        int start = static_cast<int>((history.size() / 10.) * i);
        int end = static_cast<int>((history.size() / 10.) * (i + 1));
        for (int index = start; index < end; index += SAMPLING_NUM) {
          polyline_builder.addPoint(history.at(index).pose.x, history.at(index).pose.y);
        }
        if (i != 9) {
          polyline_builder.addPoint(history.at(end).pose.x, history.at(end).pose.y);
        }
        polyline_builder.stroke("blue", start / static_cast<double>(history.size()))
          .strokeWidth(15);
        visualizer->add(polyline_builder.getSvgString());
      }
    }
  }

  if (ball_info_history.size() > SAMPLING_NUM + 1) {
    for (int i = 0; i < 10; i++) {
      SvgPolyLineBuilder polyline_builder;
      int start = static_cast<int>((ball_info_history.size() / 10.) * i);
      int end = static_cast<int>((ball_info_history.size() / 10.) * (i + 1));
      for (int index = start; index < end; index += SAMPLING_NUM) {
        polyline_builder.addPoint(
          ball_info_history.at(index).pose.x, ball_info_history.at(index).pose.y);
      }
      if (i != 9) {
        polyline_builder.addPoint(
          ball_info_history.at(end).pose.x, ball_info_history.at(end).pose.y);
      }
      polyline_builder.stroke("orange", start / static_cast<double>(ball_info_history.size()))
        .strokeWidth(30);
      visualizer->add(polyline_builder.getSvgString());
    }
  }

  data_provider.vis_data_handler.publish_vis_tracked(wrapper);
  visualizer->flush();
  CraneVisualizerBuffer::publish();
}

void WorldModelPublisherComponent::postProcessWorldModel(WorldModelWrapper::SharedPtr world_model)
{
  kick_event_detector.update(*world_model, visualizer);
  crane_msgs::msg::GameAnalysis game_analysis_msg;
  if (auto kick = kick_event_detector.getOnGoingKick(); kick.has_value()) {
    game_analysis_msg.ongoing_kick.push_back(*kick);
  }

  double ball_holizon = 10.;
  for (const auto & robot : wrapper->ours.getAvailableRobots()) {
    auto [min_slack, max_slack] = world_model->getMinMaxSlackInterceptPointAndSlackTime(
      {robot}, 3.0, 0.1, 0.5, 3.0, 5.0, ball_holizon);
    crane_msgs::msg::Slack slack_msg;
    slack_msg.id = robot->id;
    if (min_slack) {
      slack_msg.min.slack_time = min_slack->slack_time;
      slack_msg.min.x = min_slack->intercept_point.x();
      slack_msg.min.y = min_slack->intercept_point.y();

      SvgTextBuilder text_builder;
      text_builder.position(robot->pose.pos.x(), robot->pose.pos.y() - 0.3)
        .text("min slack: " + std::to_string(min_slack->slack_time))
        .fill("white")
        .fontSize(100);
      visualizer->add(text_builder.getSvgString());
      SvgLineBuilder line_builder;
      line_builder.start(robot->pose.pos)
        .end(min_slack->intercept_point)
        .stroke("red", 0.5)
        .strokeWidth(5);
      visualizer->add(line_builder.getSvgString());
    }
    if (max_slack) {
      slack_msg.max.slack_time = max_slack->slack_time;
      slack_msg.max.x = max_slack->intercept_point.x();
      slack_msg.max.y = max_slack->intercept_point.y();

      if (max_slack->slack_time > 0.) {
        SvgTextBuilder text_builder;
        text_builder.position(robot->pose.pos.x(), robot->pose.pos.y() - 0.5)
          .text("max slack: " + std::to_string(max_slack->slack_time))
          .fill("white")
          .fontSize(100);
        visualizer->add(text_builder.getSvgString());
        SvgLineBuilder line_builder;
        line_builder.start(robot->pose.pos)
          .end(max_slack->intercept_point)
          .stroke("red", 0.5)
          .strokeWidth(5);
        visualizer->add(line_builder.getSvgString());
      }
    }
    game_analysis_msg.our_slack.push_back(slack_msg);
  }

  for (const auto & robot : wrapper->theirs.getAvailableRobots()) {
    auto [min_slack, max_slack] = world_model->getMinMaxSlackInterceptPointAndSlackTime(
      {robot}, 3.0, 0.1, 0.5, 3.0, 5.0, ball_holizon);
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
  world_model->update(game_analysis_msg);
}

void WorldModelPublisherComponent::updateBallContact()
{
  auto now = rclcpp::Clock().now();

  // ローカルセンサーの情報でボール情報を更新
  auto friend_robots = wrapper->ours.getAvailableRobots();
  for (std::size_t i = 0; i < friend_robots.size(); i++) {
    auto robot = friend_robots[i];
    // ビジョンがボールを見失っているときに
    // ボールセンサが反応している間は、接触しているものとみなす。
    if (robot->getBallSensorAvailable(now) && not wrapper->ball.detected) {
      // ビジョンはボール見失っているけどロボットが保持しているので、
      // ロボットの座標にボールがあることにする
      wrapper->overwriteBallPos(robot->kicker_center());
    }
  }
}
}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::WorldModelPublisherComponent)
