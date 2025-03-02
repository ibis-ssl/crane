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
  updateBallContact();

  // wm.game_analysis = latest_game_analysis;

  wrapper->update(msg);
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
  visualizer->flush();
  CraneVisualizerBuffer::publish();
}

void WorldModelPublisherComponent::postProcessWorldModel(WorldModelWrapper::SharedPtr world_model)
{
}

void WorldModelPublisherComponent::updateBallContact()
{
  auto now = rclcpp::Clock().now();

  // ローカルセンサーの情報でボール情報を更新
  auto friend_robots = wrapper->ours.getAvailableRobots();
  // for (std::size_t i = 0; i < friend_robots.size(); i++) {
  //   auto robot = friend_robots[i];
  //   double ball_distance = robot->getDistance(wrapper->ball.pos);
  //   // ビジョンがボールを見失っているときにボールセンサが反応している間は、接触しているものとみなす。
  //   if (ball_sensor_detected[i] && friend_robots[i]->available && ball_info.disappeared) {
  //     // ビジョンはボール見失っているけどロボットが保持しているので、ロボットの座標にボールがあることにする
  //
  //     Point center_to_kicker = getNormVec(robot->pose.theta) * 0.09;
  //     ball_info.pose.x = robot->pose.pos.x() + center_to_kicker.x();
  //     ball_info.pose.y = robot->pose.pos.y() + center_to_kicker.y();
  //
  //     // robot->ball_contact.is_vision_source = false;
  //     // robot->ball_contact.current_time = now;
  //     // robot->ball_contact.last_contacted_time = now;
  //     if (not is_our_ball) {
  //       std::cout << "敵ボール接触" << std::endl;
  //       is_our_ball = true;
  //       ball_event_detected = true;
  //     }
  //   } else if (ball_sensor_detected[i] && robot->available && ball_distance < 0.3) {
  //     // robot->ball_contact.is_vision_source = false;
  //     // robot->ball_contact.current_time = now;
  //     // robot->ball_contact.last_contacted_time = now;
  //     if (not is_our_ball) {
  //       std::cout << "敵ボール接触" << std::endl;
  //       is_our_ball = true;
  //       ball_event_detected = true;
  //     }
  //   }
  // }
}
}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::WorldModelPublisherComponent)
