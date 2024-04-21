// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_msgs/msg/play_situation.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <speak_ros_interfaces/action/speak.hpp>
#include <unordered_map>

typedef std::unordered_map<uint8_t, std::string> map;
map play_situation_map = {
  {crane_msgs::msg::PlaySituation::HALT, "ホールト"},
  {crane_msgs::msg::PlaySituation::STOP, "ストップ"},
  {crane_msgs::msg::PlaySituation::OUR_KICKOFF_PREPARATION, "味方キックオフ準備"},
  {crane_msgs::msg::PlaySituation::OUR_KICKOFF_START, "味方キックオフ開始"},
  {crane_msgs::msg::PlaySituation::OUR_PENALTY_PREPARATION, "味方PK準備"},
  {crane_msgs::msg::PlaySituation::OUR_PENALTY_START, "味方PK開始"},
  {crane_msgs::msg::PlaySituation::OUR_DIRECT_FREE, "味方フリーキック"},
  {crane_msgs::msg::PlaySituation::OUR_INDIRECT_FREE, "味方インダイレクトフリーキック"},
  {crane_msgs::msg::PlaySituation::OUR_BALL_PLACEMENT, "味方ボールプレイスメント"},
  {crane_msgs::msg::PlaySituation::THEIR_KICKOFF_PREPARATION, "敵キックオフ準備"},
  {crane_msgs::msg::PlaySituation::THEIR_KICKOFF_START, "敵キックオフ開始"},
  {crane_msgs::msg::PlaySituation::THEIR_PENALTY_PREPARATION, "敵PK準備"},
  {crane_msgs::msg::PlaySituation::THEIR_PENALTY_START, "敵PK開始"},
  {crane_msgs::msg::PlaySituation::THEIR_DIRECT_FREE, "敵フリーキック"},
  {crane_msgs::msg::PlaySituation::THEIR_INDIRECT_FREE, "敵インダイレクトフリーキック"},
  {crane_msgs::msg::PlaySituation::THEIR_BALL_PLACEMENT, "敵ボールプレイスメント"},
  {crane_msgs::msg::PlaySituation::OUR_INPLAY, "味方ボール"},
  {crane_msgs::msg::PlaySituation::THEIR_INPLAY, "敵ボール"},
  {crane_msgs::msg::PlaySituation::AMBIGUOUS_INPLAY, "ボール不明"}};

class SpeakClient : public rclcpp::Node
{
public:
  using Speak = speak_ros_interfaces::action::Speak;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Speak>;

  explicit SpeakClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("speak_client", node_options)
  {
    client = rclcpp_action::create_client<Speak>(
      get_node_base_interface(), get_node_graph_interface(), get_node_logging_interface(),
      get_node_waitables_interface(), "/speak");

    play_situation_sub = create_subscription<crane_msgs::msg::PlaySituation>(
      "/play_situation", 10, [this](const crane_msgs::msg::PlaySituation::SharedPtr msg) {
        if (play_situation_map.find(msg->command) != play_situation_map.end()) {
          sendGoal(play_situation_map[msg->command]);
        }
      });
  }

  void sendGoal(std::string text)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;

    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = Speak::Goal();
    goal_msg.text = text;
    goal_msg.speed_rate = 1.0;

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Speak>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&SpeakClient::goalResponseCallback, this, _1);
    send_goal_options.feedback_callback = std::bind(&SpeakClient::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&SpeakClient::resultCallback, this, _1);
    client->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Speak>::SharedPtr client;

  rclcpp::Subscription<crane_msgs::msg::PlaySituation>::SharedPtr play_situation_sub;

  void goalResponseCallback(GoalHandle::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedbackCallback(
    GoalHandle::SharedPtr, const std::shared_ptr<const Speak::Feedback> feedback)
  {
    switch (feedback->state) {
      case Speak::Feedback::GENERATING:
        RCLCPP_INFO(get_logger(), "Generating sound file...");
        break;
      case Speak::Feedback::PLAYING:
        RCLCPP_INFO(get_logger(), "Playing sound file...");
        break;
      default:
        RCLCPP_INFO(get_logger(), "Unknown state");
        break;
    }
  }

  void resultCallback(const GoalHandle::WrappedResult & result)
  {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "elapsed time[s] : " << rclcpp::Duration(result.result->elapsed_time).seconds());
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;
  auto speaker = std::make_shared<SpeakClient>(options);
  exe.add_node(speaker->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
}
