// Copyright 2021 Roots
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

#include "robocup_ssl_comm/game_controller_component.hpp"

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace robocup_ssl_comm
{
GameController::GameController(const rclcpp::NodeOptions & options)
: Node("game_controller", options)
{
  declare_parameter("multicast_address", "224.5.23.1");
  declare_parameter("multicast_port", 10003);
  receiver = std::make_unique<multicast::MulticastReceiver>(
    get_parameter("multicast_address").get_value<std::string>(),
    get_parameter("multicast_port").get_value<int>());
  pub_referee = create_publisher<robocup_ssl_msgs::msg::Referee>("referee", 10);
  pub_game_event = create_publisher<crane_msgs::msg::GameEvent>("game_event", 10);
  timer = rclcpp::create_timer(this, get_clock(), 25ms, std::bind(&GameController::on_timer, this));
}

void GameController::on_timer()
{
  while (receiver->available()) {
    std::vector<char> buf(2048);
    const size_t size = receiver->receive(buf);

    if (size > 0) {
      robocup_ssl::Referee packet;
      packet.ParseFromString(std::string(buf.begin(), buf.end()));

      // Use proto2ros Convert API to convert protobuf message to ROS message
      auto referee_msg = std::make_unique<robocup_ssl_msgs::msg::Referee>();
      robocup_ssl_msgs::conversions::Convert(packet, referee_msg.get());

      pub_referee->publish(std::move(referee_msg));

      // Process game events
      for (const auto & proto_event : packet.game_events()) {
        auto event_msg = std::make_unique<crane_msgs::msg::GameEvent>();
        *event_msg = parse_game_event(proto_event);
        pub_game_event->publish(std::move(event_msg));
      }
    }
  }
}

crane_msgs::msg::GameEvent GameController::parse_game_event(
  const robocup_ssl::GameEvent & proto_event)
{
  crane_msgs::msg::GameEvent event_msg;

  // Set event type based on proto type enum
  switch (proto_event.type()) {
    case robocup_ssl::GameEvent::BALL_LEFT_FIELD_TOUCH_LINE:
      event_msg.event_type = "BALL_LEFT_FIELD_TOUCH_LINE";
      break;
    case robocup_ssl::GameEvent::BALL_LEFT_FIELD_GOAL_LINE:
      event_msg.event_type = "BALL_LEFT_FIELD_GOAL_LINE";
      break;
    case robocup_ssl::GameEvent::AIMLESS_KICK:
      event_msg.event_type = "AIMLESS_KICK";
      break;
    case robocup_ssl::GameEvent::GOAL:
      event_msg.event_type = "GOAL";
      break;
    case robocup_ssl::GameEvent::POSSIBLE_GOAL:
      event_msg.event_type = "POSSIBLE_GOAL";
      break;
    case robocup_ssl::GameEvent::INVALID_GOAL:
      event_msg.event_type = "INVALID_GOAL";
      break;
    case robocup_ssl::GameEvent::BOT_KICKED_BALL_TOO_FAST:
      event_msg.event_type = "BOT_KICKED_BALL_TOO_FAST";
      break;
    case robocup_ssl::GameEvent::BOT_DRIBBLED_BALL_TOO_FAR:
      event_msg.event_type = "BOT_DRIBBLED_BALL_TOO_FAR";
      break;
    case robocup_ssl::GameEvent::ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA:
      event_msg.event_type = "ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA";
      break;
    case robocup_ssl::GameEvent::DEFENDER_IN_DEFENSE_AREA:
      event_msg.event_type = "DEFENDER_IN_DEFENSE_AREA";
      break;
    case robocup_ssl::GameEvent::BOUNDARY_CROSSING:
      event_msg.event_type = "BOUNDARY_CROSSING";
      break;
    case robocup_ssl::GameEvent::KEEPER_HELD_BALL:
      event_msg.event_type = "KEEPER_HELD_BALL";
      break;
    case robocup_ssl::GameEvent::BOT_PUSHED_BOT:
      event_msg.event_type = "BOT_PUSHED_BOT";
      break;
    case robocup_ssl::GameEvent::BOT_HELD_BALL_DELIBERATELY:
      event_msg.event_type = "BOT_HELD_BALL_DELIBERATELY";
      break;
    case robocup_ssl::GameEvent::BOT_TIPPED_OVER:
      event_msg.event_type = "BOT_TIPPED_OVER";
      break;
    case robocup_ssl::GameEvent::ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA:
      event_msg.event_type = "ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA";
      break;
    case robocup_ssl::GameEvent::BOT_CRASH_UNIQUE:
      event_msg.event_type = "BOT_CRASH_UNIQUE";
      break;
    case robocup_ssl::GameEvent::BOT_CRASH_DRAWN:
      event_msg.event_type = "BOT_CRASH_DRAWN";
      break;
    case robocup_ssl::GameEvent::DEFENDER_TOO_CLOSE_TO_KICK_POINT:
      event_msg.event_type = "DEFENDER_TOO_CLOSE_TO_KICK_POINT";
      break;
    case robocup_ssl::GameEvent::BOT_TOO_FAST_IN_STOP:
      event_msg.event_type = "BOT_TOO_FAST_IN_STOP";
      break;
    case robocup_ssl::GameEvent::BOT_INTERFERED_PLACEMENT:
      event_msg.event_type = "BOT_INTERFERED_PLACEMENT";
      break;
    case robocup_ssl::GameEvent::ATTACKER_DOUBLE_TOUCHED_BALL:
      event_msg.event_type = "ATTACKER_DOUBLE_TOUCHED_BALL";
      break;
    case robocup_ssl::GameEvent::PLACEMENT_SUCCEEDED:
      event_msg.event_type = "PLACEMENT_SUCCEEDED";
      break;
    case robocup_ssl::GameEvent::PENALTY_KICK_FAILED:
      event_msg.event_type = "PENALTY_KICK_FAILED";
      break;
    case robocup_ssl::GameEvent::NO_PROGRESS_IN_GAME:
      event_msg.event_type = "NO_PROGRESS_IN_GAME";
      break;
    case robocup_ssl::GameEvent::PLACEMENT_FAILED:
      event_msg.event_type = "PLACEMENT_FAILED";
      break;
    case robocup_ssl::GameEvent::MULTIPLE_CARDS:
      event_msg.event_type = "MULTIPLE_CARDS";
      break;
    case robocup_ssl::GameEvent::MULTIPLE_FOULS:
      event_msg.event_type = "MULTIPLE_FOULS";
      break;
    case robocup_ssl::GameEvent::BOT_SUBSTITUTION:
      event_msg.event_type = "BOT_SUBSTITUTION";
      break;
    case robocup_ssl::GameEvent::TOO_MANY_ROBOTS:
      event_msg.event_type = "TOO_MANY_ROBOTS";
      break;
    case robocup_ssl::GameEvent::CHALLENGE_FLAG:
      event_msg.event_type = "CHALLENGE_FLAG";
      break;
    case robocup_ssl::GameEvent::EMERGENCY_STOP:
      event_msg.event_type = "EMERGENCY_STOP";
      break;
    case robocup_ssl::GameEvent::UNSPORTING_BEHAVIOR_MINOR:
      event_msg.event_type = "UNSPORTING_BEHAVIOR_MINOR";
      break;
    case robocup_ssl::GameEvent::UNSPORTING_BEHAVIOR_MAJOR:
      event_msg.event_type = "UNSPORTING_BEHAVIOR_MAJOR";
      break;
    default:
      event_msg.event_type = "UNKNOWN_GAME_EVENT_TYPE";
      break;
  }

  // Set origins
  for (const auto & origin : proto_event.origin()) {
    event_msg.origin.push_back(origin);
  }

  // Parse event-specific data based on event type

  if (proto_event.has_ball_left_field_touch_line()) {
    const auto & event = proto_event.ball_left_field_touch_line();
    event_msg.team = (event.by_team() == robocup_ssl::Team::YELLOW) ? "YELLOW" : "BLUE";
    if (event.has_by_bot()) {
      event_msg.robot_id = event.by_bot();
    }
    if (event.has_location()) {
      crane_msgs::msg::NamedPosition pos;
      pos.name = "location";
      pos.x = event.location().x();
      pos.y = event.location().y();
      event_msg.position_values.push_back(pos);
    }
  } else if (proto_event.has_ball_left_field_goal_line()) {
    const auto & event = proto_event.ball_left_field_goal_line();
    event_msg.team = (event.by_team() == robocup_ssl::Team::YELLOW) ? "YELLOW" : "BLUE";
    if (event.has_by_bot()) {
      event_msg.robot_id = event.by_bot();
    }
    if (event.has_location()) {
      crane_msgs::msg::NamedPosition pos;
      pos.name = "location";
      pos.x = event.location().x();
      pos.y = event.location().y();
      event_msg.position_values.push_back(pos);
    }
  } else if (proto_event.has_aimless_kick()) {
    const auto & event = proto_event.aimless_kick();
    event_msg.team = (event.by_team() == robocup_ssl::Team::YELLOW) ? "YELLOW" : "BLUE";
    if (event.has_by_bot()) {
      event_msg.robot_id = event.by_bot();
    }
    if (event.has_location()) {
      crane_msgs::msg::NamedPosition pos;
      pos.name = "location";
      pos.x = event.location().x();
      pos.y = event.location().y();
      event_msg.position_values.push_back(pos);
    }
    if (event.has_kick_location()) {
      crane_msgs::msg::NamedPosition pos;
      pos.name = "kick_location";
      pos.x = event.kick_location().x();
      pos.y = event.kick_location().y();
      event_msg.position_values.push_back(pos);
    }
  } else if (proto_event.has_goal()) {
    const auto & event = proto_event.goal();
    event_msg.team = (event.by_team() == robocup_ssl::Team::YELLOW) ? "YELLOW" : "BLUE";
    if (event.has_kicking_bot()) {
      event_msg.robot_id = event.kicking_bot();
    }
    if (event.has_location()) {
      crane_msgs::msg::NamedPosition pos;
      pos.name = "location";
      pos.x = event.location().x();
      pos.y = event.location().y();
      event_msg.position_values.push_back(pos);
    }
    if (event.has_kick_location()) {
      crane_msgs::msg::NamedPosition pos;
      pos.name = "kick_location";
      pos.x = event.kick_location().x();
      pos.y = event.kick_location().y();
      event_msg.position_values.push_back(pos);
    }
    if (event.has_max_ball_height()) {
      crane_msgs::msg::NamedFloat val;
      val.name = "max_ball_height";
      val.value = event.max_ball_height();
      event_msg.float_values.push_back(val);
    }
    if (event.has_message()) {
      crane_msgs::msg::NamedString str;
      str.name = "message";
      str.value = event.message();
      event_msg.string_values.push_back(str);
    }
  } else if (proto_event.has_bot_kicked_ball_too_fast()) {
    const auto & event = proto_event.bot_kicked_ball_too_fast();
    event_msg.team = (event.by_team() == robocup_ssl::Team::YELLOW) ? "YELLOW" : "BLUE";
    if (event.has_by_bot()) {
      event_msg.robot_id = event.by_bot();
    }
    if (event.has_location()) {
      crane_msgs::msg::NamedPosition pos;
      pos.name = "location";
      pos.x = event.location().x();
      pos.y = event.location().y();
      event_msg.position_values.push_back(pos);
    }
    if (event.has_initial_ball_speed()) {
      crane_msgs::msg::NamedFloat val;
      val.name = "initial_ball_speed";
      val.value = event.initial_ball_speed();
      event_msg.float_values.push_back(val);
    }
    if (event.has_chipped()) {
      crane_msgs::msg::NamedBool val;
      val.name = "chipped";
      val.value = event.chipped();
      event_msg.bool_values.push_back(val);
    }
  }
  // Add more event type parsing as needed...

  return event_msg;
}

}  // namespace robocup_ssl_comm

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(robocup_ssl_comm::GameController)
