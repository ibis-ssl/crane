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

#include "robocup_ssl_comm/tracker_component.hpp"

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace robocup_ssl_comm
{
VisionTracker::VisionTracker(const rclcpp::NodeOptions & options) : Node("vision", options)
{
  declare_parameter("tracker_address", "224.5.23.2");
  declare_parameter("tracker_port", 11010);
  tracker_receiver = std::make_unique<multicast::MulticastReceiver>(
    get_parameter("tracker_address").get_value<std::string>(),
    get_parameter("tracker_port").get_value<int>());
  declare_parameter("vision_address", "224.5.23.2");
  declare_parameter("vision_port", 10006);
  geometry_receiver = std::make_unique<multicast::MulticastReceiver>(
    get_parameter("vision_address").get_value<std::string>(),
    get_parameter("vision_port").get_value<int>());
  pub_tracked_frame =
    create_publisher<robocup_ssl_msgs::msg::TrackedFrame>("detection_tracked", 10);
  pub_geometry = create_publisher<robocup_ssl_msgs::msg::GeometryData>("geometry", 10);

  timer = rclcpp::create_timer(this, get_clock(), 10ms, std::bind(&VisionTracker::on_timer, this));
}

void VisionTracker::on_timer()
{
  while (tracker_receiver->available()) {
    std::vector<char> buf(2048);
    const size_t size = tracker_receiver->receive(buf);

    if (size > 0) {
      TrackerWrapperPacket packet;
      packet.ParseFromString(std::string(buf.begin(), buf.end()));

      if (packet.has_tracked_frame()) {
        publish_detection(packet.tracked_frame());
      }
    }
  }

  while (geometry_receiver->available()) {
    std::vector<char> buf(2048);
    const size_t size = geometry_receiver->receive(buf);

    if (size > 0) {
      SSL_WrapperPacket packet;
      packet.ParseFromString(std::string(buf.begin(), buf.end()));
      if (packet.has_geometry()) {
        publish_geometry(packet.geometry());
      }
    }
  }
}

void VisionTracker::publish_detection(const TrackedFrame & tracked_frame)
{
  auto tracker_msg = std::make_unique<robocup_ssl_msgs::msg::TrackedFrame>();

  tracker_msg->frame_number = tracked_frame.frame_number();
  tracker_msg->timestamp = tracked_frame.timestamp();

  for (const auto & ball : tracked_frame.balls()) {
    robocup_ssl_msgs::msg::TrackedBall tracked_ball;
    tracked_ball.pos.x = ball.pos().x();
    tracked_ball.pos.y = ball.pos().y();
    tracked_ball.pos.z = ball.pos().z();

    if (ball.has_vel()) {
      robocup_ssl_msgs::msg::Vector3 vel;
      vel.x = ball.vel().x();
      vel.y = ball.vel().y();
      vel.z = ball.vel().z();
      tracked_ball.vel.push_back(vel);
    }

    if (ball.has_visibility()) {
      tracked_ball.visibility.push_back(ball.visibility());
    }
    tracker_msg->balls.push_back(tracked_ball);
  }

  for (const auto & robot : tracked_frame.robots()) {
    robocup_ssl_msgs::msg::TrackedRobot msg_robot;
    msg_robot.robot_id.id = robot.robot_id().id();
    msg_robot.robot_id.team = robot.robot_id().team();

    msg_robot.pos.x = robot.pos().x();
    msg_robot.pos.y = robot.pos().y();

    msg_robot.orientation = robot.orientation();

    if (robot.has_vel()) {
      robocup_ssl_msgs::msg::Vector2 vel;
      vel.x = robot.vel().x();
      vel.y = robot.vel().y();
      msg_robot.vel.push_back(vel);
    }

    if (robot.has_vel_angular()) {
      msg_robot.vel_angular.push_back(robot.vel_angular());
    }

    if (robot.has_visibility()) {
      msg_robot.visibility.push_back(robot.visibility());
    }

    tracker_msg->robots.push_back(msg_robot);
  }

  if (tracked_frame.has_kicked_ball()) {
    robocup_ssl_msgs::msg::KickedBall kicked_ball;
    kicked_ball.pos.x = tracked_frame.kicked_ball().pos().x();
    kicked_ball.pos.y = tracked_frame.kicked_ball().pos().y();

    kicked_ball.vel.x = tracked_frame.kicked_ball().vel().x();
    kicked_ball.vel.y = tracked_frame.kicked_ball().vel().y();
    kicked_ball.vel.z = tracked_frame.kicked_ball().vel().z();

    kicked_ball.start_timestamp = tracked_frame.kicked_ball().start_timestamp();

    if (tracked_frame.kicked_ball().has_stop_timestamp()) {
      kicked_ball.stop_timestamp.push_back(tracked_frame.kicked_ball().stop_timestamp());
    }

    if (tracked_frame.kicked_ball().has_stop_pos()) {
      robocup_ssl_msgs::msg::Vector2 stop_pos;
      stop_pos.x = tracked_frame.kicked_ball().stop_pos().x();
      stop_pos.y = tracked_frame.kicked_ball().stop_pos().y();
      kicked_ball.stop_pos.push_back(stop_pos);
    }
    if (tracked_frame.kicked_ball().has_robot_id()) {
      robocup_ssl_msgs::msg::RobotId robot_id;
      robot_id.id = tracked_frame.kicked_ball().robot_id().id();
      robot_id.team = tracked_frame.kicked_ball().robot_id().team();
      kicked_ball.robot_id.push_back(robot_id);
    }
    tracker_msg->kicked_ball.push_back(kicked_ball);
  }

  for (const auto & capability : tracked_frame.capabilities()) {
    tracker_msg->capabilities.push_back(capability);
  }
  pub_tracked_frame->publish(std::move(tracker_msg));
}

void VisionTracker::publish_geometry(const SSL_GeometryData & geometry_data)
{
  auto geometry_msg = std::make_unique<robocup_ssl_msgs::msg::GeometryData>();
  set_geometry_field_size(geometry_msg->field, geometry_data.field());
  for (const auto & data_calib : geometry_data.calib()) {
    geometry_msg->calib.push_back(parse_calib(data_calib));
  }

  pub_geometry->publish(std::move(geometry_msg));
}

void VisionTracker::set_geometry_field_size(
  robocup_ssl_msgs::msg::GeometryFieldSize & msg_field, const SSL_GeometryFieldSize & data_field)
{
  msg_field.field_length = data_field.field_length();
  msg_field.field_width = data_field.field_width();
  msg_field.goal_width = data_field.goal_width();
  msg_field.goal_depth = data_field.goal_depth();
  msg_field.boundary_width = data_field.boundary_width();
  for (const auto & line : data_field.field_lines()) {
    robocup_ssl_msgs::msg::FieldLineSegment msg_line;
    msg_line.name = line.name();
    msg_line.p1.x = line.p1().x();
    msg_line.p1.y = line.p1().y();
    msg_line.p2.x = line.p2().x();
    msg_line.p2.y = line.p2().y();
    msg_line.thickness = line.thickness();
    if (line.has_type()) {
      msg_line.type.push_back(line.type());
    }

    msg_field.field_lines.push_back(msg_line);
  }
  for (const auto & arc : data_field.field_arcs()) {
    robocup_ssl_msgs::msg::FieldCircularArc msg_arc;
    msg_arc.name = arc.name();
    msg_arc.center.x = arc.center().x();
    msg_arc.center.y = arc.center().y();
    msg_arc.radius = arc.radius();
    msg_arc.a1 = arc.a1();
    msg_arc.a2 = arc.a2();
    msg_arc.thickness = arc.thickness();
    if (arc.has_type()) {
      msg_arc.type.push_back(arc.type());
    }

    msg_field.field_arcs.push_back(msg_arc);
  }
  if (data_field.has_penalty_area_depth()) {
    msg_field.penalty_area_depth.push_back(data_field.penalty_area_depth());
  }
  if (data_field.has_penalty_area_width()) {
    msg_field.penalty_area_width.push_back(data_field.penalty_area_width());
  }
}

robocup_ssl_msgs::msg::GeometryCameraCalibration VisionTracker::parse_calib(
  const SSL_GeometryCameraCalibration & data_calib)
{
  robocup_ssl_msgs::msg::GeometryCameraCalibration msg_calib;
  msg_calib.camera_id = data_calib.camera_id();
  msg_calib.focal_length = data_calib.focal_length();
  msg_calib.principal_point_x = data_calib.principal_point_x();
  msg_calib.principal_point_y = data_calib.principal_point_y();
  msg_calib.distortion = data_calib.distortion();
  msg_calib.q0 = data_calib.q0();
  msg_calib.q1 = data_calib.q1();
  msg_calib.q2 = data_calib.q2();
  msg_calib.q3 = data_calib.q3();
  msg_calib.tx = data_calib.tx();
  msg_calib.ty = data_calib.ty();
  msg_calib.tz = data_calib.tz();
  if (data_calib.has_derived_camera_world_tx()) {
    msg_calib.derived_camera_world_tx.push_back(data_calib.derived_camera_world_tx());
  }
  if (data_calib.has_derived_camera_world_ty()) {
    msg_calib.derived_camera_world_ty.push_back(data_calib.derived_camera_world_ty());
  }
  if (data_calib.has_derived_camera_world_tz()) {
    msg_calib.derived_camera_world_tz.push_back(data_calib.derived_camera_world_tz());
  }
  if (data_calib.has_pixel_image_width()) {
    msg_calib.pixel_image_width.push_back(data_calib.pixel_image_width());
  }
  if (data_calib.has_pixel_image_height()) {
    msg_calib.pixel_image_height.push_back(data_calib.pixel_image_height());
  }

  return msg_calib;
}

}  // namespace robocup_ssl_comm

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(robocup_ssl_comm::VisionTracker)
