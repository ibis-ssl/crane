// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// Tracker が受け取った TrackerWrapperPacket を、どの TrackedFrame として publish するかを固定する。
// UDP で送って topic で受けるので、変換の実装を差し替えても同じテストで比べられる。

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_comm/tracker_component.hpp>
#include <robocup_ssl_msgs/msg/tracked_frame.hpp>
#include <string>
#include <vector>

namespace
{
constexpr char kLoopback[] = "127.0.0.1";

// 他のテストとポートが重ならないよう、OS に空きポートを選ばせる
int pick_free_udp_port()
{
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = 0;
  inet_pton(AF_INET, kLoopback, &addr.sin_addr);
  bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
  socklen_t len = sizeof(addr);
  getsockname(fd, reinterpret_cast<sockaddr *>(&addr), &len);
  close(fd);
  return ntohs(addr.sin_port);
}

void send_udp(int port, const std::string & payload)
{
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  inet_pton(AF_INET, kLoopback, &addr.sin_addr);
  sendto(fd, payload.data(), payload.size(), 0, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
  close(fd);
}

void set_vector2(robocup_ssl::Vector2 * v, float x, float y)
{
  v->set_x(x);
  v->set_y(y);
}

void set_vector3(robocup_ssl::Vector3 * v, float x, float y, float z)
{
  v->set_x(x);
  v->set_y(y);
  v->set_z(z);
}

// optional フィールドの有無と、team の無い robot_id を混ぜた代表パケット
robocup_ssl::TrackerWrapperPacket make_packet()
{
  robocup_ssl::TrackerWrapperPacket packet;
  packet.set_uuid("test-uuid");
  auto * frame = packet.mutable_tracked_frame();
  frame->set_frame_number(42);
  frame->set_timestamp(1234.5);

  auto * ball_full = frame->add_balls();
  set_vector3(ball_full->mutable_pos(), 1.5f, -2.25f, 0.125f);
  set_vector3(ball_full->mutable_vel(), 0.5f, 0.25f, 0.0f);
  ball_full->set_visibility(0.75f);

  auto * ball_pos_only = frame->add_balls();
  set_vector3(ball_pos_only->mutable_pos(), -3.0f, 4.0f, 0.0f);

  auto * robot_full = frame->add_robots();
  robot_full->mutable_robot_id()->set_id(3);
  robot_full->mutable_robot_id()->set_team(robocup_ssl::YELLOW);
  set_vector2(robot_full->mutable_pos(), -1.0f, 2.0f);
  robot_full->set_orientation(0.5f);
  set_vector2(robot_full->mutable_vel(), 0.125f, -0.5f);
  robot_full->set_vel_angular(1.25f);
  robot_full->set_visibility(1.0f);

  auto * robot_no_team = frame->add_robots();
  robot_no_team->mutable_robot_id()->set_id(7);
  set_vector2(robot_no_team->mutable_pos(), 2.5f, -0.5f);
  robot_no_team->set_orientation(-1.5f);

  auto * kicked = frame->mutable_kicked_ball();
  set_vector2(kicked->mutable_pos(), 0.5f, 0.5f);
  set_vector3(kicked->mutable_vel(), 1.0f, 2.0f, 0.0f);
  kicked->set_start_timestamp(1230.25);
  kicked->set_stop_timestamp(1235.0);
  kicked->mutable_robot_id()->set_id(3);
  kicked->mutable_robot_id()->set_team(robocup_ssl::BLUE);

  frame->add_capabilities(robocup_ssl::CAPABILITY_DETECT_MULTIPLE_BALLS);
  frame->add_capabilities(robocup_ssl::CAPABILITY_DETECT_KICKED_BALLS);
  return packet;
}

robocup_ssl_msgs::msg::Vector2 vector2(float x, float y, uint8_t has_field)
{
  robocup_ssl_msgs::msg::Vector2 v;
  v.x = x;
  v.y = y;
  v.has_field = has_field;
  return v;
}

robocup_ssl_msgs::msg::Vector3 vector3(float x, float y, float z, uint8_t has_field)
{
  robocup_ssl_msgs::msg::Vector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  v.has_field = has_field;
  return v;
}

robocup_ssl_msgs::msg::RobotId robot_id(uint32_t id, int32_t team, uint8_t has_field)
{
  robocup_ssl_msgs::msg::RobotId r;
  r.id = id;
  r.team.value = team;
  r.has_field = has_field;
  return r;
}

robocup_ssl_msgs::msg::TrackedFrame expected_frame()
{
  using robocup_ssl_msgs::msg::KickedBall;
  using robocup_ssl_msgs::msg::Team;
  using robocup_ssl_msgs::msg::TrackedBall;
  using robocup_ssl_msgs::msg::TrackedFrame;
  using robocup_ssl_msgs::msg::TrackedRobot;
  // msg の has_field は既定値が 255（全ビット立ち）で、変換は |= でビットを足すだけなので、
  // フィールドの有無に関わらずすべての has_field が 255 のまま publish される
  constexpr uint8_t kAllBits = 255;

  TrackedFrame frame;
  frame.frame_number = 42;
  frame.timestamp = 1234.5;

  TrackedBall ball_full;
  ball_full.pos = vector3(1.5f, -2.25f, 0.125f, kAllBits);
  ball_full.vel = vector3(0.5f, 0.25f, 0.0f, kAllBits);
  ball_full.visibility = 0.75f;
  ball_full.has_field = kAllBits;
  frame.balls.push_back(ball_full);

  TrackedBall ball_pos_only;
  ball_pos_only.pos = vector3(-3.0f, 4.0f, 0.0f, kAllBits);
  ball_pos_only.has_field = kAllBits;
  frame.balls.push_back(ball_pos_only);

  TrackedRobot robot_full;
  robot_full.robot_id = robot_id(3, Team::YELLOW, kAllBits);
  robot_full.pos = vector2(-1.0f, 2.0f, kAllBits);
  robot_full.orientation = 0.5f;
  robot_full.vel = vector2(0.125f, -0.5f, kAllBits);
  robot_full.vel_angular = 1.25f;
  robot_full.visibility = 1.0f;
  robot_full.has_field = kAllBits;
  frame.robots.push_back(robot_full);

  TrackedRobot robot_no_team;
  robot_no_team.robot_id = robot_id(7, Team::UNKNOWN, kAllBits);
  robot_no_team.pos = vector2(2.5f, -0.5f, kAllBits);
  robot_no_team.orientation = -1.5f;
  robot_no_team.has_field = kAllBits;
  frame.robots.push_back(robot_no_team);

  KickedBall kicked;
  kicked.pos = vector2(0.5f, 0.5f, kAllBits);
  kicked.vel = vector3(1.0f, 2.0f, 0.0f, kAllBits);
  kicked.start_timestamp = 1230.25;
  kicked.stop_timestamp = 1235.0;
  kicked.robot_id = robot_id(3, Team::BLUE, kAllBits);
  kicked.has_field = kAllBits;
  frame.kicked_ball = kicked;

  robocup_ssl_msgs::msg::Capability multiple_balls;
  multiple_balls.value = robocup_ssl_msgs::msg::Capability::CAPABILITY_DETECT_MULTIPLE_BALLS;
  robocup_ssl_msgs::msg::Capability kicked_balls;
  kicked_balls.value = robocup_ssl_msgs::msg::Capability::CAPABILITY_DETECT_KICKED_BALLS;
  frame.capabilities = {multiple_balls, kicked_balls};

  frame.has_field = kAllBits;
  return frame;
}
}  // namespace

TEST(TrackerComponent, PublishesReceivedPacketAsTrackedFrame)
{
  const int port = pick_free_udp_port();
  const std::string ns = "/test_tracker_" + std::to_string(getpid());

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__ns:=" + ns});
  options.parameter_overrides(
    {rclcpp::Parameter("multicast_address", kLoopback), rclcpp::Parameter("multicast_port", port)});
  auto tracker = std::make_shared<robocup_ssl_comm::Tracker>(options);

  auto listener = std::make_shared<rclcpp::Node>("tracker_test_listener");
  std::optional<robocup_ssl_msgs::msg::TrackedFrame> received;
  auto sub = listener->create_subscription<robocup_ssl_msgs::msg::TrackedFrame>(
    ns + "/tracked_frame", 10, [&](const robocup_ssl_msgs::msg::TrackedFrame & msg) {
      if (msg.frame_number == 42) {
        received = msg;
      }
    });

  const auto packet = make_packet();
  ASSERT_TRUE(packet.IsInitialized());
  std::string payload;
  ASSERT_TRUE(packet.SerializeToString(&payload));

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(tracker);
  exec.add_node(listener);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    send_udp(port, payload);
    exec.spin_once(std::chrono::milliseconds(50));
  }

  ASSERT_TRUE(received) << "tracked_frame を 5 秒以内に受け取れなかった";
  EXPECT_EQ(
    robocup_ssl_msgs::msg::to_yaml(*received), robocup_ssl_msgs::msg::to_yaml(expected_frame()));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
