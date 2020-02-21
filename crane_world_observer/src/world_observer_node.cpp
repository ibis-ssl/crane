// Copyright (c) 2020 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <rclcpp/rclcpp.hpp>

#include <boost/range/adaptor/indexed.hpp>
#include <consai2r2_msgs/msg/ball_info.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <consai2r2_msgs/msg/robot_info.hpp>
#include <consai2r2_msgs/msg/vision_detections.hpp>
#include <crane_msgs/msg/world_model.hpp>

#include <functional>
#include <cmath>
#include <vector>
#include <memory>

enum class Color : uint8_t
{
  BLUE,
  YELLOW,
};

class WorldModelComponent : public rclcpp::Node
{
public:
  WorldModelComponent()
  : Node("world_model_node")
  {
    sub_vision_ = this->create_subscription<consai2r2_msgs::msg::VisionDetections>(
      "consai2r2_vision_receiver/raw_detections",
      std::bind(&WorldModelComponent::visionDetectionsCallback, this, std::placeholders::_1));
    pub_ball_ = this->create_publisher<consai2r2_msgs::msg::BallInfo>("~/ball_info", 1);
    pub_robot_[static_cast<uint8_t>(Color::BLUE)] =
      this->create_publisher<consai2r2_msgs::msg::RobotInfo>("~/robot_info_blue", 1);
    pub_robot_[static_cast<uint8_t>(Color::YELLOW)] =
      this->create_publisher<consai2r2_msgs::msg::RobotInfo>("~/robot_info_yellow", 1);

    pub_world_model_ = create_publisher<crane_msgs::msg::WorldModel>("~/world_model",1);

    robot_info_[static_cast<int>(Color::BLUE)].resize(max_id);
    robot_info_[static_cast<int>(Color::YELLOW)].resize(max_id);

    enable_publish_ball = true;
    enable_publish_robot[static_cast<uint8_t>(Color::BLUE)] = true;
    enable_publish_robot[static_cast<uint8_t>(Color::YELLOW)] = true;

    //TODO(HansRobo) : input our_color & their_color from param
    our_color = Color::BLUE;
    their_color = Color::YELLOW;
  }

  void visionDetectionsCallback(const consai2r2_msgs::msg::VisionDetections::SharedPtr msg)
  {
    std::vector<consai2r2_msgs::msg::DetectionBall> detection_balls;
    std::vector<consai2r2_msgs::msg::DetectionRobot> detection_blue;
    std::vector<consai2r2_msgs::msg::DetectionRobot> detection_yellow;
    for (auto frame : msg->frames) {
      for (auto ball : frame.balls) {
        detection_balls.emplace_back(ball);
      }

      for (auto blue : frame.robots_blue) {
        detection_blue.emplace_back(blue);
      }

      for (auto yellow : frame.robots_yellow) {
        detection_yellow.emplace_back(yellow);
      }

      auto time_stamp = msg->header.stamp;
    }

    extractBallPose(detection_balls,msg->header.stamp);
    extractRobotPose(Color::BLUE,detection_blue,msg->header.stamp);
    extractRobotPose(Color::YELLOW,detection_yellow,msg->header.stamp);
  }

private:
  void extractBallPose(
    std::vector<consai2r2_msgs::msg::DetectionBall> & balls,
    builtin_interfaces::msg::Time time_stamp)
  {
    if (!balls.empty()) {
      auto average_pose = getAveragePose(balls);
      auto velocity = getVelocity(ball_info_.pose, average_pose, ball_info_.detection_stamp,
          time_stamp);

      ball_info_.pose = average_pose;
      ball_info_.last_detection_pose = average_pose;
      ball_info_.velocity = velocity;
      ball_info_.detected = true;
      ball_info_.detection_stamp = time_stamp;
      ball_info_.disappeared = false;

    } else {
      ball_info_.detected = false;

      if (!ball_info_.disappeared) {
        // 座標を受け取らなかった場合は、速度を用いて線形予測する
        auto diff_time = rclcpp::Clock(RCL_ROS_TIME).now() -
          rclcpp::Time(ball_info_.detection_stamp);

        ball_info_.pose = getLinearPredictionPose(ball_info_.last_detection_pose,
            ball_info_.velocity, diff_time.seconds());
        // 一定時間、座標を受け取らなかったら消滅判定にする
        if (diff_time.seconds() > DISAPPEARED_TIME_THRESH_) {
          ball_info_.disappeared = true;
        }
      }
    }
    if (enable_publish_ball) {
      pub_ball_->publish(ball_info_);
    }
  }

  void extractRobotPose(
    Color color, std::vector<consai2r2_msgs::msg::DetectionRobot> & robots,
    builtin_interfaces::msg::Time time_stamp)
  {
    std::vector<std::vector<consai2r2_msgs::msg::DetectionRobot>> detections;
    detections.resize(max_id);

    for (auto robot : robots) {
      uint8_t id = robot.robot_id;
      detections.at(id).emplace_back(robot);
    }

    for (auto && detection : detections | boost::adaptors::indexed()) {
      uint8_t id = detection.index();
      auto & robot_info = robot_info_[static_cast<uint8_t>(color)].at(id);
      if (!detection.value().empty()) {
        auto pose = getAveragePose(detection.value());
        auto vel = getVelocity(robot_info.pose,
            pose, robot_info.detection_stamp, time_stamp);
        robot_info.robot_id = id;
        robot_info.pose = pose;
        robot_info.last_detection_pose = pose;
        robot_info.velocity = vel;
        robot_info.detected = true;
        robot_info.detection_stamp = time_stamp;
        robot_info.disappeared = false;
      } else {
        robot_info.detected = false;
        robot_info.robot_id = id;

        if (!robot_info.disappeared) {
          auto diff_time = rclcpp::Clock(RCL_ROS_TIME).now() - rclcpp::Time(
            robot_info.detection_stamp);

          robot_info.pose = getLinearPredictionPose(robot_info.last_detection_pose,
              robot_info.velocity, diff_time.seconds());
          if (diff_time.seconds() > DISAPPEARED_TIME_THRESH_) {
            robot_info.disappeared = true;
          }
        }
      }
      if (enable_publish_robot[static_cast<uint8_t>(color)]) {
        pub_robot_[static_cast<uint8_t>(color)]->publish(robot_info);
      }
    }
  }

  void publishWorldModel(){
    crane_msgs::msg::WorldModel wm;
    wm.ball_info = ball_info_;
    for(auto robot : robot_info_[static_cast<uint8_t>(our_color)]){
      wm.robot_info_ours.emplace_back(robot);
    }
    for(auto robot : robot_info_[static_cast<uint8_t>(our_color)]){
      wm.robot_info_ours.emplace_back(robot);
    }
  }
  template<typename DetectionT>
  geometry_msgs::msg::Pose2D getAveragePose(std::vector<DetectionT> detections)
  {
    geometry_msgs::msg::Pose2D sum_pose;
    //  角度計算用
    float sum_x, sum_y;
    sum_x = sum_y = 0.f;

    for (auto detection : detections) {
      sum_pose.x += detection.pose.x;
      sum_pose.y += detection.pose.y;
      sum_x += std::cos(detection.pose.theta);
      sum_y += std::sin(detection.pose.theta);
    }

    sum_pose.x = sum_pose.x / detections.size();
    sum_pose.y = sum_pose.y / detections.size();

    sum_pose.theta = fmodf(std::atan2(sum_y, sum_x), M_PI);

    return sum_pose;
  }

  geometry_msgs::msg::Pose2D getVelocity(
    geometry_msgs::msg::Pose2D prev_pose,
    geometry_msgs::msg::Pose2D current_pose,
    builtin_interfaces::msg::Time prev_stamp,
    builtin_interfaces::msg::Time current_stamp)
  {
    geometry_msgs::msg::Pose2D velocity;


    rclcpp::Duration diff_time = rclcpp::Time(current_stamp) - rclcpp::Time(prev_stamp);

    if (diff_time.seconds() > 0) {
      geometry_msgs::msg::Pose2D diff_pose;
      diff_pose.x = current_pose.x - prev_pose.x;
      diff_pose.y = current_pose.y - prev_pose.y;
      diff_pose.theta = getNormalizedAngle(current_pose.theta - prev_pose.theta);

      velocity.x = diff_pose.x / diff_time.seconds();
      velocity.y = diff_pose.y / diff_time.seconds();
      velocity.theta = diff_pose.theta / diff_time.seconds();
    }
    return velocity;
  }

  float getNormalizedAngle(float angle_rad)
  {
    while (angle_rad > M_PI) {
      angle_rad -= 2 * M_PI;
    }
    while (angle_rad < -M_PI) {
      angle_rad += 2 * M_PI;
    }
    return angle_rad;
  }

  geometry_msgs::msg::Pose2D getLinearPredictionPose(
    geometry_msgs::msg::Pose2D pose,
    geometry_msgs::msg::Pose2D velocity,
    float diff_time_sec)
  {
    geometry_msgs::msg::Pose2D prediction_pose;
    prediction_pose.x = pose.x + velocity.x * diff_time_sec;
    prediction_pose.y = pose.y + velocity.y * diff_time_sec;
    prediction_pose.theta = pose.theta + velocity.theta * diff_time_sec;

    return prediction_pose;
  }
  bool enable_publish_ball;
  bool enable_publish_robot[2];
  Color our_color;
  Color their_color;
  uint8_t max_id;
  static constexpr float DISAPPEARED_TIME_THRESH_ = 3.0f;


  consai2r2_msgs::msg::BallInfo ball_info_;
  std::vector<consai2r2_msgs::msg::RobotInfo> robot_info_[2];
  rclcpp::Subscription<consai2r2_msgs::msg::VisionDetections>::SharedPtr sub_vision_;
  rclcpp::Publisher<consai2r2_msgs::msg::BallInfo>::SharedPtr pub_ball_;
  rclcpp::Publisher<consai2r2_msgs::msg::RobotInfo>::SharedPtr pub_robot_[2];
  rclcpp::Publisher<crane_msgs::msg::WorldModel>::SharedPtr pub_world_model_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldModelComponent>());
  rclcpp::shutdown();
  return 0;
}
