// Copyright (c) 2019 ibis-ssl
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

#include <crane_msgs/msg/world_model.hpp>
#include <eigen3/Eigen/Core>
#include <vector>

struct Pose2D {
  Eigen::Vector2f pos;
  float theta;
};

struct Velocity2D {
  Eigen::Vector2f linear;
  float omega;
};

struct Rect {
  Eigen::Vector2f min;
  Eigen::Vector2f max;
};

struct RobotInfo {
  uint8_t id;
  Pose2D pose;
  Velocity2D vel;
};

struct TeamInfo {
  Rect defense_area;
  std::vector<RobotInfo> robots;
};

struct Ball {
  Eigen::Vector2f pos;
  Eigen::Vector2f vel;
  bool is_curve;
};

class WorldModel {
public:
  WorldModel() {
    //メモリ確保
    //ヒトサッカーの台数は超えないはず
    constexpr uint8_t MAX_ROBOT_NUM = 11;
    ours.robots.reserve(MAX_ROBOT_NUM);
    theirs.robots.reserve(MAX_ROBOT_NUM);
  }
  void update(crane_msgs::msg::WorldModel::SharedPtr world_model) {
    ours.robots.clear();
    theirs.robots.clear();
    for (auto robot : world_model->robot_info_ours) {
      if (!robot.disappeared) {
        RobotInfo info;
//        info.id = robot.id;
        info.pose.pos << robot.pose.position.x , robot.pose.position.y;
        //todo : theta
        info.vel.linear << robot.velocity.linear.x , robot.velocity.linear.y;
        //todo : omega
        ours.robots.emplace_back(info);
      }
    }

    for (auto robot : world_model->robot_info_theirs) {
      if (!robot.disappeared) {
        RobotInfo info;
//        info.id = robot.id;
        info.pose.pos << robot.pose.position.x , robot.pose.position.y;
        //todo : theta
        info.vel.linear << robot.velocity.linear.x , robot.velocity.linear.y;
        //todo : omega
        theirs.robots.emplace_back(info);
      }
    }

    ours.defense_area.max << world_model->our_defense.max.x,world_model->our_defense.max.y;
    ours.defense_area.min << world_model->our_defense.min.x,world_model->our_defense.min.y;

    theirs.defense_area.max << world_model->our_defense.max.x,world_model->our_defense.max.y;
    theirs.defense_area.min << world_model->their_defense.min.x,world_model->their_defense.min.y;

    ball.pos << world_model->ball_info.pose.position.x , world_model->ball_info.pose.position.y;
    ball.vel << world_model->ball_info.velocity.linear.x , world_model->ball_info.velocity.linear.y;
    ball.is_curve = world_model->ball_info.curved;

    field_size << world_model->field_info.x , world_model->field_info.y;
  }

public:
  TeamInfo ours;
  TeamInfo theirs;
  Eigen::Vector2f field_size;
  Ball ball;
};