// Copyright (c) 2022 ibis-ssl
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

#include <memory>
#include "crane_planner_plugins/receive_planner.hpp"
#include "crane_planner_plugins/matplotlibcpp.hpp"
#include "crane_msgs/msg/robot_info_ours.hpp"
#include "crane_msgs/msg/robot_info_theirs.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  crane::ReceivePlanner receive_planner(options);
  receive_planner.session_info.receiver_id = 2;
  auto world_model = std::make_shared<crane_msgs::msg::WorldModel>();
  // ball
  world_model->ball_info.pose.x = 1.0;
  world_model->ball_info.pose.y = 1.0;
  world_model->ball_info.velocity.x = 1.0;
  world_model->ball_info.velocity.y = 1.0;

  std::vector<double> ours_x = { 1.0, 3.5, 0.5 };
  std::vector<double> ours_y = { 1.0, 2.0, 3.0 };

  crane_msgs::msg::RobotInfoOurs robot_info_ours;
  robot_info_ours.disappeared = false;

  for (int i = 0; i < 3; i++)
  {
    robot_info_ours.id = i + 1;
    robot_info_ours.pose.x = ours_x[i];
    robot_info_ours.pose.y = ours_y[i];
    world_model->robot_info_ours.push_back(robot_info_ours);
  }

  std::vector<double> theirs_x = { 1.5, 1.8, 2.5 };
  std::vector<double> theirs_y = { 2.3, 3.0, 3.9 };
  crane_msgs::msg::RobotInfoTheirs robot_info_theirs;
  robot_info_theirs.disappeared = false;

  for (int i = 0; i < 3; i++)
  {
    robot_info_theirs.id = i + 1;
    robot_info_theirs.pose.x = theirs_x[i];
    robot_info_theirs.pose.y = theirs_y[i];
    world_model->robot_info_theirs.push_back(robot_info_theirs);
  }

  receive_planner.world_model_->update(*world_model);

  Segment ball_line;
  ball_line.first << 1.0, 1.0;
  ball_line.second << 3.5, 3.5;

  Point next_target;
  next_target << 0.5, 4.5;

  auto position_with_score = receive_planner.getPositionsWithScore(ball_line, next_target);
  std::vector<double> pos_x, pos_y, score;
  for (auto elem : position_with_score)
  {
    score.push_back(elem.first);
    pos_x.push_back(elem.second.x());
    pos_y.push_back(elem.second.y());
  }

  //    std::vector<double> pass_x = {1.0, 3.5, 0.5};
  //    std::vector<double> pass_y = {1.0, 3.5, 4.5};

  std::vector<double> pass_x = { 0.5 };
  std::vector<double> pass_y = { 4.5 };

  namespace plt = matplotlibcpp;

  plt::title("Fig. 1 : A nice figure");
  plt::xlabel("x [m]");
  plt::ylabel("y [m]");
  plt::scatter_colored(pos_x, pos_y, score, 25.0);
  plt::scatter(ours_x, ours_y, 50.0);
  plt::scatter(theirs_x, theirs_y, 50.0);

  plt::scatter(pass_x, pass_y, 50.0);
  plt::set_aspect_equal();
  plt::show();

  return 0;
}
