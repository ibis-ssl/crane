// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <matplotlibcpp17/pyplot.h>

#include <crane_msgs/msg/robot_info_ours.hpp>
#include <crane_msgs/msg/robot_info_theirs.hpp>
#include <memory>

#include "crane_planner_plugins/receive_planner.hpp"

namespace crane
{
std::shared_ptr<std::unordered_map<uint8_t, RobotRole>> PlannerBase::robot_roles = nullptr;
}  // namespace crane

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto world_model = std::make_shared<crane_msgs::msg::WorldModel>();

  auto node = rclcpp::Node::make_shared("receive_planner");
  auto world_model_wrapper = std::make_shared<crane::WorldModelWrapper>(*node);
  auto visualizer = std::make_shared<crane::ConsaiVisualizerWrapper>(*node);

  crane::ReceivePlanner receive_planner(world_model_wrapper, visualizer);
  receive_planner.session_info.receiver_id = 2;
  // ball
  world_model->ball_info.pose.x = 1.0;
  world_model->ball_info.pose.y = 1.0;
  world_model->ball_info.velocity.x = 1.0;
  world_model->ball_info.velocity.y = 1.0;

  std::vector<double> ours_x = {1.0, 3.5, 0.5};
  std::vector<double> ours_y = {1.0, 2.0, 3.0};

  crane_msgs::msg::RobotInfoOurs robot_info_ours;
  robot_info_ours.disappeared = false;

  for (int i = 0; i < 3; i++) {
    robot_info_ours.id = i + 1;
    robot_info_ours.pose.x = ours_x[i];
    robot_info_ours.pose.y = ours_y[i];
    world_model->robot_info_ours.push_back(robot_info_ours);
  }

  std::vector<double> theirs_x = {1.5, 1.8, 2.5};
  std::vector<double> theirs_y = {2.3, 3.0, 3.9};
  crane_msgs::msg::RobotInfoTheirs robot_info_theirs;
  robot_info_theirs.disappeared = false;

  for (int i = 0; i < 3; i++) {
    robot_info_theirs.id = i + 1;
    robot_info_theirs.pose.x = theirs_x[i];
    robot_info_theirs.pose.y = theirs_y[i];
    world_model->robot_info_theirs.push_back(robot_info_theirs);
  }

  world_model->goal_size.y = 0.3;
  world_model->field_info.x = 6.0;
  world_model->field_info.y = 4.0;

  world_model_wrapper->update(*world_model);

  Segment ball_line;
  ball_line.first << 1.0, 1.0;
  ball_line.second << 3.5, 3.5;

  //  Point base;
  //  base << 0.5, 4.5;

  auto position_with_score =
    receive_planner.getPositionsWithScore(0.25, 16, world_model_wrapper->ball.pos);
  std::vector<double> pos_x, pos_y, score;
  for (auto elem : position_with_score) {
    std::cout << elem.first << std::endl;
    score.push_back(elem.first);
    pos_x.push_back(elem.second.x());
    pos_y.push_back(elem.second.y());
  }

  //    std::vector<double> pass_x = {1.0, 3.5, 0.5};
  //    std::vector<double> pass_y = {1.0, 3.5, 4.5};

  std::vector<double> pass_x = {0.5};
  std::vector<double> pass_y = {4.5};

  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();

  plt.title(Args("Fig. 1 : A nice figure"));
  plt.xlabel(Args("x [m]"));
  plt.ylabel(Args("y [m]"));
  plt.scatter(Args(pos_x, pos_y, 20.0), Kwargs("c"_a = score));
  plt.scatter(Args(ours_x, ours_y, 50.0));
  plt.scatter(Args(theirs_x, theirs_y, 50.0));

  plt.scatter(Args(pass_x, pass_y, 50.0));
  // cspell: ignore figaspect
  plt.figaspect(Args(1.0));
  //  plt.set_aspect_equal();
  plt.show();

  return 0;
}
