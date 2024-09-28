// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/vo_planner.hpp"

#include <matplotlibcpp17/common.h>
#include <matplotlibcpp17/patches.h>
#include <matplotlibcpp17/pyplot.h>
#include <pybind11/pybind11.h>

namespace matplotlibcpp17::patches
{
/**
 * @brief A wrapper class for matplotlib.patches.Arrow
 **/
struct DECL_STRUCT_ATTR Arrow : public BaseWrapper
{
public:
  Arrow(
    const pybind11::tuple & args = pybind11::tuple(),
    const pybind11::dict & kwargs = pybind11::dict())
  {
    arrow_attr = pybind11::module::import("matplotlib.patches").attr("Arrow");
    self = arrow_attr(*args, **kwargs);
  }

private:
  pybind11::object arrow_attr;
};
}  // namespace matplotlibcpp17::patches

int main()
{
  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();
  using matplotlibcpp17::pyplot::PyPlot;

  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node("node");
  auto world_model = std::make_shared<crane::WorldModelWrapper>(node);

  crane_msgs::msg::WorldModel wm_msg;
  wm_msg.is_yellow = true;
  wm_msg.field_info.x = 9.0;
  wm_msg.field_info.y = 6.0;
  wm_msg.goal_size.x = 0.18;
  wm_msg.goal_size.y = 1.0;
  wm_msg.ball_info.pose.x = 0.;
  wm_msg.ball_info.pose.y = 0.;
  wm_msg.ball_info.pose.theta = 0.;

  auto get_robot_patch = [](auto robot_info, std::string color = "g") {
    return matplotlibcpp17::patches::Circle(
      Args(py::make_tuple(robot_info.pose.x, robot_info.pose.y), 0.1),
      Kwargs("fc"_a = color, "ec"_a = "black"));
  };

  auto ax = plt.axes();

  crane_msgs::msg::RobotInfoOurs our_robot;
  our_robot.disappeared = false;

  our_robot.id = 0;
  our_robot.pose.x = 1.0;
  our_robot.pose.y = 0.0;
  wm_msg.robot_info_ours.push_back(our_robot);
  ax.add_patch(Args(get_robot_patch(our_robot, "b").unwrap()));

  our_robot.id = 1;
  our_robot.pose.x = 1.5;
  our_robot.pose.y = 0.5;
  wm_msg.robot_info_ours.push_back(our_robot);
  ax.add_patch(Args(get_robot_patch(our_robot, "b").unwrap()));

  our_robot.id = 2;
  our_robot.pose.x = 1.5;
  our_robot.pose.y = -0.5;
  wm_msg.robot_info_ours.push_back(our_robot);
  ax.add_patch(Args(get_robot_patch(our_robot, "b").unwrap()));

  our_robot.id = 4;
  our_robot.pose.x = 2.0;
  our_robot.pose.y = 0.0;
  wm_msg.robot_info_ours.push_back(our_robot);
  ax.add_patch(Args(get_robot_patch(our_robot, "b").unwrap()));

  world_model->update(wm_msg);

  auto vel = crane::optimizeVelocity(
    world_model->getOurRobot(4), Point(-1.0, 0), world_model->ours.getAvailableRobots(4), 0.2);

  auto vel_line = matplotlibcpp17::patches::Arrow(
    Args(py::make_tuple(1.0, 0.0), py::make_tuple(vel.x(), vel.y())));
  ax.add_patch(vel_line.unwrap());

  plt.axis(Args("scaled"));
  ax.set_aspect(Args("equal"));
  /// user code
  // plt.plot(Args(std::vector<int>({1, 3, 2, 4})),
  //          Kwargs("color"_a = "blue", "linewidth"_a = 1.0));
  plt.show();

  return 0;
}
