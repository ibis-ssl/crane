// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__CONSAI_VISUALIZER_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__CONSAI_VISUALIZER_WRAPPER_HPP_

#include <consai_visualizer_msgs/msg/objects.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace crane
{
struct ConsaiVisualizerWrapper
{
  typedef std::shared_ptr<ConsaiVisualizerWrapper> SharedPtr;

  typedef std::unique_ptr<ConsaiVisualizerWrapper> UniquePtr;

  rclcpp::Publisher<consai_visualizer_msgs::msg::Objects>::SharedPtr publisher;

  consai_visualizer_msgs::msg::Objects latest_msg;

  ConsaiVisualizerWrapper(
    rclcpp::Node & node, const std::string layyer = "default",
    const std::string sub_layer = "default", int z_order = 0)
  : publisher(
      node.create_publisher<consai_visualizer_msgs::msg::Objects>("/visualizer_objects", 10))
  {
    latest_msg.layer = layyer;
    latest_msg.sub_layer = sub_layer;
    latest_msg.z_order = z_order;
  }

  void publish(bool clear = true)
  {
    publisher->publish(latest_msg);
    if (clear) {
      latest_msg.annotations.clear();
      latest_msg.points.clear();
      latest_msg.lines.clear();
      latest_msg.arcs.clear();
      latest_msg.rects.clear();
      latest_msg.circles.clear();
      latest_msg.tubes.clear();
      latest_msg.robots.clear();
    }
  }

  void addAnnotation(
    const std::string & text, double normed_x, double normed_y, double normed_width,
    double normed_height, std::string color = "white", double alpha = 1.0)
  {
    consai_visualizer_msgs::msg::ShapeAnnotation annotation;
    annotation.text = text;
    annotation.normalized_x = normed_x;
    annotation.normalized_y = normed_y;
    annotation.normalized_width = normed_width;
    annotation.normalized_height = normed_height;
    annotation.color.name = color;
    annotation.color.alpha = alpha;
    latest_msg.annotations.push_back(annotation);
  }

  void addPoint(
    double x, double y, int size, std::string color = "white", double alpha = 1.0,
    std::string caption = "")
  {
    consai_visualizer_msgs::msg::ShapePoint point;
    point.x = x;
    point.y = y;
    point.color.name = color;
    point.color.alpha = alpha;
    point.size = size;
    point.caption = caption;
    latest_msg.points.push_back(point);
  }

  void addLine(
    double x1, double y1, double x2, double y2, int size, std::string color = "white",
    double alpha = 1.0, std::string caption = "")
  {
    consai_visualizer_msgs::msg::ShapeLine line;
    line.p1.x = x1;
    line.p1.y = y1;
    line.p2.x = x2;
    line.p2.y = y2;
    line.color.name = color;
    line.color.alpha = alpha;
    line.size = size;
    line.caption = caption;
    latest_msg.lines.push_back(line);
  }

  void addLine(
    Point p1, Point p2, int size, std::string color = "white", double alpha = 1.0,
    std::string caption = "")
  {
    addLine(p1.x(), p1.y(), p2.x(), p2.y(), size, color, alpha, caption);
  }

  void addArc(
    double x, double y, double radius, double start_angle, double end_angle, int size,
    std::string color = "white", double alpha = 1.0, std::string caption = "")
  {
    consai_visualizer_msgs::msg::ShapeArc arc;
    arc.center.x = x;
    arc.center.y = y;
    arc.radius = radius;
    arc.start_angle = start_angle;
    arc.end_angle = end_angle;
    arc.color.name = color;
    arc.color.alpha = alpha;
    arc.size = size;
    arc.caption = caption;
    latest_msg.arcs.push_back(arc);
  }

  void addArc(
    Point center, double radius, double start_angle, double end_angle, int size,
    std::string color = "white", double alpha = 1.0, std::string caption = "")
  {
    addArc(center.x(), center.y(), radius, start_angle, end_angle, size, color, alpha, caption);
  }

  void addRect(
    double x, double y, double width, double height, int line_size,
    std::string line_color = "white", std::string fill_color = "white", double alpha = 1.0,
    std::string caption = "")
  {
    consai_visualizer_msgs::msg::ShapeRectangle rect;
    rect.center.x = x;
    rect.center.y = y;
    rect.width = width;
    rect.height = height;
    rect.line_color.name = line_color;
    rect.line_color.alpha = alpha;
    rect.fill_color.name = fill_color;
    rect.fill_color.alpha = alpha;
    rect.line_size = line_size;
    rect.caption = caption;
    latest_msg.rects.push_back(rect);
  }

  void addRect(
    Point center, double width, double height, int line_size, std::string line_color = "white",
    std::string fill_color = "white", double alpha = 1.0, std::string caption = "")
  {
    addRect(
      center.x(), center.y(), width, height, line_size, line_color, fill_color, alpha, caption);
  }

  void addCircle(
    double x, double y, double radius, int line_size, std::string line_color = "white",
    std::string fill_color = "white", double alpha = 1.0, std::string caption = "")
  {
    consai_visualizer_msgs::msg::ShapeCircle circle;
    circle.center.x = x;
    circle.center.y = y;
    circle.radius = radius;
    circle.line_color.name = line_color;
    circle.line_color.alpha = alpha;
    circle.fill_color.name = fill_color;
    circle.fill_color.alpha = alpha;
    circle.line_size = line_size;
    circle.caption = caption;
    latest_msg.circles.push_back(circle);
  }

  void addCircle(
    Point center, double radius, int line_size, std::string line_color = "white",
    std::string fill_color = "white", double alpha = 1.0, std::string caption = "")
  {
    addCircle(center.x(), center.y(), radius, line_size, line_color, fill_color, alpha, caption);
  }

  void addTube(
    double x1, double y1, double x2, double y2, double radius, int line_size,
    std::string line_color = "white", std::string fill_color = "white", double alpha = 1.0,
    std::string caption = "")
  {
    consai_visualizer_msgs::msg::ShapeTube tube;
    tube.p1.x = x1;
    tube.p1.y = y1;
    tube.p2.x = x2;
    tube.p2.y = y2;
    tube.radius = radius;
    tube.line_color.name = line_color;
    tube.line_color.alpha = alpha;
    tube.fill_color.name = fill_color;
    tube.fill_color.alpha = alpha;
    tube.line_size = line_size;
    tube.caption = caption;
    latest_msg.tubes.push_back(tube);
  }

  void addTube(
    Point p1, Point p2, double radius, int line_size, std::string line_color = "white",
    std::string fill_color = "white", double alpha = 1.0, std::string caption = "")
  {
    addTube(
      p1.x(), p1.y(), p2.x(), p2.y(), radius, line_size, line_color, fill_color, alpha, caption);
  }

  void addRobot(
    double id, double x, double y, double theta, std::string line_color = "white",
    std::string fill_color = "white", double alpha = 1.0, double line_size = 1,
    std::string caption = "", bool color_type = 0)
  {
    consai_visualizer_msgs::msg::Robot robot;
    robot.x = x;
    robot.y = y;
    robot.theta = theta;
    robot.radius = 0.09;
    robot.line_color.name = line_color;
    robot.line_color.alpha = alpha;
    robot.fill_color.name = fill_color;
    robot.fill_color.alpha = alpha;
    robot.line_size = line_size;
    robot.caption = caption;
    robot.id = id;
    robot.color_type = color_type;
    latest_msg.robots.push_back(robot);
  }

  void addRobot(
    double id, Point p, double theta, std::string line_color = "white",
    std::string fill_color = "white", double alpha = 1.0, double line_size = 1,
    std::string caption = "", bool color_type = 0)
  {
    addRobot(
      id, p.x(), p.y(), theta, line_color, fill_color, alpha, line_size, caption, color_type);
  }
};
}  // namespace crane
#endif  // CRANE_MSG_WRAPPERS__CONSAI_VISUALIZER_WRAPPER_HPP_
