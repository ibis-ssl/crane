// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__CONSAI_VISUALIZER_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__CONSAI_VISUALIZER_WRAPPER_HPP_

#include <consai_visualizer_msgs/msg/objects.hpp>
#include <crane_geometry/boost_geometry.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace crane
{

template <class MsgT>
struct ColorBuilder
{
  MsgT & msg;
  explicit ColorBuilder(MsgT & msg) : msg(msg)
  {
    msg.color.name = "white";
    msg.color.alpha = 1.0;
  }

  ColorBuilder<MsgT> & color(const std::string & name)
  {
    msg.color.name = name;
    return *this;
  }

  ColorBuilder<MsgT> & alpha(double a)
  {
    msg.color.alpha = a;
    return *this;
  }

  ColorBuilder<MsgT> & color(double r, double g, double b, double a = 1.0)
  {
    msg.color.red = r;
    msg.color.green = g;
    msg.color.blue = b;
    msg.color.alpha = a;
    return *this;
  }
};

template <class MsgT>
struct FillShapeColorBuilder
{
  MsgT & msg;
  explicit FillShapeColorBuilder(MsgT & msg) : msg(msg)
  {
    msg.fill_color.name = "white";
    msg.fill_color.alpha = 0.0;
    msg.line_color.name = "white";
    msg.line_color.alpha = 1.0;
  }

  FillShapeColorBuilder<MsgT> & color(const std::string & name)
  {
    msg.fill_color.name = name;
    msg.fill_color.alpha = 1.0;
    msg.line_color.name = name;
    msg.line_color.alpha = 1.0;
    return *this;
  }

  FillShapeColorBuilder<MsgT> & color(double r, double g, double b, double a = 1.0)
  {
    msg.fill_color.red = r;
    msg.fill_color.green = g;
    msg.fill_color.blue = b;
    msg.fill_color.alpha = a;
    msg.line_color.red = r;
    msg.line_color.green = g;
    msg.line_color.blue = b;
    msg.line_color.alpha = a;
    return *this;
  }

  FillShapeColorBuilder<MsgT> & alpha(double a)
  {
    msg.fill_color.alpha = a;
    msg.line_color.alpha = a;
    return *this;
  }

  FillShapeColorBuilder<MsgT> & fill_alpha(double a)
  {
    msg.fill_color.alpha = a;
    return *this;
  }

  FillShapeColorBuilder<MsgT> & line_alpha(double a)
  {
    msg.line_color.alpha = a;
    return *this;
  }

  FillShapeColorBuilder<MsgT> & fill_color(double r, double g, double b, double a = 1.0)
  {
    msg.fill_color.red = r;
    msg.fill_color.green = g;
    msg.fill_color.blue = b;
    msg.fill_color.alpha = a;
    return *this;
  }

  FillShapeColorBuilder<MsgT> & line_color(double r, double g, double b, double a = 1.0)
  {
    msg.line_color.red = r;
    msg.line_color.green = g;
    msg.line_color.blue = b;
    msg.line_color.alpha = a;
    return *this;
  }

  FillShapeColorBuilder<MsgT> & fill_color(const std::string & name)
  {
    msg.fill_color.name = name;
    return *this;
  }

  FillShapeColorBuilder<MsgT> & line_color(const std::string & name)
  {
    msg.line_color.name = name;
    return *this;
  }
};

struct ShapeAnnotationBuilder : public ColorBuilder<consai_visualizer_msgs::msg::ShapeAnnotation>
{
  consai_visualizer_msgs::msg::ShapeAnnotation annotation;
  ShapeAnnotationBuilder() : ColorBuilder<consai_visualizer_msgs::msg::ShapeAnnotation>(annotation)
  {
    annotation.color.name = "white";
    annotation.color.alpha = 1.0;
  }
  operator consai_visualizer_msgs::msg::ShapeAnnotation() const { return annotation; }

  ShapeAnnotationBuilder & text(const std::string & text)
  {
    annotation.text = text;
    return *this;
  }

  ShapeAnnotationBuilder & normalized_x(double normalized_x)
  {
    annotation.normalized_x = normalized_x;
    return *this;
  }

  ShapeAnnotationBuilder & normalized_y(double normalized_y)
  {
    annotation.normalized_y = normalized_y;
    return *this;
  }

  ShapeAnnotationBuilder & normalized_position(Point p)
  {
    annotation.normalized_x = p.x();
    annotation.normalized_y = p.y();
    return *this;
  }

  ShapeAnnotationBuilder & normalized_width(double normalized_width)
  {
    annotation.normalized_width = normalized_width;
    return *this;
  }

  ShapeAnnotationBuilder & normalized_height(double normalized_height)
  {
    annotation.normalized_height = normalized_height;
    return *this;
  }
};

struct ShapePointBuilder : public ColorBuilder<consai_visualizer_msgs::msg::ShapePoint>
{
  consai_visualizer_msgs::msg::ShapePoint point;
  ShapePointBuilder() : ColorBuilder<consai_visualizer_msgs::msg::ShapePoint>(point)
  {
    point.color.name = "white";
    point.color.alpha = 1.0;
    point.size = 1;
  }
  operator consai_visualizer_msgs::msg::ShapePoint() const { return point; }

  ShapePointBuilder & x(double x)
  {
    point.x = x;
    return *this;
  }

  ShapePointBuilder & y(double y)
  {
    point.y = y;
    return *this;
  }

  ShapePointBuilder & position(Point p)
  {
    point.x = p.x();
    point.y = p.y();
    return *this;
  }

  ShapePointBuilder & size(int size)
  {
    point.size = size;
    return *this;
  }

  ShapePointBuilder & caption(const std::string & caption)
  {
    point.caption = caption;
    return *this;
  }
};

struct ShapeArcBuilder : public ColorBuilder<consai_visualizer_msgs::msg::ShapeArc>
{
  consai_visualizer_msgs::msg::ShapeArc arc;
  ShapeArcBuilder() : ColorBuilder<consai_visualizer_msgs::msg::ShapeArc>(arc) {}
  operator consai_visualizer_msgs::msg::ShapeArc() const { return arc; }

  ShapeArcBuilder & center(double x, double y)
  {
    arc.center.x = x;
    arc.center.y = y;
    return *this;
  }

  ShapeArcBuilder & center(Point p)
  {
    arc.center.x = p.x();
    arc.center.y = p.y();
    return *this;
  }

  ShapeArcBuilder & radius(double radius)
  {
    arc.radius = radius;
    return *this;
  }

  ShapeArcBuilder & start_angle(double start_angle)
  {
    arc.start_angle = start_angle;
    return *this;
  }

  ShapeArcBuilder & end_angle(double end_angle)
  {
    arc.end_angle = end_angle;
    return *this;
  }

  ShapeArcBuilder & angle_range(double start, double end)
  {
    arc.start_angle = start;
    arc.end_angle = end;
    return *this;
  }
  ShapeArcBuilder & size(int size)
  {
    arc.size = size;
    return *this;
  }

  ShapeArcBuilder & caption(const std::string & caption)
  {
    arc.caption = caption;
    return *this;
  }
};

struct ShapeCircleBuilder : public FillShapeColorBuilder<consai_visualizer_msgs::msg::ShapeCircle>
{
  consai_visualizer_msgs::msg::ShapeCircle circle;
  ShapeCircleBuilder() : FillShapeColorBuilder<consai_visualizer_msgs::msg::ShapeCircle>(circle)
  {
    circle.line_size = 1;
  }
  operator consai_visualizer_msgs::msg::ShapeCircle() const { return circle; }

  ShapeCircleBuilder & center(double x, double y)
  {
    circle.center.x = x;
    circle.center.y = y;
    return *this;
  }

  ShapeCircleBuilder & center(Point p)
  {
    circle.center.x = p.x();
    circle.center.y = p.y();
    return *this;
  }

  ShapeCircleBuilder & radius(double radius)
  {
    circle.radius = radius;
    return *this;
  }

  ShapeCircleBuilder & line_size(int line_size)
  {
    circle.line_size = line_size;
    return *this;
  }

  ShapeCircleBuilder & caption(const std::string & caption)
  {
    circle.caption = caption;
    return *this;
  }
};

struct ShapeLineBuilder : public ColorBuilder<consai_visualizer_msgs::msg::ShapeLine>
{
  consai_visualizer_msgs::msg::ShapeLine line;
  ShapeLineBuilder() : ColorBuilder<consai_visualizer_msgs::msg::ShapeLine>(line) { line.size = 1; }

  operator consai_visualizer_msgs::msg::ShapeLine() const { return line; }

  ShapeLineBuilder & p1(double x, double y)
  {
    line.p1.x = x;
    line.p1.y = y;
    return *this;
  }

  ShapeLineBuilder & p1(Point p)
  {
    line.p1.x = p.x();
    line.p1.y = p.y();
    return *this;
  }

  ShapeLineBuilder & p2(double x, double y)
  {
    line.p2.x = x;
    line.p2.y = y;
    return *this;
  }

  ShapeLineBuilder & p2(Point p)
  {
    line.p2.x = p.x();
    line.p2.y = p.y();
    return *this;
  }

  ShapeLineBuilder & size(int size)
  {
    line.size = size;
    return *this;
  }

  ShapeLineBuilder & caption(const std::string & caption)
  {
    line.caption = caption;
    return *this;
  }
};

struct ShapeRectangleBuilder
: public FillShapeColorBuilder<consai_visualizer_msgs::msg::ShapeRectangle>
{
  consai_visualizer_msgs::msg::ShapeRectangle rect;
  ShapeRectangleBuilder() : FillShapeColorBuilder<consai_visualizer_msgs::msg::ShapeRectangle>(rect)
  {
  }

  operator consai_visualizer_msgs::msg::ShapeRectangle() const { return rect; }

  ShapeRectangleBuilder & center(double x, double y)
  {
    rect.center.x = x;
    rect.center.y = y;
    return *this;
  }

  ShapeRectangleBuilder & center(Point p)
  {
    rect.center.x = p.x();
    rect.center.y = p.y();
    return *this;
  }

  ShapeRectangleBuilder & size(double x, double y)
  {
    rect.width = x;
    rect.height = y;
    return *this;
  }

  ShapeRectangleBuilder & size(Point p)
  {
    rect.width = p.x();
    rect.height = p.y();
    return *this;
  }

  ShapeRectangleBuilder & line_size(int line_size)
  {
    rect.line_size = line_size;
    return *this;
  }

  ShapeRectangleBuilder & caption(const std::string & caption)
  {
    rect.caption = caption;
    return *this;
  }
};

struct ShapeRobotBuilder : public FillShapeColorBuilder<consai_visualizer_msgs::msg::ShapeRobot>
{
  consai_visualizer_msgs::msg::ShapeRobot robot;
  ShapeRobotBuilder() : FillShapeColorBuilder<consai_visualizer_msgs::msg::ShapeRobot>(robot)
  {
    robot.radius = 0.09;
    robot.line_size = 1;
  }
  operator consai_visualizer_msgs::msg::ShapeRobot() const { return robot; }

  ShapeRobotBuilder & x(double x)
  {
    robot.x = x;
    return *this;
  }

  ShapeRobotBuilder & y(double y)
  {
    robot.y = y;
    return *this;
  }

  ShapeRobotBuilder & position(Point p)
  {
    robot.x = p.x();
    robot.y = p.y();
    return *this;
  }

  ShapeRobotBuilder & theta(double theta)
  {
    robot.theta = theta;
    return *this;
  }

  ShapeRobotBuilder & id(double id)
  {
    robot.id = id;
    return *this;
  }

  ShapeRobotBuilder & color_type(bool color_type)
  {
    robot.color_type = color_type;
    return *this;
  }

  ShapeRobotBuilder & color_type_default()
  {
    robot.color_type = 0;
    return *this;
  }

  ShapeRobotBuilder & color_type_real()
  {
    robot.color_type = 1;
    return *this;
  }

  ShapeRobotBuilder & radius(double radius)
  {
    robot.radius = radius;
    return *this;
  }

  ShapeRobotBuilder & line_size(int line_size)
  {
    robot.line_size = line_size;
    return *this;
  }

  ShapeRobotBuilder & caption(const std::string & caption)
  {
    robot.caption = caption;
    return *this;
  }
};

struct ShapeTubeBuilder : public FillShapeColorBuilder<consai_visualizer_msgs::msg::ShapeTube>
{
  consai_visualizer_msgs::msg::ShapeTube tube;
  ShapeTubeBuilder() : FillShapeColorBuilder<consai_visualizer_msgs::msg::ShapeTube>(tube)
  {
    tube.line_size = 1;
  }
  operator consai_visualizer_msgs::msg::ShapeTube() const { return tube; }

  ShapeTubeBuilder & p1(double x, double y)
  {
    tube.p1.x = x;
    tube.p1.y = y;
    return *this;
  }

  ShapeTubeBuilder & p1(Point p)
  {
    tube.p1.x = p.x();
    tube.p1.y = p.y();
    return *this;
  }

  ShapeTubeBuilder & p2(double x, double y)
  {
    tube.p2.x = x;
    tube.p2.y = y;
    return *this;
  }

  ShapeTubeBuilder & p2(Point p)
  {
    tube.p2.x = p.x();
    tube.p2.y = p.y();
    return *this;
  }

  ShapeTubeBuilder & radius(double radius)
  {
    tube.radius = radius;
    return *this;
  }

  ShapeTubeBuilder & line_size(int line_size)
  {
    tube.line_size = line_size;
    return *this;
  }

  ShapeTubeBuilder & caption(const std::string & caption)
  {
    tube.caption = caption;
    return *this;
  }
};

struct ConsaiVisualizerWrapper
{
  typedef std::shared_ptr<ConsaiVisualizerWrapper> SharedPtr;

  typedef std::unique_ptr<ConsaiVisualizerWrapper> UniquePtr;

  rclcpp::Publisher<consai_visualizer_msgs::msg::Objects>::SharedPtr publisher;

  consai_visualizer_msgs::msg::Objects latest_msg;

  ConsaiVisualizerWrapper(
    rclcpp::Node & node, const std::string & layer = "default",
    const std::string & sub_layer = "default", int z_order = 0)
  : publisher(
      node.create_publisher<consai_visualizer_msgs::msg::Objects>("/visualizer_objects", 10))
  {
    latest_msg.layer = layer;
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

  void addAnnotation(consai_visualizer_msgs::msg::ShapeAnnotation annotation)
  {
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

  void addPoint(
    Point p, int size, std::string color = "white", double alpha = 1.0, std::string caption = "")
  {
    addPoint(p.x(), p.y(), size, color, alpha, caption);
  }

  void addPoint(consai_visualizer_msgs::msg::ShapePoint point)
  {
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

  void addLine(consai_visualizer_msgs::msg::ShapeLine line) { latest_msg.lines.push_back(line); }

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

  void addArc(consai_visualizer_msgs::msg::ShapeArc arc) { latest_msg.arcs.push_back(arc); }

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

  void addRect(consai_visualizer_msgs::msg::ShapeRectangle rect)
  {
    latest_msg.rects.push_back(rect);
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

  void addCircle(consai_visualizer_msgs::msg::ShapeCircle circle)
  {
    latest_msg.circles.push_back(circle);
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

  void addTube(consai_visualizer_msgs::msg::ShapeTube tube) { latest_msg.tubes.push_back(tube); }

  void addRobot(
    double id, double x, double y, double theta, std::string line_color = "white",
    std::string fill_color = "white", double alpha = 1.0, double line_size = 1,
    std::string caption = "", bool color_type = 0)
  {
    consai_visualizer_msgs::msg::ShapeRobot robot;
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

  void addRobot(consai_visualizer_msgs::msg::ShapeRobot robot)
  {
    latest_msg.robots.push_back(robot);
  }
};
}  // namespace crane
#endif  // CRANE_MSG_WRAPPERS__CONSAI_VISUALIZER_WRAPPER_HPP_
