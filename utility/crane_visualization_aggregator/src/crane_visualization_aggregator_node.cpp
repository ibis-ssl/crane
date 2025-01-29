// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <crane_visualization_interfaces/msg/svg_primitive_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <unordered_map>

/** crane_visualization_interfaces/msg/SvgPrimitiveArray.msg
string layer
string[] svg_primitives
 */
class VisualizationAggregator : public rclcpp::Node
{
public:
  VisualizationAggregator() : Node("visualization_aggregator")
  {
    subscriber = create_subscription<crane_visualization_interfaces::msg::SvgPrimitiveArray>(
      "/visualizer_svgs", rclcpp::SensorDataQoS(),
      [&](const crane_visualization_interfaces::msg::SvgPrimitiveArray::ConstSharedPtr & msg) {
        // store into　layers
        layers.try_emplace(msg->layer, msg->svg_primitives);
      });
    publisher = create_publisher<std_msgs::msg::String>("/aggregated_svgs", 10);
    timer = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      std::stringstream svg;
      svg << R"(<?xml version="1.0" encoding="UTF-8"?>)" << std::endl;
      svg << R"(<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="1000">)" << std::endl;

      for (const auto & [layer, primitives] : layers) {
        svg << "\t<g data-layer=\"" << layer << "\">" << std::endl;
        for (const auto & primitive : primitives) {
          svg << "\t\t" << primitive << std::endl;
        }
        svg << "\t</g>" << std::endl;
      }

      svg << "</svg>";

      std_msgs::msg::String msg;
      msg.data = svg.str();
      publisher->publish(msg);
    });
  }

private:
  rclcpp::Subscription<crane_visualization_interfaces::msg::SvgPrimitiveArray>::SharedPtr
    subscriber;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;

  std::unordered_map<std::string, std::vector<std::string>> layers;

  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisualizationAggregator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
