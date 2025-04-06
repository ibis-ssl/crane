// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <crane_visualization_interfaces/msg/svg_layer_array.hpp>
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
    subscriber = create_subscription<crane_visualization_interfaces::msg::SvgLayerArray>(
      "/visualizer_svgs", rclcpp::SensorDataQoS(),
      [&](const crane_visualization_interfaces::msg::SvgLayerArray::ConstSharedPtr & msg) {
        // store into　layers
        for (const auto & layer_msg : msg->svg_primitive_arrays) {
          layers[layer_msg.layer] = layer_msg.svg_primitives;
        }
      });
    publisher =
      create_publisher<crane_visualization_interfaces::msg::SvgLayerArray>("/aggregated_svgs", 10);
    timer = create_wall_timer(std::chrono::milliseconds(16), [this]() {
      crane_visualization_interfaces::msg::SvgLayerArray msg;

      for (const auto & [layer, primitives] : layers) {
        crane_visualization_interfaces::msg::SvgPrimitiveArray layer_msg;
        layer_msg.layer = layer;
        layer_msg.svg_primitives = primitives;
        msg.svg_primitive_arrays.push_back(layer_msg);
      }
      publisher->publish(msg);
    });
  }

private:
  rclcpp::Subscription<crane_visualization_interfaces::msg::SvgLayerArray>::SharedPtr subscriber;
  rclcpp::Publisher<crane_visualization_interfaces::msg::SvgLayerArray>::SharedPtr publisher;

  std::unordered_map<std::string, std::vector<std::string>> layers;

  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualizationAggregator>());
  rclcpp::shutdown();
  return 0;
}
