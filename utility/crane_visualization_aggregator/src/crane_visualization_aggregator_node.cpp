// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <crane_visualization_interfaces/msg/svg_layer_snapshot.hpp>
#include <crane_visualization_interfaces/msg/svg_snapshot.hpp>
#include <crane_visualization_interfaces/msg/svg_updates.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <unordered_map>

/** crane_visualization_interfaces/msg/SvgLayerSnapshot.msg
string layer
string[] svg_primitives
 */
class VisualizationAggregator : public rclcpp::Node
{
public:
  VisualizationAggregator() : Node("visualization_aggregator")
  {
    updates_sub_ = create_subscription<crane_visualization_interfaces::msg::SvgUpdates>(
      "/visualizer_svgs", rclcpp::SensorDataQoS(),
      [&](const crane_visualization_interfaces::msg::SvgUpdates::ConstSharedPtr & msg) {
        // Apply updates per layer (simple)
        for (const auto & update : msg->updates) {
          auto & current = layers[update.layer];
          if (update.operation == "replace") {
            current = update.svg_primitives;
          } else if (update.operation == "append") {
            current.insert(
              current.end(), update.svg_primitives.begin(), update.svg_primitives.end());
          } else if (update.operation == "clear") {
            current.clear();
          }
        }
      });
    publisher =
      create_publisher<crane_visualization_interfaces::msg::SvgSnapshot>("/aggregated_svgs", 10);
    // Publish full snapshot periodically (fixed interval)
    timer = create_wall_timer(std::chrono::milliseconds(5000), [this]() { publishSnapshot(); });
    // Emit an initial snapshot immediately so subscribers see traffic
    publishSnapshot();
  }

private:
  void publishSnapshot()
  {
    crane_visualization_interfaces::msg::SvgSnapshot msg;
    msg.header.stamp = this->now();
    msg.epoch = epoch_;
    msg.seq = seq_++;
    for (const auto & [layer, primitives] : layers) {
      crane_visualization_interfaces::msg::SvgLayerSnapshot layer_msg;
      layer_msg.layer = layer;
      layer_msg.svg_primitives = primitives;
      msg.layers.push_back(layer_msg);
    }
    publisher->publish(msg);
  }

  rclcpp::Subscription<crane_visualization_interfaces::msg::SvgUpdates>::SharedPtr updates_sub_;

  rclcpp::Publisher<crane_visualization_interfaces::msg::SvgSnapshot>::SharedPtr publisher;

  std::unordered_map<std::string, std::vector<std::string>> layers;

  rclcpp::TimerBase::SharedPtr timer;

  uint32_t epoch_ = 0;
  uint32_t seq_ = 0;
};

auto main(int argc, char ** argv) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualizationAggregator>());
  rclcpp::shutdown();
  return 0;
}
