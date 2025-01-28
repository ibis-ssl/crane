// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <crane_visualization_interfaces/msg/svg_primitive_array.hpp>
#include <unordered_map>
#include <std_msgs/msg/string.hpp>

/** crane_visualization_interfaces/msg/SvgPrimitiveArray.msg
string layer
string[] svg_primitives
 */

/**

*/
class VisualizationAggregator : public rclcpp::Node
{
public:
  VisualizationAggregator()
  : Node("visualization_aggregator")
  {
    subscriber = create_subscription<crane_visualization_interfaces::msg::SvgPrimitiveArray>("/visualizer_svgs", 10,
      [&](const crane_visualization_interfaces::msg::SvgPrimitiveArray::ConstSharedPtr & msg) {
        // store into

    });
  }

private:
  rclcpp::Subscription<crane_visualization_interfaces::msg::SvgPrimitiveArray>::SharedPtr subscriber;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;

  std::unordered_map<std::string, std::string> layers;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisualizationAggregator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
