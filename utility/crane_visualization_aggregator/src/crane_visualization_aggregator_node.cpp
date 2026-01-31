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

struct LayerState
{
  std::vector<std::string> svg_primitives;
  std::optional<rclcpp::Time> expiration;  // 期限（nullopt = 無限）
};

class VisualizationAggregator : public rclcpp::Node
{
public:
  VisualizationAggregator() : Node("visualization_aggregator")
  {
    updates_sub_ = create_subscription<crane_visualization_interfaces::msg::SvgUpdates>(
      "/visualizer_svgs", rclcpp::SensorDataQoS(),
      [&](const crane_visualization_interfaces::msg::SvgUpdates::ConstSharedPtr & msg) {
        for (const auto & update : msg->updates) {
          auto & current = layers[update.layer];

          // durationから期限を計算
          if (update.duration > 0) {
            current.expiration = this->now() + rclcpp::Duration::from_seconds(update.duration);
          } else {
            current.expiration = std::nullopt;  // 無限
          }

          // 既存の処理
          if (update.operation == "replace") {
            current.svg_primitives = update.svg_primitives;
          } else if (update.operation == "append") {
            current.svg_primitives.insert(
              current.svg_primitives.end(), update.svg_primitives.begin(),
              update.svg_primitives.end());
          } else if (update.operation == "clear") {
            current.svg_primitives.clear();
          }
        }
      });
    publisher =
      create_publisher<crane_visualization_interfaces::msg::SvgSnapshot>("/aggregated_svgs", 10);
    timer = create_wall_timer(std::chrono::milliseconds(5000), [this]() { publishSnapshot(); });

    // 期限切れレイヤーをクリーンアップするタイマー（100ms周期）
    cleanup_timer =
      create_wall_timer(std::chrono::milliseconds(100), [this]() { cleanupExpiredLayers(); });

    publishSnapshot();
  }

private:
  void publishSnapshot()
  {
    crane_visualization_interfaces::msg::SvgSnapshot msg;
    msg.header.stamp = this->now();
    msg.epoch = epoch_;
    msg.seq = seq_++;
    for (const auto & [layer, state] : layers) {
      crane_visualization_interfaces::msg::SvgLayerSnapshot layer_msg;
      layer_msg.layer = layer;
      layer_msg.svg_primitives = state.svg_primitives;
      msg.layers.push_back(layer_msg);
    }
    publisher->publish(msg);
  }

  void cleanupExpiredLayers()
  {
    auto now = this->now();
    for (auto it = layers.begin(); it != layers.end();) {
      if (it->second.expiration && now > *it->second.expiration) {
        it = layers.erase(it);  // 期限切れ: 削除
      } else {
        ++it;
      }
    }
  }

  rclcpp::Subscription<crane_visualization_interfaces::msg::SvgUpdates>::SharedPtr updates_sub_;

  rclcpp::Publisher<crane_visualization_interfaces::msg::SvgSnapshot>::SharedPtr publisher;

  std::unordered_map<std::string, LayerState> layers;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::TimerBase::SharedPtr cleanup_timer;

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
