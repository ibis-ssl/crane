// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cv_bridge/cv_bridge.h>

#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class GridMapToImageNode : public rclcpp::Node
{
public:
  GridMapToImageNode() : Node("grid_map_to_image_node")
  {
    // grid_mapの購読
    grid_map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      "/local_planner/grid_map", 10,
      std::bind(&GridMapToImageNode::gridMapCallback, this, std::placeholders::_1));
  }

private:
  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(*msg, map);

    // GridMap内の全てのレイヤーを取得
    for (const auto & layer : map.getLayers()) {
      // GridMapをcv::Matに変換
      cv::Mat image;
      grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, layer, CV_8UC1, 0.0, 1.0, image);

      // cv::Matをsensor_msgs/Imageに変換
      auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image).toImageMsg();
      image_msg->header.stamp = this->get_clock()->now();

      // トピック名をレイヤー名に基づいて生成し、「/」を「_」に置換
      std::string topic_name = "grid_map_image_" + layer;
      std::replace(topic_name.begin(), topic_name.end(), '/', '_');

      // 該当トピックのパブリッシャーが存在しない場合は作成
      if (image_pubs_.find(topic_name) == image_pubs_.end()) {
        image_pubs_[topic_name] = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
      }

      // Imageメッセージの発行
      image_pubs_[topic_name]->publish(*image_msg);
    }
  }

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
    image_pubs_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridMapToImageNode>());
  rclcpp::shutdown();
  return 0;
}
