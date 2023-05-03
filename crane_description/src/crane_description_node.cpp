// Copyright (c) 2019 SSL-Roots
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <memory>

#include <rclcpp/rclcpp.hpp>

class DescriptionNode : public rclcpp::Node
{
public:
  DescriptionNode() : Node("crane_description")
  {
    this->declare_parameter("max_id", 15);
    this->declare_parameter("our_side", "left");
    this->declare_parameter("our_color", "blue");
  }

private:
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DescriptionNode>());
  rclcpp::shutdown();
  return 0;
}
