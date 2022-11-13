// Copyright (c) 2019 SSL-Roots
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_DESCRIPTION__PARAMETERS_HPP_
#define CRANE_DESCRIPTION__PARAMETERS_HPP_

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>

using std::placeholders::_1;

namespace crane
{
namespace description
{
struct Parameters
{
public:
  int max_id;
  std::string our_side;
  std::string our_color;

  Parameters()
  {
    max_id = 15;
    our_side = "left";
    our_color = "blue";
  }
};

class ParametersClient
{
public:
  explicit ParametersClient(rclcpp::Node * node) : client(node, "crane_description") {}

  void get_parameters(crane::description::Parameters * crane_parameters)
  {
    if (!client.wait_for_service(std::chrono::seconds(5))) {
      throw std::runtime_error("Wait for service timed out");
    }
    auto parameters = client.get_parameters({"max_id", "our_side", "our_color"});

    crane_parameters->max_id = parameters[0].as_int();
    crane_parameters->our_side = parameters[1].as_string();
    crane_parameters->our_color = parameters[2].as_string();
  }

private:
  rclcpp::SyncParametersClient client;
};

}  // namespace description

}  // namespace crane

#endif  // CRANE_DESCRIPTION__PARAMETERS_HPP_
