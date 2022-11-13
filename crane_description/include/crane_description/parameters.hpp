// Copyright (c) 2019 SSL-Roots
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

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
