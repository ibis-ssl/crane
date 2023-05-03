// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__SKILL__STOP_HPP_
#define CRANE_BT_EXECUTOR__SKILL__STOP_HPP_

#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "crane_bt_executor/composite/composite.hpp"
#include "crane_bt_executor/robot_io.hpp"
#include "crane_bt_executor/utils/target.hpp"

class Stop : public Composite
{
public:
  explicit Stop(float stop_time = -1) : stop_time_(stop_time), clock_(RCL_ROS_TIME) {}

  Status run(WorldModelWrapper::SharedPtr world_model, RobotIO robot) override
  {
    if (!configured_) {
      target_theta_ = robot.info->pose.theta;
      configured_ = true;
      start_time_ = clock_.now();
    }

    if (stop_time_ > 0 && (clock_.now() - start_time_).seconds() >= stop_time_) {
      return Status::SUCCESS;
    }
    robot.builder->stop();
    return Status::RUNNING;
  }

private:
  bool configured_ = false;
  float stop_time_;
  rclcpp::Time start_time_;
  rclcpp::Clock clock_;

  float target_theta_;
};
#endif  // CRANE_BT_EXECUTOR__SKILL__STOP_HPP_
