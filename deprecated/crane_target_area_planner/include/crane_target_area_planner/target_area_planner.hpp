// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TARGET_AREA_PLANNER__TARGET_AREA_PLANNER_HPP_
#define CRANE_TARGET_AREA_PLANNER__TARGET_AREA_PLANNER_HPP_

#include <Eigen/Core>
#include <boost/geometry/extensions/algorithms/closest_point.hpp>
#include <rclcpp/rclcpp.hpp>

#include "crane_geometry/boost_geometry.hpp"
#include "crane_geometry/eigen_adapter.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/pass_info.hpp"
#include "crane_target_area_planner/visibility_control.hpp"
namespace crane
{
struct FutureKick
{
  float remaining_time;
  Eigen::Vector2f position;
};
class TargetAreaPlanner : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit TargetAreaPlanner(const rclcpp::NodeOptions & options)
  : rclcpp::Node("target_area_planner", options)
  {
    world_model_ = std::make_shared<WorldModelWrapper>();
    pass_info_sub_ = create_subscription<crane_msgs::msg::PassInfo>(
      "/receiver_planner/pass_info", 1,
      std::bind(&TargetAreaPlanner::passInfoCallback, this, std::placeholders::_1));
    pass_info_pub_ = create_publisher<crane_msgs::msg::PassInfo>("pass_info", 1);
  }
  void passInfoCallback(const crane_msgs::msg::PassInfo::SharedPtr msg)
  {
    world_model_->update(msg->world_model);
    FutureKick kick;
    kick.remaining_time = msg->passer_receive_time_s.data;
    kick.position << msg->passer_receive_position.x, msg->passer_receive_position.y;
    auto a = calcTarget(kick, msg->receiver_id.data);
  }

private:
  Eigen::Vector2f calcTarget(FutureKick kick, int receiver_id)
  {
    auto pos = world_model_->ours.robots.at(receiver_id)->pose.pos;
    float grid_size = 0.3f;
    float window_radius = 3.0f;

    float max_score = 0.0f;
    Eigen::Vector2f target;
    for (float x = pos.x() - window_radius; x <= pos.x() + window_radius; x += grid_size) {
      for (float y = pos.y() - window_radius; y <= pos.y() + window_radius; y += grid_size) {
        float score = calcScore(x, y, kick);
        if (score > max_score) {
          target << x, y;
        }
      }
    }
    return target;
  }
  float calcScore(float x, float y, const FutureKick & kick)
  {
    auto field = world_model_->field_size;
    // field limitation
    if (abs(x) > field.x() * 0.5f || abs(y) > field.y() * 0.5f) {
      return 0.0f;
    }
    Eigen::Vector2f end_pos(x, y);
    float enemy_dist = 100.0f;
    //    float
    auto start_pos = kick.position;
    Segment seg;
    seg.first = start_pos;
    seg.second = end_pos;
    auto pass_vec = end_pos - start_pos;

    using ClosestPoint = bg::closest_point_result<Point>;
    for (auto enemy : world_model_->theirs.robots) {
      auto & epos = enemy->pose.pos;
      auto epos_vec = epos - start_pos;
      ClosestPoint result;
      bg::closest_point(seg, epos, result);
    }
    // TODO(HansRobo): check enemy block
    // TODO(HansRobo):
    return 0.0f;
  }
  rclcpp::Subscription<crane_msgs::msg::PassInfo>::SharedPtr pass_info_sub_;
  rclcpp::Publisher<crane_msgs::msg::PassInfo>::SharedPtr pass_info_pub_;
  WorldModelWrapper::SharedPtr world_model_;
};
}  // namespace crane
#endif  // CRANE_TARGET_AREA_PLANNER__TARGET_AREA_PLANNER_HPP_
