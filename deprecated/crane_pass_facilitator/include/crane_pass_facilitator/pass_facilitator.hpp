// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PASS_FACILITATOR__PASS_FACILITATOR_HPP_
#define CRANE_PASS_FACILITATOR__PASS_FACILITATOR_HPP_

#include <algorithm>
#include <functional>
#include <memory>

#include "crane_geometry/eigen_adapter.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/srv/pass_request.hpp"
#include "crane_pass_facilitator/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

namespace crane
{
class PassFacilitator : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit PassFacilitator(const rclcpp::NodeOptions & options)
  : rclcpp::Node("pass_facilitator", options)
  {
    using std::chrono_literals::operator""ms;
    receive_point_pub_ = create_publisher<geometry_msgs::msg::Point>("receive_point", 1);
    pass_req_client_ = create_client<crane_msgs::srv::PassRequest>("pass_request");
    auto req = std::make_shared<crane_msgs::srv::PassRequest::Request>();
    req->pass.receiver_id.data = 0;
    req->pass.passer_id.data = 1;
    auto response_future = pass_req_client_->async_send_request(req);
    RCLCPP_INFO(get_logger(), "pass request sent");

    crane_msgs::srv::PassRequest::Response::SharedPtr response;
    // Wait for the result.
    if (
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), response_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
      response = response_future.get();
      RCLCPP_INFO(get_logger(), "pass request success");
      RCLCPP_INFO(get_logger(), "%s", response->message.data());
    } else {
      RCLCPP_INFO(get_logger(), "pass request failed");
    }
  }

  auto getFieldPoints()
  {
    auto points = world_model_->generateFieldPoints(0.5f);
    auto new_end = std::remove_if(
      points.begin(), points.end(), [this](auto & p) { return world_model_->isGoalArea(p); });
    points.erase(new_end, points.end());
    return points;
  }
  void calc()
  {
    auto points = getFieldPoints();

    // 2つのスコアを計算
    std::vector<float> avoid_pass_score(points.size()), robot_dist_score(points.size());
    std::transform(points.begin(), points.end(), avoid_pass_score.begin(), [this](auto p) {
      return getAvoidPassCutScore(p);
    });
    std::transform(points.begin(), points.end(), robot_dist_score.begin(), [this](auto p) {
      return getRobotDistanceScore(p);
    });

    // 2つのスコアをかけ合わせて最終スコアを計算
    std::vector<float> score(points.size());
    std::transform(
      avoid_pass_score.begin(), avoid_pass_score.end(), robot_dist_score.begin(), score.begin(),
      std::multiplies<float>());
  }

  float getAvoidPassCutScore(Point p) { return 1.f; }

  /**
   * @brief 敵より味方ロボットの方が先に到達できる
   * @param p
   * @return
   */
  float getRobotDistanceScore(Point p) { return 1.f; }

  void world_model_callback(const crane_msgs::msg::WorldModel::SharedPtr msg)
  {
    world_model_->update(*msg);
  }

private:
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr sub_world_model_;
  WorldModelWrapper::SharedPtr world_model_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr receive_point_pub_;
  rclcpp::Client<crane_msgs::srv::PassRequest>::SharedPtr pass_req_client_;
};

}  // namespace crane
#endif  // CRANE_PASS_FACILITATOR__PASS_FACILITATOR_HPP_
