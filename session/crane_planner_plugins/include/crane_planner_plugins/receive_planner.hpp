// Copyright (c) 2021 ibis-ssl
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

#ifndef CRANE_RECEIVE_PLANNER__RECEIVE_PLANNER_HPP_
#define CRANE_RECEIVE_PLANNER__RECEIVE_PLANNER_HPP_

#include "crane_planner_base/planner_base.hpp"
#include <functional>
#include <memory>

#include "crane_game_analyzer/evaluations/evaluations.hpp"
#include "crane_geometry/eigen_adapter.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/pass_info.hpp"
#include "crane_msgs/msg/receiver_plan.hpp"
#include "crane_msgs/msg/world_model.hpp"
#include "crane_msgs/srv/pass_request.hpp"
#include "crane_planner_plugins/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

namespace crane
{

/**
 * ReceivePlannerは現在進行形でパスされているボールに対して，
 * ボールを受けるロボット(passer),その次にボールを受けるロボット(receiver)を指定するだけで
 * 最適なパス地点を計算し，その2台に対する指令を生成するプランナーです
 */
class ReceivePlanner : public rclcpp::Node
{
public:
  enum class ReceivePhase {
    NONE,
    MOVE_ROUGH,
    MOVE_TO_EXPECTED_BALL_LINE,
    MOVE_TO_ACTUAL_BALL_LINE,
  };
  struct SessionInfo
  {
    int receiver_id;
  } session_info;

  struct PositionsWithScore
  {
    Point passer_pos;
    Point receiver_pos;
    double score;
  };

  COMPOSITION_PUBLIC
  explicit ReceivePlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("receive_planner", options)
  {
    pass_info_pub_ = create_publisher<crane_msgs::msg::PassInfo>("path_info", 1);
    using namespace std::placeholders;
    pass_req_service_ = create_service<crane_msgs::srv::PassRequest>(
      "pass_request", std::bind(&ReceivePlanner::passRequestHandle, this, _1, _2, _3));
    world_model_ = std::make_shared<WorldModelWrapper>(*this);
    world_model_->addCallback(
      [this](void) -> void { pass_info_.world_model = world_model_->getMsg(); });
  }

  void passRequestHandle(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<crane_msgs::srv::PassRequest::Request> request,
    const std::shared_ptr<crane_msgs::srv::PassRequest::Response> response)
  {
    RCLCPP_INFO(get_logger(), "receive pass request!");
    (void)request_header;
    pass_info_.passer_id = request->pass_plan.passer_id;
    pass_info_.receiver_id = request->pass_plan.receiver_id;
    auto & ball = world_model_->ball;
    auto pos = world_model_->ours.robots.at(pass_info_.passer_id.data)->pose.pos;
    //  こちらへ向かう速度成分
    float ball_vel = ball.vel.dot((pos - ball.pos).normalized());
    Segment ball_line(ball.pos, (ball.pos + ball.vel.normalized() * (ball.pos - pos).norm()));
    Point target;
    if (ball_vel > 0.5f) {
      //  ボールの進路上に移動
      ClosestPoint result;
      bg::closest_point(pos, ball_line, result);
      target = result.closest_point;
    }

    auto & recv_pos = pass_info_.passer_receive_position;
    recv_pos.x = target.x();
    recv_pos.y = target.y();
    recv_pos.z = 0.0;
    pass_info_pub_->publish(pass_info_);

    //            std::vector<std::pair<double, Point>> getPositionsWithScore(Segment ball_line, Point next_target);
    RobotIdentifier receiver_id;
    receiver_id.is_ours = true;
    receiver_id.robot_id = pass_info_.receiver_id.data;
    auto receiver = world_model_->getRobot(receiver_id);
    if (!receiver) {
      return;
    }

    std::vector<PositionsWithScore> positions_with_score;

    auto receive_pos_candidates = getPoints(receiver->pose.pos, 0.05, 20);
    for (auto receive_pos : receive_pos_candidates) {
      auto pos_score = getPositionsWithScore(ball_line, receive_pos);
      auto max_score_pos = std::max_element(
        pos_score.begin(), pos_score.end(),
        [](const auto & a, const auto & b) { return a.first < b.first; });
      PositionsWithScore record;
      record.score = max_score_pos->first;
      record.passer_pos = max_score_pos->second;
      record.receiver_pos = receive_pos;
      positions_with_score.emplace_back(record);
    }

    auto max_record = std::max_element(
      positions_with_score.begin(), positions_with_score.end(),
      [](const auto & a, const auto & b) { return a.score < b.score; });

    std::tie(response->passer_target_pose.theta, response->receiver_target_pose.theta) =
      calcRobotsTargetAngle(*max_record, ball_line);

    response->passer_target_pose.x = max_record->passer_pos.x();
    response->passer_target_pose.y = max_record->passer_pos.y();

    response->receiver_target_pose.x = max_record->receiver_pos.x();
    response->receiver_target_pose.y = max_record->receiver_pos.y();

    response->success = true;
    response->message = "publish receiver point successfully";
  }

  std::pair<double, double> calcRobotsTargetAngle(PositionsWithScore record, Segment ball_line)
  {
    std::pair<double, double> ret;
    // calculate passer angle
    auto passer2ball = (ball_line.first - record.passer_pos).normalized();
    auto passer2receiver = (record.receiver_pos - record.passer_pos).normalized();
    auto passer_direction = passer2receiver + passer2ball;
    ret.first = atan2(passer_direction.y(), passer_direction.x());

    ret.second = atan2(-passer2receiver.y(), -passer2receiver.x());

    return ret;
  }
  std::vector<std::pair<double, Point>> getPositionsWithScore(Segment ball_line, Point next_target)
  {
    auto points = getPoints(ball_line, 0.05);
    std::vector<std::pair<double, Point>> position_with_score;
    for (auto point : points) {
      double score = getPointScore(point, next_target);
      position_with_score.push_back(std::make_pair(score, point));
    }
    return position_with_score;
  }

  std::vector<Point> getPoints(Segment ball_line, double interval)
  {
    std::vector<Point> points;
    float ball_line_len = (ball_line.first - ball_line.second).norm();
    auto norm_vec = (ball_line.second - ball_line.first).normalized();
    for (double d = 0.0; d <= ball_line_len; d += interval) {
      points.emplace_back(ball_line.first + d * norm_vec);
    }
    return points;
  }

  std::vector<Point> getPoints(Point center, float unit, int unit_num)
  {
    std::vector<Point> points;
    for (float x = center.x() - unit * (unit_num / 2.f); x <= center.x() + unit * (unit_num / 2.f);
         x += unit) {
      for (float y = center.y() - unit * (unit_num / 2.f);
           y <= center.y() + unit * (unit_num / 2.f); y += unit) {
        points.emplace_back(Point(x, y));
      }
    }
    return points;
  }

  double getPointScore(Point p, Point next_target)
  {
    double nearest_dist;
    RobotIdentifier receiver{true, static_cast<uint8_t>(session_info.receiver_id)};
    return evaluation::getNextTargetVisibleScore(p, next_target, world_model_) *
           evaluation::getReachScore(receiver, p, nearest_dist, world_model_) *
           evaluation::getAngleScore(receiver, p, next_target, world_model_) *
           evaluation::getEnemyDistanceScore(p, world_model_);
  }

  WorldModelWrapper::SharedPtr world_model_;
private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<crane_msgs::msg::PassInfo>::SharedPtr pass_info_pub_;
  rclcpp::Service<crane_msgs::srv::PassRequest>::SharedPtr pass_req_service_;
  crane_msgs::msg::PassInfo pass_info_;
};

}  // namespace crane
#endif  // CRANE_RECEIVE_PLANNER__RECEIVE_PLANNER_HPP_
