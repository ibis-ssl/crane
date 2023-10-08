// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__RECEIVE_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__RECEIVE_PLANNER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "crane_game_analyzer/evaluations/evaluations.hpp"
#include "crane_geometry/eigen_adapter.hpp"
#include "crane_geometry/interval.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msg_wrappers/robot_command_wrapper.hpp"
#include "crane_msgs/msg/pass_info.hpp"
#include "crane_msgs/msg/receiver_plan.hpp"
#include "crane_msgs/msg/world_model.hpp"
#include "crane_msgs/srv/pass_request.hpp"
#include "crane_planner_base/planner_base.hpp"
#include "crane_planner_plugins/visibility_control.h"

namespace crane
{
/**
 * ReceivePlannerは現在進行形でパスされているボールに対して，
 * ボールを受けるロボット(passer),その次にボールを受けるロボット(receiver)を指定するだけで
 * 最適なパス地点を計算し，その2台に対する指令を生成するプランナーです
 *
 * 何もないとき：パスの通る隙の多い地点に陣取る
 * ボールが向かってきているとき：最適なパス地点を計算し，その地点に向かう
 *
 */
class ReceivePlanner : public rclcpp::Node, public PlannerBase
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
  : rclcpp::Node("receive_planner", options), PlannerBase("receive", *this)
  {
    RCLCPP_INFO(get_logger(), "initializing");
    pass_info_pub = create_publisher<crane_msgs::msg::PassInfo>("path_info", 1);
    using namespace std::placeholders;
    pass_req_service = create_service<crane_msgs::srv::PassRequest>(
      "pass_request", std::bind(&ReceivePlanner::passRequestHandle, this, _1, _2, _3));
    world_model = std::make_shared<WorldModelWrapper>(*this);
    world_model->addCallback(
      [this](void) -> void { pass_info.world_model = world_model->getMsg(); });
  }

  std::vector<crane_msgs::msg::RobotCommand> calculateControlTarget(
  const std::vector<RobotIdentifier> & robots) override
{
    auto dpps_points = getDPPSPoints(world_model->ball.pos, 0.5, 8);

    Point best_position;
    double best_score = 0.0;
    for (const auto & dpps_point : dpps_points) {
      Segment line{world_model->ball.pos, dpps_point};
      double closest_distance = [&]() -> double{
        double closest_distance = std::numeric_limits<double>::max();
        for (const auto & robot : world_model->theirs.getAvailableRobots()) {
          ClosestPoint result;
          bg::closest_point(robot->pose.pos, line, result);
          if (result.distance < closest_distance) {
            closest_distance = result.distance;
          }
        }
        return closest_distance;
      }();

      if(closest_distance < 0.4) {
        continue;
      }

      const double score = getLargestGoalAngleWidthFromPosition(dpps_point);
      if (score > best_score) {
        best_score = score;
        best_position = dpps_point;
      }
    }

    std::vector<crane_msgs::msg::RobotCommand> commands;
    for(const auto & robot: robots){
      crane::RobotCommandWrapper target(robot.robot_id, world_model);
      target.setTargetPosition(best_position);
      target.setTargetTheta(getAngle(world_model->ball.pos - best_position));
      commands.push_back(target.getMsg());
    }

    return commands;
}
auto getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots)
  -> std::vector<uint8_t> override
{
    return this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots, [this](const std::shared_ptr<RobotInfo> & robot) {
        return 100. / world_model->getSquareDistanceFromRobotToBall({true, robot->id});
      });
}

  void passRequestHandle(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<crane_msgs::srv::PassRequest::Request> request,
    const std::shared_ptr<crane_msgs::srv::PassRequest::Response> response)
  {
    RCLCPP_INFO(get_logger(), "receive pass request!");
    (void)request_header;
    pass_info.passer_id = request->pass_plan.passer_id;
    pass_info.receiver_id = request->pass_plan.receiver_id;
    auto & ball = world_model->ball;
    auto pos = world_model->ours.robots.at(pass_info.passer_id.data)->pose.pos;
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

    auto & recv_pos = pass_info.passer_receive_position;
    recv_pos.x = target.x();
    recv_pos.y = target.y();
    recv_pos.z = 0.0;
    pass_info_pub->publish(pass_info);

    //            std::vector<std::pair<double, Point>> getPositionsWithScore(Segment ball_line, Point next_target);
    RobotIdentifier receiver_id;
    receiver_id.is_ours = true;
    receiver_id.robot_id = pass_info.receiver_id.data;
    auto receiver = world_model->getRobot(receiver_id);
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

  auto getLargestGoalAngleWidthFromPosition(const Point point) -> double
  {
    Interval goal_range;

    auto goal_posts = world_model->getTheirGoalPosts();
    goal_range.append(getAngle(goal_posts.first - point), getAngle(goal_posts.second - point));

    for (auto & enemy : world_model->theirs.robots) {
      double distance = (point - enemy->pose.pos).norm();
      constexpr double MACHINE_RADIUS = 0.1;

      double center_angle = getAngle(enemy->pose.pos - point);
      double diff_angle =
        atan(MACHINE_RADIUS / std::sqrt(distance * distance - MACHINE_RADIUS * MACHINE_RADIUS));

      goal_range.erase(center_angle - diff_angle, center_angle + diff_angle);
    }

    auto largest_interval = goal_range.getLargestInterval();
    return largest_interval.second - largest_interval.first;
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

  std::vector<Point> getDPPSPoints(Point center, double r_resolution, int theta_div_num){
    std::vector<Point> points;
    for (int theta_index = 0; theta_index < theta_div_num; theta_index++){
      double theta = 2.0 * M_PI * theta_index / theta_div_num;
      for (double r = r_resolution; r <= 10.0; r += r_resolution){
        points.emplace_back(Point(center.x() + r * cos(theta), center.y() + r * sin(theta)));
      }
    }
    points.erase(std::remove_if(points.begin(), points.end(), [&](const auto & point){
      return not world_model->isFieldInside(point);
    }), points.end());

    return points;
  }

  double getPointScore(Point p, Point next_target)
  {
    double nearest_dist;
    RobotIdentifier receiver{true, static_cast<uint8_t>(session_info.receiver_id)};
    return evaluation::getNextTargetVisibleScore(p, next_target, world_model) *
           evaluation::getReachScore(receiver, p, nearest_dist, world_model) *
           evaluation::getAngleScore(receiver, p, next_target, world_model) *
           evaluation::getEnemyDistanceScore(p, world_model);
  }

  WorldModelWrapper::SharedPtr world_model;

private:
  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Publisher<crane_msgs::msg::PassInfo>::SharedPtr pass_info_pub;

  rclcpp::Service<crane_msgs::srv::PassRequest>::SharedPtr pass_req_service;

  crane_msgs::msg::PassInfo pass_info;
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__RECEIVE_PLANNER_HPP_
