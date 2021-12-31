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

#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"

#include "crane_receive_planner/visibility_control.h"
#include "crane_msgs/msg/world_model.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/receiver_plan.hpp"
#include "crane_msgs/msg/pass_info.hpp"
#include "crane_geometry/eigen_adapter.hpp"
#include "crane_msgs/srv/pass_request.hpp"

namespace crane {
    class ReceivePlanner : public rclcpp::Node {
    public:
        enum class ReceivePhase {
            NONE,
            MOVE_ROUGH,
            MOVE_TO_EXPECTED_BALL_LINE,
            MOVE_TO_ACTUAL_BALL_LINE,
        };
        struct SessionInfo {
            int receiver_id;
        } session_info;

        COMPOSITION_PUBLIC
        explicit ReceivePlanner(const rclcpp::NodeOptions &options)
                : rclcpp::Node("receive_planner", options) {
            pass_info_pub_ = create_publisher<crane_msgs::msg::PassInfo>("path_info", 1);
            using namespace std::placeholders;
            pass_req_service_ = create_service<crane_msgs::srv::PassRequest>("pass_request", std::bind(
                    &ReceivePlanner::passRequestHandle, this, _1, _2, _3));
            world_model_ = std::make_shared<WorldModelWrapper>();
        }

        void passRequestHandle(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<crane_msgs::srv::PassRequest::Request> request,
                               const std::shared_ptr<crane_msgs::srv::PassRequest::Response> response) {
            RCLCPP_INFO(get_logger(), "receive pass request!");
            (void) request_header;
            pass_info_.passer_id = request->pass.passer_id;
            pass_info_.receiver_id = request->pass.receiver_id;
            publishReceivePoint(request->pass.passer_id.data);
            response->success = true;
            response->message = "publish receiver point successfully";
        }

        void publishReceivePoint(int passer_id) {
            auto &ball = world_model_->ball;
            auto pos = world_model_->ours.robots.at(passer_id)->pose.pos;
            //  こちらへ向かう速度成分
            float ball_vel = ball.vel.dot((pos - ball.pos).normalized());
            Point target;
            if (ball_vel > 0.5f) {
                //  ボールの進路上に移動
                ClosestPoint result;
                Segment ball_line(ball.pos, (ball.pos + ball.vel.normalized() * (ball.pos - pos).norm()));
                bg::closest_point(pos, ball_line, result);
                target = result.closest_point;
            }

            auto &recv_pos = pass_info_.passer_receive_position;
            recv_pos.x = target.x();
            recv_pos.y = target.y();
            recv_pos.z = 0.0;
            pass_info_pub_->publish(pass_info_);
        }

        void calc(){

        }

        std::vector<std::pair<double, Point>> getPositionsWithScore(Segment ball_line, Point next_target) {
            auto points = getPoints(ball_line, 0.05);
            std::vector<std::pair<double, Point>> position_with_score;
            for (auto point: points) {
                double score = getPointScore(point, next_target);
                position_with_score.push_back(std::make_pair(score, point));
            }
            return position_with_score;
        }

        std::vector<Point> getPoints(Segment ball_line, double interval) {
            std::vector<Point> points;
            float ball_line_len = (ball_line.first - ball_line.second).norm();
            auto norm_vec = (ball_line.second - ball_line.first).normalized();
            for (double d = 0.0; d <= ball_line_len; d += interval) {
                points.emplace_back(ball_line.first + d * norm_vec);
            }
            return points;
        }

        double getPointScore(Point p, Point next_target) {
            double nearest_dist;
            return getNextTargetVisibleScore(p, next_target) * getReachScore(p, nearest_dist) * getAngleScore(p,next_target) * getEnemyDistanceScore(p);
        }

        /**
         * @brief 次の目標地点までのパスコースが敵ロボットに遮られていないかを評価する
         * @param p 評価する座標(ロボットがボールに触れてパスする地点)
         * @param next_target ボールを送り込む目標地点
         * @return 評価値(0~1)
         * @note 0 : パスカット可能性高（次のパスコースから敵が近い）
         * @note 1 : パスカット可能性低（次のパスコースから敵が遠い）
         */
        double getNextTargetVisibleScore(Point p, Point next_target) {
            auto ball_line_norm = (next_target - p).normalized();
            // 次のパスライン単位ベクトルと敵方向の内積で評価（パスラインと敵方向のパスコースから角度差分のcos）
            double max_cos = 0.0;
            for(auto enemy : world_model_->theirs.robots){
                if(enemy->available){
                    auto norm = (enemy->pose.pos - p).normalized();
                    double cos = ball_line_norm.dot(norm);
                    max_cos = std::max(max_cos,cos);
                }
            }
            // 角度が大きい(cosが小さい)ほど安全
            return 1 - max_cos;
        }

        /**
         * @brief パス地点までの味方ロボットの到達性を評価する
         * @param p 評価する座標(ロボットがボールに触れてパスする地点)
         * @param nearest_dist 受け手ロボットから現在のパスコースへの最短距離
         * @return 評価値(0~1)
         * @note 0 : 到達性低(パス地点にパスロボットが遠い)
         * @note 1 : 到達性高(パス地点にパスロボットが近い)
         */
        double getReachScore(Point p, double nearest_dist) {
            auto &pos = world_model_->ours.robots.at(session_info.receiver_id)->pose.pos;
            double distance = (p - pos).norm();
            return nearest_dist / distance;
        }

        /**
         * @brief キック角度の難易度スコア（入射・反射角が大きいキックは難しい）
         * @param p 評価する座標(ロボットがボールに触れてパスする地点)
         * @param next_target next_target ボールを送り込む目標地点
         * @return 評価値(0~1)
         * @note 0 : キックが難しい(キック角度が大きい)
         * @note 1 : キックが簡単(キック角度が小さい)
         */
        double getAngleScore(Point p,Point next_target) {
            // 入射角＋反射角のcosを計算(内積を使用)
            auto &pos = world_model_->ours.robots.at(session_info.receiver_id)->pose.pos;
            auto current_pass_line =  (world_model_->ball.pos- p).normalized();
            auto next_pass_line = (next_target - p).normalized();
            float dot = current_pass_line.dot(next_pass_line);
            return dot;
        }

        /**
         * @brief パス地点の安全性を評価する（どれだけ敵ロボットから遠いか）
         * @param p 評価する座標(ロボットがボールに触れてパスする地点)
         * @return 評価値(0~1)
         * @note 0 : 危険(敵が近い)
         * @note 1 : 安全(敵が遠い)
         */
        double getEnemyDistanceScore(Point p){
            // 一番近い敵ロボットからの距離を求める
            double min_sq_dist = 100.0f;
            for(auto enemy : world_model_->theirs.robots){
                if(enemy->available){
                    double sq_dist = (enemy->pose.pos - p).squaredNorm();
                    min_sq_dist = std::min(min_sq_dist,sq_dist);
                }
            }
            //最大距離は3m(それ以上は評価値を1(安全)とする)
            min_sq_dist = std::min(min_sq_dist,3.0*3.0);
            return sqrt(min_sq_dist)/3.0;
        }
        void world_model_callback(const crane_msgs::msg::WorldModel::SharedPtr msg) {
            world_model_->update(*msg);
            pass_info_.world_model = *msg;
        }
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr sub_world_model_;
        std::shared_ptr<WorldModelWrapper> world_model_;
        rclcpp::Publisher<crane_msgs::msg::PassInfo>::SharedPtr pass_info_pub_;
        rclcpp::Service<crane_msgs::srv::PassRequest>::SharedPtr pass_req_service_;
        crane_msgs::msg::PassInfo pass_info_;
    };

}  // namespace crane
#endif  // CRANE_RECEIVE_PLANNER__RECEIVE_PLANNER_HPP_
