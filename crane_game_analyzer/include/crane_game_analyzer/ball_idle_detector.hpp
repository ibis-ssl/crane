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

#ifndef CRANE_GAME_ANALYZER__BALL_IDLE_DETECTOR_HPP_
#define CRANE_GAME_ANALYZER__BALL_IDLE_DETECTOR_HPP_

#include "crane_msg_wrappers/world_model_wrapper.hpp"

struct BallPositionStamped{
    Point position;
    rclcpp::Time stamp;
};
class BallIdleDetector{
public:
    BallIdleDetector() : threshold_duration(5, 0), ros_clock_(RCL_ROS_TIME) {}
    bool update(const WorldModelWrapper & world_model){
        BallPositionStamped record;
        record.position = world_model.ball.pos;
        record.stamp = ros_clock_.now();
        ball_trajectory_.push_front(record);

        return judgeBallIdle();
    }

    bool judgeBallIdle(){
        auto latest_time = ball_trajectory_.front().stamp;
        auto latest_position = ball_trajectory_.front().position;
        //delete records over threshold
        ball_trajectory_.erase(std::remove_if(ball_trajectory_.begin(), ball_trajectory_.end(),[&](auto & record){
            return (latest_time - record.stamp) > threshold_duration * 2;
        }),ball_trajectory_.end());

        // earlier first , older next
        // check boll idling
        for(auto record : ball_trajectory_){
            if((latest_position - record.position).norm() < move_distance_threshold_meter){
                if((latest_time - record.stamp) < threshold_duration){
                    return false;
                }
            }
        }
        return true;
    }
private:
    rclcpp::Duration threshold_duration;
    double move_distance_threshold_meter = 0.05;
    rclcpp::Clock ros_clock_;
    std::deque<BallPositionStamped> ball_trajectory_;
};
#endif  // CRANE_GAME_ANALYZER__BALL_IDLE_DETECTOR_HPP_
