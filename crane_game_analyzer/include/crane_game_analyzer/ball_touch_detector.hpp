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

#ifndef CRANE_GAME_ANALYZER__BALL_TOUCH_DETECTOR_HPP_
#define CRANE_GAME_ANALYZER__BALL_TOUCH_DETECTOR_HPP_

#include "crane_msg_wrappers/world_model_wrapper.hpp"

struct BallTouchInfo{
    bool touch_event = false;
    RobotIdentifier robot_id;
    double distance;
};

class BallTouchDetector{
public:
    BallTouchDetector(){}
    BallTouchInfo update(WorldModelWrapper & world_model){
        BallTouchInfo info;
        auto & ours = world_model.ours.robots;
        auto & theirs = world_model.theirs.robots;
        auto & ball_pos = world_model.ball.pos;
        auto get_nearest_ball_robot = [&](std::vector<RobotInfo::SharedPtr> & robots){
            return *std::min_element(robots.begin(), robots.end(),[ball_pos](auto & a, auto & b){
                return (a->pose.pos - ball_pos).squaredNorm() < (b->pose.pos - ball_pos).squaredNorm();
            });
        };
        auto nearest_ours = get_nearest_ball_robot(ours);
        auto nearest_theirs = get_nearest_ball_robot(theirs);

        double ours_distance = (nearest_ours->pose.pos - ball_pos).norm();
        double theirs_distance = (nearest_theirs->pose.pos - ball_pos).norm();

        if(ours_distance < theirs_distance){
            info.robot_id.is_ours = true;
            info.robot_id.robot_id = nearest_ours->id;
            info.distance = ours_distance;
        }else{
            info.robot_id.is_ours = false;
            info.robot_id.robot_id = nearest_theirs->id;
            info.distance = theirs_distance;
        }
        info.touch_event = (info.distance < touch_threshold_meter_);
        return info;
    }

private:
    double touch_threshold_meter_ = 0.05;
};
#endif  // CRANE_GAME_ANALYZER__BALL_TOUCH_DETECTOR_HPP_
