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

class BallIdleDetector{
public:
    BallIdleDetector(){}
    void update(const WorldModelWrapper & world_model){
//        static auto last_move_time = world_model.
    }
private:
    double interval_s_ = 5.0;
    double move_distance_threshold_meter = 0.05;
};
#endif  // CRANE_GAME_ANALYZER__BALL_IDLE_DETECTOR_HPP_
