// Copyright 2021 Roots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONSAI_VISION_TRACKER__BALL_TRACKER_HPP_
#define CONSAI_VISION_TRACKER__BALL_TRACKER_HPP_

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

#include <memory>
#include <robocup_ssl_msgs/msg/detection_ball.hpp>
#include <robocup_ssl_msgs/msg/tracked_ball.hpp>
#include <vector>

namespace consai_vision_tracker
{
using DetectionBall = robocup_ssl_msgs::msg::DetectionBall;

using TrackedBall = robocup_ssl_msgs::msg::TrackedBall;

using ConditionalGaussian = BFL::LinearAnalyticConditionalGaussian;

using SystemModelGaussianUncertainty = BFL::LinearAnalyticSystemModelGaussianUncertainty;

using MeasurementModelGaussianUncertainty = BFL::LinearAnalyticMeasurementModelGaussianUncertainty;

using Gaussian = BFL::Gaussian;

using ExtendedKalmanFilter = BFL::ExtendedKalmanFilter;

class BallTracker
{
public:
  explicit BallTracker(const double dt = 0.01);

  void push_back_observation(const DetectionBall & ball);

  TrackedBall update();

private:
  void reset_prior();

  bool is_outlier(const TrackedBall & observation) const;

  std::vector<TrackedBall> ball_observations;

  TrackedBall prev_tracked_ball;

  std::shared_ptr<ConditionalGaussian> sys_pdf;

  std::shared_ptr<SystemModelGaussianUncertainty> sys_model;

  std::shared_ptr<ConditionalGaussian> meas_pdf;

  std::shared_ptr<MeasurementModelGaussianUncertainty> meas_model;

  std::shared_ptr<Gaussian> prior;

  std::shared_ptr<ExtendedKalmanFilter> filter;
};

}  // namespace consai_vision_tracker

#endif  // CONSAI_VISION_TRACKER__BALL_TRACKER_HPP_
