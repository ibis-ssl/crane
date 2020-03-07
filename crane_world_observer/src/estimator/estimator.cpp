// Copyright (c) 2020 ibis-ssl
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

#include <iostream>
#include <vector>
#include "estimator.hpp"

// PoseKalmanFilterクラス
// 二次元座標での位置と速度のカルマンフィルタ

PoseKalmanFilter::PoseKalmanFilter()
: KIDNAPPED_TIME_THRESH_(1.0),
  latest_inlier_stamp_(rclcpp::Time(0))
{
}

void PoseKalmanFilter::Init(double loop_time)
{
  this->dt = loop_time;

  this->InitSystemModel(&(this->sys_pdf), &(this->sys_model));
  this->InitMeasurementModel(&(this->meas_pdf), &(this->meas_model));
  this->InitPrior(&(this->prior));

  /****************************
   * Linear prior DENSITY     *
   ***************************/
  // Continuous Gaussian prior (for Kalman filters)
  this->filter = new BFL::ExtendedKalmanFilter(this->prior);

  BFL::Pdf<MatrixWrapper::ColumnVector> * posterior = filter->PostGet();
  this->last_estimation.val = posterior->ExpectedValueGet();
  this->last_estimation.cov = posterior->CovarianceGet();
}

geometry2d::Odometry PoseKalmanFilter::estimate()
{
  std::vector<geometry2d::Pose> null_poses;
  return this->estimate(null_poses);
}

geometry2d::Odometry PoseKalmanFilter::estimate(std::vector<geometry2d::Pose> observations)
{
  geometry2d::Accel null_acc;
  return this->estimate(null_acc, observations);
}

geometry2d::Odometry PoseKalmanFilter::estimate(
  geometry2d::Accel accel,
  std::vector<geometry2d::Pose> observations)
{
  // System update by only system model with input
  predict(accel.toColumnVector());

  bool is_observation_coming = observations.size() > 0;

  for (auto observation : observations) {
    MatrixWrapper::ColumnVector observation_cv = observation.toColumnVector();

    // collect angle continuity
    observation_cv(3) = EulerAngle::normalize(observation_cv(3), this->last_estimation.val(3));

    if (isOutlier(observation_cv)) {
      continue;
    }
    this->latest_inlier_stamp_ = rclcpp::Clock(RCL_ROS_TIME).now();

    update(observation_cv);
  }

  if (is_observation_coming) {
    // 観測が全く来ていない（ロボットが存在しないときなど）は誘拐判定しないように

    if ((rclcpp::Clock(RCL_ROS_TIME).now() - this->latest_inlier_stamp_) >
      this->KIDNAPPED_TIME_THRESH_)
    {
      // 誘拐状態
      this->Reset();
    }
  }

  return convetEstimationToOdometry();
}

void PoseKalmanFilter::Reset()
{
  MatrixWrapper::ColumnVector prior_Mu(6);
  prior_Mu = 0.0;

  MatrixWrapper::SymmetricMatrix prior_Cov(6);
  prior_Cov = 0.0;
  prior_Cov(1, 1) = 100.0;
  prior_Cov(2, 2) = 100.0;
  prior_Cov(3, 3) = 100.0;
  prior_Cov(4, 4) = 100.0;
  prior_Cov(5, 5) = 100.0;
  prior_Cov(6, 6) = 100.0;

  this->prior->ExpectedValueSet(prior_Mu);
  this->prior->CovarianceSet(prior_Cov);

  this->filter->Reset(this->prior);
}


// Private methods
//

void PoseKalmanFilter::collectAngleOverflow(
  MatrixWrapper::ColumnVector & state, MatrixWrapper::SymmetricMatrix & cov)
{
  if (state(3) < -M_PI || state(3) > M_PI) {
    state(3) = EulerAngle::normalize(state(3));

    this->prior->ExpectedValueSet(state);
    this->prior->CovarianceSet(cov);

    filter->Reset(prior);
  }
}

PoseKalmanFilter::Estimation PoseKalmanFilter::getResult()
{
  Estimation est;
  BFL::Pdf<MatrixWrapper::ColumnVector> * posterior = filter->PostGet();

  est.val = posterior->ExpectedValueGet();
  est.cov = posterior->CovarianceGet();

  return est;
}


void PoseKalmanFilter::predict(MatrixWrapper::ColumnVector input)
{
  Estimation est;

  this->filter->Update(sys_model, input);
  est = getResult();
  collectAngleOverflow(est.val, est.cov);

  this->last_estimation = est;
}


void PoseKalmanFilter::update(MatrixWrapper::ColumnVector measurement)
{
  Estimation est;

  this->filter->Update(meas_model, measurement);
  est = getResult();

  collectAngleOverflow(est.val, est.cov);

  this->last_estimation = est;
}


geometry2d::Odometry PoseKalmanFilter::convetEstimationToOdometry()
{
  geometry2d::Odometry odom;

  odom.pose.x = this->last_estimation.val(1);
  odom.pose.y = this->last_estimation.val(2);
  odom.pose.theta = this->last_estimation.val(3);

  odom.velocity.x = this->last_estimation.val(4);
  odom.velocity.y = this->last_estimation.val(5);
  odom.velocity.theta = this->last_estimation.val(6);

  return odom;
}


bool PoseKalmanFilter::isOutlier(MatrixWrapper::ColumnVector measurement)
{
  //  Reference: https://myenigma.hatenablog.com/entry/20140825/1408975706

  double mahala_dist = mahalanobisDistance(measurement);
  double thresh = 5.99;   //  自由度2、棄却率5%のしきい値

  if (mahala_dist > thresh) {
    return true;
  }
  return false;
}


double PoseKalmanFilter::mahalanobisDistance(MatrixWrapper::ColumnVector measurement)
{
  double measurementX = measurement(1);
  double measurementY = measurement(2);
  double estimationX = this->last_estimation.val(1);
  double estimationY = this->last_estimation.val(2);
  double covarianceX = this->last_estimation.cov(1, 1);
  double covarianceY = this->last_estimation.cov(2, 2);

  double diffX = measurementX - estimationX;
  double diffY = measurementY - estimationY;

  // avoid 0 division
  if (covarianceX == 0 || covarianceY == 0) {
    return 0;
  }

  return sqrt(pow(diffX, 2) / covarianceX + pow(diffY, 2) / covarianceY);
}


PoseKalmanFilter::~PoseKalmanFilter()
{
  delete this->sys_pdf;
  delete this->sys_model;
  delete this->meas_pdf;
  delete this->meas_model;
  delete this->prior;
  delete this->filter;
}

// EnemyEstimatorクラス
// ボールの位置・速度推定を担当
void EnemyEstimator::InitSystemModel(
  BFL::LinearAnalyticConditionalGaussian ** sys_pdf,
  BFL::LinearAnalyticSystemModelGaussianUncertainty ** sys_model)
{
  // Create the matrices A and B for the linear system model
  MatrixWrapper::Matrix A(6, 6);
  A(1, 1) = 1.0;  A(1, 2) = 0.0;  A(1, 3) = 0.0;  A(1, 4) = dt;   A(1, 5) = 0.0;  A(1, 6) = 0.0;
  A(2, 1) = 0.0;  A(2, 2) = 1.0;  A(2, 3) = 0.0;  A(2, 4) = 0.0;  A(2, 5) = dt;   A(2, 6) = 0.0;
  A(3, 1) = 0.0;  A(3, 2) = 0.0;  A(3, 3) = 1.0;  A(3, 4) = 0.0;  A(3, 5) = 0.0;  A(3, 6) = dt;
  A(4, 1) = 0.0;  A(4, 2) = 0.0;  A(4, 3) = 0.0;  A(4, 4) = 1.0;  A(4, 5) = 0.0;  A(4, 6) = 0.0;
  A(5, 1) = 0.0;  A(5, 2) = 0.0;  A(5, 3) = 0.0;  A(5, 4) = 0.0;  A(5, 5) = 1.0;  A(5, 6) = 0.0;
  A(6, 1) = 0.0;  A(6, 2) = 0.0;  A(6, 3) = 0.0;  A(6, 4) = 0.0;  A(6, 5) = 0.0;  A(6, 6) = 1.0;

  MatrixWrapper::Matrix B(6, 3);
  B(1, 1) = 0.0;  B(1, 2) = 0.0;  B(1, 3) = 0.0;
  B(2, 1) = 0.0;  B(2, 2) = 0.0;  B(2, 3) = 0.0;
  B(3, 1) = 0.0;  B(3, 2) = 0.0;  B(3, 3) = 0.0;
  B(4, 1) = dt;   B(4, 2) = 0.0;  B(4, 3) = 0.0;
  B(5, 1) = 0.0;  B(5, 2) = dt;   B(5, 3) = 0.0;
  B(6, 1) = 0.0;  B(6, 2) = 0.0;  B(6, 3) = dt;

  std::vector<MatrixWrapper::Matrix> AB(2);
  AB[0] = A;
  AB[1] = B;

  // create gaussian
  MatrixWrapper::ColumnVector sysNoise_Mu(6);
  sysNoise_Mu = 0.0;

  // 位置、速度変化はノイズとして表現
  const double MAX_LINEAR_ACC_MPS = 5.0;
  const double MAX_ANGULAR_ACC_RADPS = 2 * M_PI;

  const double MAX_LINEAR_MOVEMENT_IN_DT = MAX_LINEAR_ACC_MPS / 2 * pow(dt, 2);
  const double MAX_ANGULAR_MOVEMENT_IN_DT = MAX_ANGULAR_ACC_RADPS / 2 * pow(dt, 2);
  const double MAX_LINEAR_ACCEL_IN_DT = MAX_LINEAR_ACC_MPS * dt;
  const double MAX_ANGULAR_ACCEL_IN_DT = MAX_ANGULAR_ACC_RADPS * dt;

  MatrixWrapper::SymmetricMatrix sysNoise_Cov(6);
  sysNoise_Cov = 0.0;
  sysNoise_Cov(1, 1) = pow(MAX_LINEAR_MOVEMENT_IN_DT, 2);
  sysNoise_Cov(2, 2) = pow(MAX_LINEAR_MOVEMENT_IN_DT, 2);
  sysNoise_Cov(3, 3) = pow(MAX_ANGULAR_MOVEMENT_IN_DT, 2);
  sysNoise_Cov(4, 4) = pow(MAX_LINEAR_ACCEL_IN_DT, 2);
  sysNoise_Cov(5, 5) = pow(MAX_LINEAR_ACCEL_IN_DT, 2);
  sysNoise_Cov(6, 6) = pow(MAX_ANGULAR_ACCEL_IN_DT, 2);

  BFL::Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);

  // create the model
  *sys_pdf = new BFL::LinearAnalyticConditionalGaussian(AB, system_Uncertainty);
  *sys_model = new BFL::LinearAnalyticSystemModelGaussianUncertainty(*sys_pdf);
}

void EnemyEstimator::InitMeasurementModel(
  BFL::LinearAnalyticConditionalGaussian ** meas_pdf,
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty ** meas_model)
{
  // create matrix H for linear measurement model
  MatrixWrapper::Matrix H(3, 6);
  H = 0.0;
  H(1, 1) = 1.0;
  H(2, 2) = 1.0;
  H(3, 3) = 1.0;

  // Construct the measurement noise
  MatrixWrapper::ColumnVector measNoise_Mu(3);
  measNoise_Mu = 0.0;

  MatrixWrapper::SymmetricMatrix measNoise_Cov(3);
  measNoise_Cov(1, 1) = pow(0.02, 2);    // 観測ノイズを標準偏差 0.02[m] と仮定
  measNoise_Cov(2, 2) = pow(0.02, 2);
  measNoise_Cov(3, 3) = pow(0.02, 2);
  BFL::Gaussian measurement_Uncertainty(measNoise_Mu, measNoise_Cov);

  // create the model
  *meas_pdf = new BFL::LinearAnalyticConditionalGaussian(H, measurement_Uncertainty);
  *meas_model = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(*meas_pdf);
}

void EnemyEstimator::InitPrior(BFL::Gaussian ** prior)
{
  MatrixWrapper::ColumnVector prior_Mu(6);
  prior_Mu = 0.0;

  MatrixWrapper::SymmetricMatrix prior_Cov(6);
  prior_Cov = 0.0;
  prior_Cov(1, 1) = 100.0;
  prior_Cov(2, 2) = 100.0;
  prior_Cov(3, 3) = 100.0;
  prior_Cov(4, 4) = 100.0;
  prior_Cov(5, 5) = 100.0;
  prior_Cov(6, 6) = 100.0;

  *prior = new BFL::Gaussian(prior_Mu, prior_Cov);
}

geometry2d::Odometry EnemyEstimator::estimateWithConsideringOtherRobots(
  std::vector<geometry2d::Odometry> other_robots)
{
  return this->estimate();
}

geometry2d::Odometry EnemyEstimator::estimateWithConsideringOtherRobots(
  std::vector<geometry2d::Pose> observations, std::vector<geometry2d::Odometry> other_robots)
{
  return this->estimate(observations);
}

geometry2d::Odometry EnemyEstimator::estimateWithConsideringOtherRobots(
  geometry2d::Accel accel,
  std::vector<geometry2d::Pose> observations,
  std::vector<geometry2d::Odometry> other_robots)
{
  return this->estimate(accel, observations);
}


// BallEstimatorクラス
// ボールの位置・速度推定を担当
void BallEstimator::InitSystemModel(
  BFL::LinearAnalyticConditionalGaussian ** sys_pdf,
  BFL::LinearAnalyticSystemModelGaussianUncertainty ** sys_model)
{
  // Create the matrices A and B for the linear system model
  MatrixWrapper::Matrix A(6, 6);
  A(1, 1) = 1.0;  A(1, 2) = 0.0;  A(1, 3) = 0.0;  A(1, 4) = dt;   A(1, 5) = 0.0;  A(1, 6) = 0.0;
  A(2, 1) = 0.0;  A(2, 2) = 1.0;  A(2, 3) = 0.0;  A(2, 4) = 0.0;  A(2, 5) = dt;   A(2, 6) = 0.0;
  A(3, 1) = 0.0;  A(3, 2) = 0.0;  A(3, 3) = 1.0;  A(3, 4) = 0.0;  A(3, 5) = 0.0;  A(3, 6) = dt;
  A(4, 1) = 0.0;  A(4, 2) = 0.0;  A(4, 3) = 0.0;  A(4, 4) = 1.0;  A(4, 5) = 0.0;  A(4, 6) = 0.0;
  A(5, 1) = 0.0;  A(5, 2) = 0.0;  A(5, 3) = 0.0;  A(5, 4) = 0.0;  A(5, 5) = 1.0;  A(5, 6) = 0.0;
  A(6, 1) = 0.0;  A(6, 2) = 0.0;  A(6, 3) = 0.0;  A(6, 4) = 0.0;  A(6, 5) = 0.0;  A(6, 6) = 1.0;

  MatrixWrapper::Matrix B(6, 3);
  B(1, 1) = 0.0;  B(1, 2) = 0.0;  B(1, 3) = 0.0;
  B(2, 1) = 0.0;  B(2, 2) = 0.0;  B(2, 3) = 0.0;
  B(3, 1) = 0.0;  B(3, 2) = 0.0;  B(3, 3) = 0.0;
  B(4, 1) = dt;   B(4, 2) = 0.0;  B(4, 3) = 0.0;
  B(5, 1) = 0.0;  B(5, 2) = dt;   B(5, 3) = 0.0;
  B(6, 1) = 0.0;  B(6, 2) = 0.0;  B(6, 3) = dt;

  std::vector<MatrixWrapper::Matrix> AB(2);
  AB[0] = A;
  AB[1] = B;

  // create gaussian
  MatrixWrapper::ColumnVector sysNoise_Mu(6);
  sysNoise_Mu = 0.0;

  // 位置、速度変化はノイズとして表現
  const double MAX_LINEAR_ACC_MPS = 100.0;      // 6.5[m/s] / 16[ms] = 500

  const double MAX_LINEAR_MOVEMENT_IN_DT = MAX_LINEAR_ACC_MPS / 2 * pow(dt, 2);
  const double MAX_LINEAR_ACCEL_IN_DT = MAX_LINEAR_ACC_MPS * dt;

  MatrixWrapper::SymmetricMatrix sysNoise_Cov(6);
  sysNoise_Cov = 0.0;
  sysNoise_Cov(1, 1) = pow(MAX_LINEAR_MOVEMENT_IN_DT, 2);
  sysNoise_Cov(2, 2) = pow(MAX_LINEAR_MOVEMENT_IN_DT, 2);
  sysNoise_Cov(3, 3) = 1e9;
  sysNoise_Cov(4, 4) = pow(MAX_LINEAR_ACCEL_IN_DT, 2);
  sysNoise_Cov(5, 5) = pow(MAX_LINEAR_ACCEL_IN_DT, 2);
  sysNoise_Cov(6, 6) = 1e9;

  BFL::Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);

  // create the model
  *sys_pdf = new BFL::LinearAnalyticConditionalGaussian(AB, system_Uncertainty);
  *sys_model = new BFL::LinearAnalyticSystemModelGaussianUncertainty(*sys_pdf);
}

void BallEstimator::InitMeasurementModel(
  BFL::LinearAnalyticConditionalGaussian ** meas_pdf,
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty ** meas_model)
{
  // create matrix H for linear measurement model
  MatrixWrapper::Matrix H(3, 6);
  H = 0.0;
  H(1, 1) = 1.0;
  H(2, 2) = 1.0;
  H(3, 3) = 1.0;

  // Construct the measurement noise
  MatrixWrapper::ColumnVector measNoise_Mu(3);
  measNoise_Mu = 0.0;

  MatrixWrapper::SymmetricMatrix measNoise_Cov(3);
  measNoise_Cov(1, 1) = pow(0.05, 2);    // 観測ノイズを標準偏差 0.02[m] と仮定
  measNoise_Cov(2, 2) = pow(0.05, 2);
  measNoise_Cov(3, 3) = 1e9;
  BFL::Gaussian measurement_Uncertainty(measNoise_Mu, measNoise_Cov);

  // create the model
  *meas_pdf = new BFL::LinearAnalyticConditionalGaussian(H, measurement_Uncertainty);
  *meas_model = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(*meas_pdf);
}

void BallEstimator::InitPrior(BFL::Gaussian ** prior)
{
  MatrixWrapper::ColumnVector prior_Mu(6);
  prior_Mu = 0.0;

  MatrixWrapper::SymmetricMatrix prior_Cov(6);
  prior_Cov = 0.0;
  prior_Cov(1, 1) = 100.0;
  prior_Cov(2, 2) = 100.0;
  prior_Cov(3, 3) = 100.0;
  prior_Cov(4, 4) = 100.0;
  prior_Cov(5, 5) = 100.0;
  prior_Cov(6, 6) = 100.0;

  *prior = new BFL::Gaussian(prior_Mu, prior_Cov);
}

geometry2d::Odometry BallEstimator::estimateWithConsideringOtherRobots(
  std::vector<geometry2d::Odometry> other_robots)
{
  return this->estimate();
}

geometry2d::Odometry BallEstimator::estimateWithConsideringOtherRobots(
  std::vector<geometry2d::Pose> observations, std::vector<geometry2d::Odometry> other_robots)
{
  geometry2d::Odometry latest_ball_odom = this->convetEstimationToOdometry();
  bool will_reflection_occur = this->ball_reflection_detector_.WillReflectionOccur(latest_ball_odom,
      other_robots);

  // TODO(HansRobo): implement
  if (will_reflection_occur) {
    this->SetSytemNoiseForReflecting();
  } else {
    this->SetSystemNoiseToRolling();
  }

  return this->estimate(observations);
}

geometry2d::Odometry BallEstimator::estimateWithConsideringOtherRobots(
  geometry2d::Accel accel,
  std::vector<geometry2d::Pose> observations,
  std::vector<geometry2d::Odometry> other_robots)
{
  return this->estimate(accel, observations);
}


void BallEstimator::SetSytemNoiseForReflecting()
{
  // 位置、速度変化はノイズとして表現
  const double MAX_LINEAR_ACC_MPS = 500.0;      // 6.5[m/s] / 16[ms] = 500

  const double MAX_LINEAR_MOVEMENT_IN_DT = MAX_LINEAR_ACC_MPS / 2 * pow(dt, 2);
  const double MAX_LINEAR_ACCEL_IN_DT = MAX_LINEAR_ACC_MPS * dt;

  MatrixWrapper::SymmetricMatrix sysNoise_Cov(6);
  sysNoise_Cov = 0.0;
  sysNoise_Cov(1, 1) = pow(MAX_LINEAR_MOVEMENT_IN_DT, 2);
  sysNoise_Cov(2, 2) = pow(MAX_LINEAR_MOVEMENT_IN_DT, 2);
  sysNoise_Cov(3, 3) = 1e9;
  sysNoise_Cov(4, 4) = pow(MAX_LINEAR_ACCEL_IN_DT, 2);
  sysNoise_Cov(5, 5) = pow(MAX_LINEAR_ACCEL_IN_DT, 2);
  sysNoise_Cov(6, 6) = 1e9;

  sys_pdf->AdditiveNoiseSigmaSet(sysNoise_Cov);
}

void BallEstimator::SetSystemNoiseToRolling()
{
  // 位置、速度変化はノイズとして表現
  const double MAX_LINEAR_ACC_MPS = 10.0;

  const double MAX_LINEAR_MOVEMENT_IN_DT = MAX_LINEAR_ACC_MPS / 2 * pow(dt, 2);
  const double MAX_LINEAR_ACCEL_IN_DT = MAX_LINEAR_ACC_MPS * dt;

  MatrixWrapper::SymmetricMatrix sysNoise_Cov(6);
  sysNoise_Cov = 0.0;
  sysNoise_Cov(1, 1) = pow(MAX_LINEAR_MOVEMENT_IN_DT, 2);
  sysNoise_Cov(2, 2) = pow(MAX_LINEAR_MOVEMENT_IN_DT, 2);
  sysNoise_Cov(3, 3) = 1e9;
  sysNoise_Cov(4, 4) = pow(MAX_LINEAR_ACCEL_IN_DT, 2);
  sysNoise_Cov(5, 5) = pow(MAX_LINEAR_ACCEL_IN_DT, 2);
  sysNoise_Cov(6, 6) = 1e9;

  sys_pdf->AdditiveNoiseSigmaSet(sysNoise_Cov);
}

// BallReflectionDetector クラス
// ボールがキックまたは跳ね返ることを検出するクラス
// 急激な速度変化がある際はカルマンフィルタのシステムノイズを大きく設定し、フィルタの追従性を向上させるため、そのサポート役のクラス
bool BallEstimator::BallReflectionDetector::WillReflectionOccur(
  geometry2d::Odometry odom_ball,
  std::vector<geometry2d::Odometry> odom_robots)
{
  // ロボットの正面にいる場合、キックされる可能性があるのでTrue
  for (auto odom_robot : odom_robots) {
    geometry2d::Pose robot_pose = odom_robot.pose;
    geometry2d::Pose ball_pose = odom_ball.pose;


    if (this->IsBallInFrontOfRobot(robot_pose, ball_pose)) {
      return true;
    }

    if (this->WillBallContactToRobotSoon(odom_robot, odom_ball)) {
      return true;
    }
  }

  return false;
}

bool BallEstimator::BallReflectionDetector::IsBallInFrontOfRobot(
  geometry2d::Pose pose_robot,
  geometry2d::Pose pose_ball)
{
  const double ROBOT_FRONT_ANGLE_RANGE = M_PI;      // ロボット正面と判断する範囲
  const double ROBOT_FRONT_DIST = 0.2;              // ロボット正面と判断する距離
  geometry2d::Pose robot_to_ball = pose_robot.transpose(pose_ball);
  double robot_ball_distance = robot_to_ball.getNorm();
  double robot_ball_angle = robot_to_ball.getAngle();

  if (-(ROBOT_FRONT_ANGLE_RANGE / 2) < robot_ball_angle &&
    robot_ball_angle < (ROBOT_FRONT_ANGLE_RANGE / 2))
  {
    if (robot_ball_distance < ROBOT_FRONT_DIST) {
      return true;
    }
  }

  return false;
}

bool BallEstimator::BallReflectionDetector::WillBallContactToRobotSoon(
  geometry2d::Odometry odom_robot, geometry2d::Odometry odom_ball)
{
  const double ROBOT_RADIUS_METER = 0.09;
  const double MARGIN = 0.05;
  const double CONTACT_TIME_THRESH_SEC = 0.5;

  double velocity_angle = odom_ball.velocity.getAngle();
  geometry2d::Pose ball_pose_on_velocity(odom_ball.pose.x, odom_ball.pose.y, velocity_angle);
  geometry2d::Pose ball_to_robot =
      ball_pose_on_velocity.transpose(odom_robot.pose);

  double ball_velocity_norm = odom_ball.velocity.getNorm();
  if (ball_velocity_norm < 1e-9) {
    // ボールが止まっている
    return false;
  }

  if (ball_to_robot.x < 0) {
    // ボールの進行方向の裏にいる
    return false;
  }

  if (ball_to_robot.y > (ROBOT_RADIUS_METER + MARGIN)) {
    // ボール軌道から上側にそれた場所にいる
    return false;
  }

  if (ball_to_robot.y < -(ROBOT_RADIUS_METER + MARGIN)) {
    // ボール軌道から下側にそれた場所にいる
    return false;
  }

  if ((ball_to_robot.x / ball_velocity_norm) > CONTACT_TIME_THRESH_SEC) {
    // CONTACT_TIME_THRESH_SEC[s] 以内には衝突しない
    return false;
  }

  return true;
}

/**********************************************************
 * This is implementation of EulerAngle class
 ***********************************************************/
double EulerAngle::normalize(double angle)
{
  while (angle >= M_PI) {angle -= 2.0 * M_PI;}
  while (angle <= -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}


double EulerAngle::normalize(double angle, double center)
{
  return center + normalize(angle - center);
}
