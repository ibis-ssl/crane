#ifndef ESTIMATOR_HPP_
#define ESTIMATOR_HPP_

#include  <rclcpp/rclcpp.hpp>

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

#include "geometry.hpp"

class PoseKalmanFilter
{
public:
  PoseKalmanFilter();
  ~PoseKalmanFilter();

  void Init(double loop_time);

  geometry2d::Odometry estimate();
  geometry2d::Odometry estimate(std::vector<geometry2d::Pose> observations);
  geometry2d::Odometry estimate(
  geometry2d::Accel accel,
  std::vector<geometry2d::Pose> observations);

  virtual geometry2d::Odometry estimateWithConsideringOtherRobots(
    std::vector<geometry2d::Odometry> other_robots) = 0;
  virtual geometry2d::Odometry estimateWithConsideringOtherRobots(
    std::vector<geometry2d::Pose> observations, std::vector<geometry2d::Odometry> other_robots) = 0;
  virtual geometry2d::Odometry estimateWithConsideringOtherRobots(
    geometry2d::Accel accel,
    std::vector<geometry2d::Pose> observations,
    std::vector<geometry2d::Odometry> other_robots) = 0;

  void Reset();

protected:
  double dt;
  BFL::LinearAnalyticConditionalGaussian * sys_pdf;
  BFL::LinearAnalyticSystemModelGaussianUncertainty * sys_model;
  BFL::LinearAnalyticConditionalGaussian * meas_pdf;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty * meas_model;
  BFL::KalmanFilter * filter;
  BFL::Gaussian * prior;

  virtual void InitSystemModel(
      BFL::LinearAnalyticConditionalGaussian ** sys_pdf,
      BFL::LinearAnalyticSystemModelGaussianUncertainty ** sys_model) = 0;
  virtual void InitMeasurementModel(
      BFL::LinearAnalyticConditionalGaussian ** meas_pdf,
      BFL::LinearAnalyticMeasurementModelGaussianUncertainty ** meas_model) = 0;
  virtual void InitPrior(BFL::Gaussian ** prior) = 0;
  geometry2d::Odometry convetEstimationToOdometry();

private:
  class Estimation
  {
public:
    MatrixWrapper::ColumnVector val;
    MatrixWrapper::SymmetricMatrix cov;
  };

  rclcpp::Duration KIDNAPPED_TIME_THRESH_;
  Estimation last_estimation;
  rclcpp::Time latest_inlier_stamp_;

  void  predict(MatrixWrapper::ColumnVector input);
  void  update(MatrixWrapper::ColumnVector measurement);

  Estimation getResult();
  void collectAngleOverflow(MatrixWrapper::ColumnVector & state, MatrixWrapper::SymmetricMatrix & cov);

  bool isOutlier(MatrixWrapper::ColumnVector measurement);
  double mahalanobisDistance(MatrixWrapper::ColumnVector measurement);

};


class EnemyEstimator : public PoseKalmanFilter
{
public:
  geometry2d::Odometry estimateWithConsideringOtherRobots(
    std::vector<geometry2d::Odometry> other_robots);
  geometry2d::Odometry estimateWithConsideringOtherRobots(
    std::vector<geometry2d::Pose> observations, std::vector<geometry2d::Odometry> other_robots);
  geometry2d::Odometry estimateWithConsideringOtherRobots(
    geometry2d::Accel accel,
    std::vector<geometry2d::Pose> observations, std::vector<geometry2d::Odometry> other_robots);

protected:
  void InitSystemModel(
    BFL::LinearAnalyticConditionalGaussian ** sys_pdf,
    BFL::LinearAnalyticSystemModelGaussianUncertainty ** sys_model);
  void InitMeasurementModel(
    BFL::LinearAnalyticConditionalGaussian ** meas_pdf,
    BFL::LinearAnalyticMeasurementModelGaussianUncertainty ** meas_model);
  void InitPrior(BFL::Gaussian ** prior);
};


class BallEstimator : public PoseKalmanFilter
{
public:
  geometry2d::Odometry estimateWithConsideringOtherRobots(
    std::vector<geometry2d::Odometry> other_robots);
  geometry2d::Odometry estimateWithConsideringOtherRobots(
    std::vector<geometry2d::Pose> observations, std::vector<geometry2d::Odometry> other_robots);
  geometry2d::Odometry estimateWithConsideringOtherRobots(
    geometry2d::Accel accel,
    std::vector<geometry2d::Pose> observations, std::vector<geometry2d::Odometry> other_robots);

protected:
  void InitSystemModel(
    BFL::LinearAnalyticConditionalGaussian ** sys_pdf,
    BFL::LinearAnalyticSystemModelGaussianUncertainty ** sys_model);
  void InitMeasurementModel(
    BFL::LinearAnalyticConditionalGaussian ** meas_pdf,
    BFL::LinearAnalyticMeasurementModelGaussianUncertainty ** meas_model);
  void InitPrior(BFL::Gaussian ** prior);

private:
  // BallReflectionDetector クラス
  // ボールがキックまたは跳ね返ることを検出するクラス
  // 急激な速度変化がある際はカルマンフィルタのシステムノイズを大きく設定し、フィルタの追従性を向上させるため、そのサポート役のクラス
  class BallReflectionDetector
  {
public:
    bool WillReflectionOccur(
      geometry2d::Odometry odom_ball,
      std::vector<geometry2d::Odometry> odom_robots);

private:
    bool IsBallInFrontOfRobot(geometry2d::Pose pose_robot, geometry2d::Pose pose_ball);
    bool WillBallContactToRobotSoon(
      geometry2d::Odometry odom_robot,
      geometry2d::Odometry odom_ball);
  };

  BallReflectionDetector ball_reflection_detector_;

  void SetSytemNoiseForReflecting();
  void SetSystemNoiseToRolling();
};


class EulerAngle
{
public:
  static double  normalize(double angle);
  static double  normalize(double angle, double center);
};

#endif  // ESTIMATOR_HPP_
