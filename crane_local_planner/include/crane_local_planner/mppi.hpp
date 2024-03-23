// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__MPPI_HPP_
#define CRANE_LOCAL_PLANNER__MPPI_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <random>
#include <utility>
#include <vector>

namespace crane
{
namespace models
{
template <int BATCH, int STEP>
struct State
{
  Pose2D pose;
  Pose2D velocity;
  Eigen::Matrix<float, BATCH, STEP> vx, vy, wz;     // 車両の速度
  Eigen::Matrix<float, BATCH, STEP> cvx, cvy, cwz;  // 制御速度
  void reset()
  {
    vx = Eigen::MatrixXf::Zero(BATCH, STEP);
    vy = Eigen::MatrixXf::Zero(BATCH, STEP);
    wz = Eigen::MatrixXf::Zero(BATCH, STEP);
    cvx = Eigen::MatrixXf::Zero(BATCH, STEP);
    cvy = Eigen::MatrixXf::Zero(BATCH, STEP);
    cwz = Eigen::MatrixXf::Zero(BATCH, STEP);
  }
};

struct Velocities
{
  Pose2D actual;
  Pose2D control;
};

struct State_
{
  Pose2D pose;
  Pose2D velocity;
  std::vector<Velocities> velocities;
};

struct ControlConstraints
{
  float vx_max;
  float vx_min;
  float vy;
  float wz;
};

struct SamplingStd
{
  float vx;
  float vy;
  float wz;
};

struct Path
{
  Eigen::VectorXf x;
  Eigen::VectorXf y;
  Eigen::VectorXf yaws;

  void reset(unsigned int size)
  {
    x = Eigen::VectorXf::Zero(size);
    y = Eigen::VectorXf::Zero(size);
    yaws = Eigen::VectorXf::Zero(size);
  }
};

struct Path_
{
  std::vector<Pose2D> poses;
  void reset(unsigned int size)
  {
    poses.clear();
    poses.resize(size);
  }
};

template <int BATCH, int STEP>
struct Trajectories
{
  Eigen::Matrix<float, BATCH, STEP> x, y, yaws;

  void reset()
  {
    x = Eigen::MatrixXf::Zero(BATCH, STEP);
    y = Eigen::MatrixXf::Zero(BATCH, STEP);
    yaws = Eigen::MatrixXf::Zero(BATCH, STEP);
  }
};

struct Trajectories_
{
  std::vector<std::vector<Pose2D>> trajectories;

  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    trajectories.clear();
    for (int i = 0; i < batch_size; i++) {
      trajectories.push_back(std::vector<Pose2D>(time_steps));
    }
  }
};

struct Control
{
  float vx, vy, wz;
};

template <int STEP>
struct ControlSequence
{
  Eigen::Vector<float, STEP> vx, vy, wz;

  void reset()
  {
    vx = Eigen::VectorXf::Zero(STEP);
    vy = Eigen::VectorXf::Zero(STEP);
    wz = Eigen::VectorXf::Zero(STEP);
  }
};
}  // namespace models

namespace critics
{
struct CriticData
{
};
class PathAlignCritic
{
public:
  void initialize()
  {
    power_ = 1;
    weight_ = 10.f;
    max_path_occupancy_ratio_ = 0.07f;
    offset_from_furthest_ = 20;
    trajectory_point_step_ = 4;
    threshold_to_consider_ = 0.5f;
    use_path_orientations_ = false;
  }

  void score(CriticData & data) {}

protected:
  size_t offset_from_furthest_{0};
  int trajectory_point_step_{0};
  float threshold_to_consider_{0};
  float max_path_occupancy_ratio_{0};
  bool use_path_orientations_{false};
  unsigned int power_{0};
  float weight_{0};
};
}  // namespace critics

template <int BATCH, int STEP>
class MotionModel
{
public:
  virtual void predict(models::State<BATCH, STEP> & state) {}
  virtual void applyConstraints(models::ControlSequence<STEP> & /*control_sequence*/) {}
};

template <int BATCH, int STEP>
class OmniMotionModel : public MotionModel<BATCH, STEP>
{
public:
  void predict(models::State<BATCH, STEP> & state) override
  {
    // Eigenにはxt::placeholdersの直接の対応はないため、全範囲または部分範囲の操作にはブロックを使用します。
    unsigned int numRows = state.vx.rows();      // バッチサイズ
    unsigned int numCols = state.vx.cols() - 1;  // タイムステップ数 - 1

    // vxとwzの更新
    state.vx.block(0, 1, numRows, numCols) = state.cvx.block(0, 0, numRows, numCols);
    state.vy.block(0, 1, numRows, numCols) = state.cvy.block(0, 0, numRows, numCols);
    state.wz.block(0, 1, numRows, numCols) = state.cwz.block(0, 0, numRows, numCols);
  }
};

struct OptimizerSettings
{
  const int ITERATIONS = 100;
  const double DT = 0.05;
  const int TIME_STEPS = 56;
  const int BATCH_SIZE = 1000;
  const double TEMPERATURE = 0.3;
  const double GAMMA = 0.015;
  const double V_MAX = 4.0;
  const double V_MIN = 0.0;
  const double W_MAX = 1.0;
  const double VX_STD = 0.2;
  const double VY_STD = 0.2;
  const double WZ_STD = 0.4;
};

template <int BATCH, int STEP>
struct NoiseGenerator
{
  std::mt19937 gen;
  std::normal_distribution<float> dist_vx;
  std::normal_distribution<float> dist_vy;
  std::normal_distribution<float> dist_wz;
  Eigen::Matrix<float, BATCH, STEP> noises_vx_, noises_vy_, noises_wz_;

  void initialize(OptimizerSettings & s)
  {
    gen = std::mt19937(std::random_device()());
    dist_vx = std::normal_distribution<float>(0.0f, s.VX_STD);
    dist_vy = std::normal_distribution<float>(0.0f, s.VY_STD);
    dist_wz = std::normal_distribution<float>(0.0f, s.WZ_STD);
  }

  void generateNoisedControls()
  {
    noises_vx_ = Eigen::MatrixXf(BATCH, STEP);
    noises_vy_ = Eigen::MatrixXf(BATCH, STEP);
    noises_wz_ = Eigen::MatrixXf(BATCH, STEP);
    for (int i = 0; i < BATCH; ++i) {
      for (int j = 0; j < STEP; ++j) {
        noises_vx_(i, j) = dist_vx(gen);
        noises_vy_(i, j) = dist_vy(gen);
        noises_wz_(i, j) = dist_wz(gen);
      }
    }
  }

  void setNoised(
    const models::ControlSequence<STEP> & control_sequence, models::State<BATCH, STEP> & state)
  {
    std::cout << "[getNoised] input sequence size: " << control_sequence.vx.rows() << " x 1"
              << std::endl;
    std::cout << "[getNoised] noises size: " << noises_vx_.rows() << " x " << noises_vx_.cols()
              << std::endl;
    //    models::ControlSequence noised_control_sequence;
    state.cvx = noises_vx_;
    state.cvy = noises_vy_;
    state.cwz = noises_wz_;

    for (int i = 0; i < state.cvx.rows(); i++) {
      //      std::cout << "[getNoised] state.cvx.row(i) size: " << state.cvx.row(i).rows() << " x "
      //                << state.cvx.row(i).cols() << std::endl;
      state.cvx.row(i) += control_sequence.vx.transpose();
      state.cvy.row(i) += control_sequence.vy.transpose();
      state.cwz.row(i) += control_sequence.wz.transpose();
    }
    //    state.cvx = noises_vx_.colwise() + control_sequence.vx.transpose();
    //
    //    state.cvx = control_sequence.vx.array() + noises_vx_.array();
    //    noised_control_sequence.vy = control_sequence.vy.array() + noises_vy_.array();
    //    noised_control_sequence.wz = control_sequence.wz.array() + noises_wz_.array();
    //    return noised_control_sequence;
  }
};

template <int BATCH, int STEP>
class Optimizer
{
private:
  OptimizerSettings settings;

  NoiseGenerator<BATCH, STEP> noise_generator;

  OmniMotionModel<BATCH, STEP> motion_model;

  models::ControlSequence<STEP> control_sequence;

public:
  Optimizer()
  {
    noise_generator.initialize(settings);
    control_sequence.reset();
  }

  void shiftControlSequence() {}

  // evalControl
  void calcCmd(std::vector<Point> path, Pose2D goal, Pose2D pose, Pose2D vel)
  {
    // prepare();
    models::Path path_;
    {
      path_.reset(path.size());
      for (int i = 0; i < path.size(); i++) {
        path_.x(i) = path[i].x();
        path_.y(i) = path[i].y();
        path_.yaws(i) = goal.theta;
      }
    }
    optimize(path_, goal);
    //    auto control = getControlFromSequenceAsTwist();
    //    return control;
  }

  //  void getControlFromSequenceAsTwist()
  //  {
  ////    unsigned int offset = settings_.shift_control_sequence ? 1 : 0;
  //    unsigned int offset = 0;
  //
  //    auto vx = control_sequence.vx(offset);
  //    auto vy = control_sequence.vy(offset);
  //    auto wz = control_sequence.wz(offset);
  //
  //    return utils::toTwistStamped(vx, vy, wz, stamp, costmap_ros_->getBaseFrameID());
  //  }

  void integrateStateVelocities(
    models::Trajectories<BATCH, STEP> & trajectories, const models::State<BATCH, STEP> & state)
  {
    const float initial_yaw = state.pose.theta;

    // wzに基づくyawの累積和を計算する
    decltype(state.wz) cumulative_wz;
    cumulative_wz.col(0).setConstant(initial_yaw);
    for (int i = 1; i < state.wz.cols(); ++i) {
      cumulative_wz.col(i) = cumulative_wz.col(i - 1) + state.wz.col(i - 1) * settings.DT;
    }
    trajectories.yaws = cumulative_wz;

    // yawのコサインとサインを計算する
    decltype(trajectories.yaws) yaw_cos, yaw_sin;
    yaw_cos = trajectories.yaws.array().cos();
    yaw_sin = trajectories.yaws.array().sin();

    // 初期のyaw_cosとyaw_sinの値を設定
    yaw_cos.col(0).setConstant(std::cos(initial_yaw));
    yaw_sin.col(0).setConstant(std::sin(initial_yaw));

    // dx, dyの計算
    decltype(yaw_cos) dx = state.vx.array() * yaw_cos.array() - state.vy.array() * yaw_sin.array();
    decltype(yaw_cos) dy = state.vx.array() * yaw_sin.array() + state.vy.array() * yaw_cos.array();

    // x, yの累積和を計算する
    decltype(dx) cumulative_dx = Eigen::MatrixXf::Zero(dx.rows(), dx.cols());
    decltype(dy) cumulative_dy = Eigen::MatrixXf::Zero(dy.rows(), dy.cols());
    for (int i = 1; i < dx.cols(); ++i) {
      cumulative_dx.col(i) = cumulative_dx.col(i - 1) + dx.col(i - 1) * settings.DT;
      cumulative_dy.col(i) = cumulative_dy.col(i - 1) + dy.col(i - 1) * settings.DT;
    }

    trajectories.x =
      cumulative_dx.colwise() + Eigen::VectorXf::Constant(dx.rows(), state.pose.pos.x());
    trajectories.y =
      cumulative_dy.colwise() + Eigen::VectorXf::Constant(dy.rows(), state.pose.pos.y());
  }

  void optimize(models::Path & path, Pose2D goal)
  {
    for (int i = 0; i < settings.ITERATIONS; i++) {
      auto [state, trajectories] = generateNoisedTrajectories();
      // critic_manager_.evalTrajectoriesScores(critics_data_);
      {
        getScore(state, trajectories, path, goal);
      }
      //      auto socre = getScore();
      updateControlSequence(state);
    }
  }

  std::pair<models::State<BATCH, STEP>, models::Trajectories<BATCH, STEP>>
  generateNoisedTrajectories()
  {
    models::State<BATCH, STEP> state;
    {
      noise_generator.generateNoisedControls();
      noise_generator.setNoised(control_sequence, state);

      std::cout << "[generateNoisedTrajectories] generated noised control length: "
                << state.cvx.rows() << std::endl;
    }

    //    noise_generator_.setNoisedControls(state_, control_sequence_);
    //    noise_generator_.generateNextNoises();
    //    updateStateVelocities(state_);
    {
      // updateInitialStateVelocities(state);
      {
        state.vx.col(0).setConstant(state.velocity.pos.x());
        state.vy.col(0).setConstant(state.velocity.pos.y());
        state.wz.col(0).setConstant(state.velocity.theta);
      }
      // propagateStateVelocitiesFromInitials(state);
      {
        motion_model.predict(state);
      }
    }

    // integrateStateVelocities(generated_trajectories_, state_);
    models::Trajectories<BATCH, STEP> trajectories;
    integrateStateVelocities(trajectories, state);
    return {state, trajectories};
  }

  void updateControlSequence(const models::State<BATCH, STEP> & state)
  {
    std::cout << "[updateControlSequence] updated control sequence size: "
              << control_sequence.vx.rows() << " x 1" << std::endl;
    // 各バッチのコストを計算
    Eigen::MatrixXf costs_ = Eigen::VectorXf::Zero(settings.BATCH_SIZE, 1);
    const auto & s = settings;

    //    Eigen::MatrixXf bounded_noises_vx = state.cvx;
    //    Eigen::MatrixXf bounded_noises_vy = state.cvy;
    //    Eigen::MatrixXf bounded_noises_wz = state.cwz;

    Eigen::MatrixXf bounded_noises_vx = noise_generator.noises_vx_;
    Eigen::MatrixXf bounded_noises_vy = noise_generator.noises_vy_;
    Eigen::MatrixXf bounded_noises_wz = noise_generator.noises_wz_;

    for (int i = 0; i < state.cvx.rows(); i++) {
      //      bounded_noises_vx.row(i) -= control_sequence.vx.transpose();
      //      bounded_noises_vy.row(i) -= control_sequence.vy.transpose();
      //      bounded_noises_wz.row(i) -= control_sequence.wz.transpose();
    }

    Eigen::MatrixXf control_sequence_vx_broadcasted =
      control_sequence.vx.replicate(1, settings.BATCH_SIZE).transpose();
    Eigen::MatrixXf control_sequence_vy_broadcasted =
      control_sequence.vy.replicate(1, settings.BATCH_SIZE).transpose();
    Eigen::MatrixXf control_sequence_wz_broadcasted =
      control_sequence.wz.replicate(1, settings.BATCH_SIZE).transpose();

    float scale = s.GAMMA / std::pow(s.VX_STD, 2);
    std::cout << "control_sequence_vx_broadcasted: " << control_sequence_vx_broadcasted.rows()
              << " x " << control_sequence_vx_broadcasted.cols() << std::endl;
    std::cout << "bounded_noises_vx: " << bounded_noises_vx.rows() << " x "
              << bounded_noises_vx.cols() << std::endl;
    auto product = (control_sequence_vx_broadcasted.array() * bounded_noises_vx.array());
    std::cout << "product: " << product.rows() << " x " << product.cols() << std::endl;
    auto a =
      (control_sequence_vx_broadcasted.array() * bounded_noises_vx.array()).rowwise().sum() * scale;
    std::cout << "a: " << a.rows() << " x " << a.cols() << std::endl;
    std::cout << "costs_: " << costs_.rows() << " x " << costs_.cols() << std::endl;

    // コストの更新
    costs_ += Eigen::MatrixXf(
      (control_sequence_vx_broadcasted.array() * bounded_noises_vx.array()).rowwise().sum() *
      s.GAMMA / std::pow(s.VX_STD, 2));
    costs_ += Eigen::MatrixXf(
      (control_sequence_vy_broadcasted.array() * bounded_noises_vy.array()).rowwise().sum() *
      s.GAMMA / std::pow(s.VY_STD, 2));
    costs_ += Eigen::MatrixXf(
      (control_sequence_wz_broadcasted.array() * bounded_noises_wz.array()).rowwise().sum() *
      s.GAMMA / std::pow(s.WZ_STD, 2));

    // コストの正規化とソフトマックスの計算
    Eigen::VectorXf costs_normalized =
      costs_ - Eigen::VectorXf::Constant(costs_.size(), costs_.minCoeff());
    std::cout << "[updateControlSequence] costs_normalized size: " << costs_normalized.rows()
              << " x 1" << std::endl;
    Eigen::VectorXf exponents = (-1.0f / s.TEMPERATURE * costs_normalized).array().exp();
    std::cout << "[updateControlSequence] exponents size: " << exponents.rows() << " x 1"
              << std::endl;
    Eigen::VectorXf softmaxes = exponents / exponents.sum();  // 1000x1
    std::cout << "[updateControlSequence] softmaxes size: " << softmaxes.rows() << " x 1"
              << std::endl;

    // TODO(HansRobo):
    //  バッチサイズ個あるcostやsoftmaxでタイムステップ個あるcontrol_sequenceを更新する処理が間違っている

    // ソフトマックスを用いたコントロールシーケンスの更新
    // 56 x 1
    // cvx(1000x56) * softmaxes(1000x1)

    // size: state.cvx.col(0).array()
    std::cout << "state.cvx.cols(0): " << state.cvx.col(0).rows() << " x "
              << state.cvx.col(0).cols() << std::endl;
    for (int i = 0; i < state.cvx.cols(); i++) {
      // 1000x1と1000x1の要素積 => 1000x1
      auto aaaa = (state.cvx.col(i).array() * softmaxes.array()).sum();
      // このsumを56個並べて更新したい
      //      std::cout << "aaaa: " << aaaa.rows() << " x " << aaaa.cols() << std::endl;
    }
    //    control_sequence.vx = (state.cvx.array().rowwise() * softmaxes).colwise().sum();
    control_sequence.vy = (state.cvy.array().colwise() * softmaxes.array()).rowwise().sum();
    control_sequence.wz = (state.cwz.array().colwise() * softmaxes.array()).rowwise().sum();

    std::cout << "[updateControlSequence] updated control sequence size: "
              << control_sequence.vx.rows() << " x 1" << std::endl;
    // applyControlSequenceConstraints();
    {
      // 最大最小値制約を適用
      control_sequence.vx = control_sequence.vx.cwiseMax(s.V_MIN).cwiseMin(s.V_MAX);
      std::cout << "[updateControlSequence] updated control sequence size: "
                << control_sequence.vx.rows() << " x 1" << std::endl;
      control_sequence.vy = control_sequence.vy.cwiseMax(s.V_MIN).cwiseMin(s.V_MAX);
      control_sequence.wz = control_sequence.wz.cwiseMax(-s.W_MAX).cwiseMin(s.W_MAX);
    }
  }

  double getScore(
    const models::State<BATCH, STEP> & state,
    const models::Trajectories<BATCH, STEP> & trajectories, const models::Path & path, Pose2D goal)
  {
    Eigen::VectorXf costs = Eigen::VectorXf::Zero(settings.BATCH_SIZE);
    // ゴール・パスへ合わせ込む
    {
      if ((state.pose.pos - goal.pos).norm() < 0.5) {
        // 近いときはゴールへ合わせ込む
        Eigen::MatrixXf dists = (trajectories.x.colwise() -
                                 Eigen::VectorXf::Constant(trajectories.x.rows(), goal.pos.x()))
                                  .array()
                                  .square() +
                                (trajectories.y.colwise() -
                                 Eigen::VectorXf::Constant(trajectories.y.rows(), goal.pos.y()))
                                  .array()
                                  .square();
        dists = dists.array().sqrt();

        auto get_diff_angle = [](auto from, auto to) {
          Eigen::MatrixXf angles = (to - from);
          const float pi = M_PI;
          const float two_pi = 2.0 * M_PI;

          // fmodで-πから+3πの範囲に制限し、その後の処理で-πから+πに正規化
          auto theta =
            ((angles.array() + pi).unaryExpr([&](auto angle) { return std::fmod(angle, two_pi); }) +
             two_pi)
              .unaryExpr([&](auto angle) { return std::fmod(angle, two_pi); }) -
            pi;

          // thetaが-πから0の範囲であれば、+πを追加し、それ以外では-πを引くことで正規化
          // ただし、Eigenでは直接的なwhere関数がないため、条件演算子を使用する
          return theta.unaryExpr([&](auto angle) { return angle > pi ? angle - two_pi : angle; });
        };
        // 角度の差を計算し、絶対値を取得
        dists = dists + Eigen::MatrixXf(get_diff_angle(
                                          trajectories.yaws, Eigen::VectorXf::Constant(
                                                               trajectories.x.rows(), goal.theta))
                                          .array()
                                          .abs());

        // 距離の平均を計算し、コストに追加
        float power_ = 1;
        float weight_ = 10.f;
        Eigen::VectorXf meanDists = dists.colwise().mean();  // 列ごとの平均を取得
        costs = costs + Eigen::VectorXf(meanDists.array().pow(power_).eval());
      } else {
        // 遠いときはPathへ合わせ込む
      }
    }
    return 0.0;
  }
};
}  // namespace crane

#endif  // CRANE_LOCAL_PLANNER__MPPI_HPP_
