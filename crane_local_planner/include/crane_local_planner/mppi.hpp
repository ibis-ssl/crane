// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__MPPI_HPP_
#define CRANE_LOCAL_PLANNER__MPPI_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <grid_map_core/GridMap.hpp>
#include <random>
#include <string>
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
    vx.setZero();
    vy.setZero();
    wz.setZero();
    cvx.setZero();
    cvy.setZero();
    cwz.setZero();
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

  void reset(int batch_size, unsigned int time_steps)
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
  const int ITERATIONS = 10;
  const double DT = 0.05;
  const double TEMPERATURE = 0.3;
  const double GAMMA = 0.015;
  const double V_MAX = 4.0;
  const double V_MIN = 0.0;
  const double W_MAX = 1.0;
  const double VX_STD = 0.2;
  const double VY_STD = 0.2;
  const double WZ_STD = 0.2;
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
    for (int i = 0; i < BATCH; ++i) {
      for (int j = 0; j < STEP; ++j) {
        noises_vx_(i, j) = dist_vx(gen);
        noises_vy_(i, j) = dist_vy(gen);
        noises_wz_(i, j) = dist_wz(gen);
      }
    }
    //    std::cout << "noise x: " << noises_vx_ << std::endl;
  }

  void setNoised(
    const models::ControlSequence<STEP> & control_sequence, models::State<BATCH, STEP> & state)
  {
    state.cvx = noises_vx_;
    state.cvy = noises_vy_;
    state.cwz = noises_wz_;

    for (int i = 0; i < state.cvx.rows(); i++) {
      state.cvx.row(i) += control_sequence.vx.transpose();
      state.cvy.row(i) += control_sequence.vy.transpose();
      state.cwz.row(i) += control_sequence.wz.transpose();
    }
  }
};

template <int BATCH, int STEP>
class Optimizer
{
public:
  OptimizerSettings settings;

  NoiseGenerator<BATCH, STEP> noise_generator;

  OmniMotionModel<BATCH, STEP> motion_model;

  models::ControlSequence<STEP> control_sequence;

  models::State<BATCH, STEP> state;

public:
  Optimizer()
  {
    noise_generator.initialize(settings);
    control_sequence.reset();
    state.reset();
  }

  void shiftControlSequence() {}

  // evalControl
  void calcCmd(
    std::vector<Point> path, Pose2D goal, Pose2D pose, Pose2D vel, const grid_map::GridMap & map,
    const std::string & layer)
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
    optimize(path_, goal, map, layer);
  }

  void integrateStateVelocities(models::Trajectories<BATCH, STEP> & trajectories) const
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
    //    decltype(trajectories.yaws) yaw_cos, yaw_sin;
    //    yaw_cos = trajectories.yaws.array().cos();
    //    yaw_sin = trajectories.yaws.array().sin();

    // 初期のyaw_cosとyaw_sinの値を設定
    //    yaw_cos.col(0).setConstant(std::cos(initial_yaw));
    //    yaw_sin.col(0).setConstant(std::sin(initial_yaw));

    // dx, dyの計算
    decltype(trajectories.x) dx = state.vx;
    decltype(trajectories.y) dy = state.vy;
    // decltype(trajectories.y) dy = state.vx.array() * yaw_sin.array()
    // + state.vy.array() * yaw_cos.array();

    // x, yの累積和を計算する
    decltype(dx) cumulative_dx, cumulative_dy;
    cumulative_dx.col(0).setConstant(state.pose.pos.x());
    cumulative_dy.col(0).setConstant(state.pose.pos.y());

    for (int i = 1; i < dx.cols(); ++i) {
      cumulative_dx.col(i) = cumulative_dx.col(i - 1) + dx.col(i - 1) * settings.DT;
      cumulative_dy.col(i) = cumulative_dy.col(i - 1) + dy.col(i - 1) * settings.DT;
    }

    trajectories.x =
      cumulative_dx.colwise() + Eigen::VectorXf::Constant(dx.rows(), state.pose.pos.x());
    trajectories.y =
      cumulative_dy.colwise() + Eigen::VectorXf::Constant(dy.rows(), state.pose.pos.y());
  }

  void optimize(
    models::Path & path, Pose2D goal, grid_map::GridMap & map, const std::string & layer)
  {
    for (int i = 0; i < settings.ITERATIONS; i++) {
      auto trajectories = generateNoisedTrajectories();
      auto cost = getScore(trajectories, path, goal, map, layer);
      updateControlSequence(cost);
    }
  }

  models::Trajectories<BATCH, STEP> generateNoisedTrajectories()
  {
    std::cout << "control sequence: " << control_sequence.vx.transpose() << std::endl;
    {
      noise_generator.generateNoisedControls();
      std::cout << "before cvx: " << state.cvx << std::endl;
      noise_generator.setNoised(control_sequence, state);
      std::cout << "noised cvx: " << state.cvx << std::endl;
    }

    {
      // updateInitialStateVelocities(state);
      {
        state.vx.col(0).setConstant(state.velocity.pos.x());
        state.vy.col(0).setConstant(state.velocity.pos.y());
        state.wz.col(0).setConstant(state.velocity.theta);
      }
      std::cout << "before vx: " << state.vx << std::endl;
      // propagateStateVelocitiesFromInitials(state);
      {
        motion_model.predict(state);
      }
      std::cout << "after vx: " << state.vx << std::endl;
    }

    // integrateStateVelocities(generated_trajectories_, state_);
    models::Trajectories<BATCH, STEP> trajectories;
    integrateStateVelocities(trajectories);
    return trajectories;
  }

  void updateControlSequence(Eigen::Matrix<float, BATCH, 1> & costs_)
  {
    std::cout << "[updateControlSequence] updated control sequence size: "
              << control_sequence.vx.rows() << " x 1" << std::endl;
    // 各バッチのコストを計算

    const auto & s = settings;

    // ノイズだけ
    Eigen::Matrix<float, BATCH, STEP> bounded_noises_vx = noise_generator.noises_vx_;
    Eigen::Matrix<float, BATCH, STEP> bounded_noises_vy = noise_generator.noises_vy_;
    Eigen::Matrix<float, BATCH, STEP> bounded_noises_wz = noise_generator.noises_wz_;

    // 次の計算用に制御入力列をバッチ分用意
    Eigen::Matrix<float, BATCH, STEP> control_sequence_vx_broadcasted =
      Eigen::Matrix<float, BATCH, STEP>(control_sequence.vx.replicate(1, BATCH).transpose());
    Eigen::Matrix<float, BATCH, STEP> control_sequence_vy_broadcasted =
      control_sequence.vy.replicate(1, BATCH).transpose();
    Eigen::Matrix<float, BATCH, STEP> control_sequence_wz_broadcasted =
      control_sequence.wz.replicate(1, BATCH).transpose();

    // コストの更新
    // 制御入力列とノイズ列の要素積をとり、各行（バッチごとの）の和をとる
    //    costs_ += Eigen::Matrix<float, BATCH, 1>(
    //      (control_sequence_vx_broadcasted.array() * bounded_noises_vx.array()).rowwise().sum() *
    //      s.GAMMA / std::pow(s.VX_STD, 2));
    //    costs_ += Eigen::Matrix<float, BATCH, 1>(
    //      (control_sequence_vy_broadcasted.array() * bounded_noises_vy.array()).rowwise().sum() *
    //      s.GAMMA / std::pow(s.VY_STD, 2));
    //    costs_ += Eigen::Matrix<float, BATCH, 1>(
    //      (control_sequence_wz_broadcasted.array() * bounded_noises_wz.array()).rowwise().sum() *
    //      s.GAMMA / std::pow(s.WZ_STD, 2));

    // コストをソフトマックス関数で正規化する
    // コストの各成分は(0,1)区間に収まり、コスト合計は1になる
    Eigen::Vector<float, BATCH> costs_softmax = [costs_, s]() {
      std::cout << "min: " << costs_.minCoeff() << std::endl;
      Eigen::Vector<float, BATCH> costs_normalized =
        costs_ - Eigen::Vector<float, BATCH>::Constant(costs_.minCoeff());
      // - 1.0f / s.TEMPERATUREをかけてexpの計算 =>
      Eigen::Vector<float, BATCH> exponents =
        (-1.0f / s.TEMPERATURE * costs_normalized).array().exp();
      float sum = exponents.sum();
      Eigen::Vector<float, BATCH> result = exponents / sum;  // 1000x1
      return result;
    }();
    costs_ = costs_softmax;

    // コストを用いた制御入力列の更新
    // 全てのバッチに対して、コストの重み付き和を計算し、その結果をcontrol_sequenceに格納する

    for (int i = 0; i < state.cvx.cols(); i++) {
      // 1000x1と1000x1の要素積 => 1000x1
      control_sequence.vx(i) = (state.cvx.col(i).array() * costs_softmax.array()).sum();
      control_sequence.vy(i) = (state.cvy.col(i).array() * costs_softmax.array()).sum();
      control_sequence.wz(i) = (state.cwz.col(i).array() * costs_softmax.array()).sum();
    }

    // applyControlSequenceConstraints();
    {
      // 最大最小値制約を適用
      control_sequence.vx = control_sequence.vx.cwiseMax(s.V_MIN).cwiseMin(s.V_MAX);
      control_sequence.vy = control_sequence.vy.cwiseMax(s.V_MIN).cwiseMin(s.V_MAX);
      control_sequence.wz = control_sequence.wz.cwiseMax(-s.W_MAX).cwiseMin(s.W_MAX);
    }
  }

  Eigen::Vector<float, BATCH> getScore(
    const models::Trajectories<BATCH, STEP> & trajectories, const models::Path & path, Pose2D goal,
    grid_map::GridMap & map, const std::string & layer)
  {
    Eigen::Vector<float, BATCH> costs;
    costs.setZero();
    // ゴール・パスへ合わせ込む
    if ((state.pose.pos - goal.pos).norm() < 0.5) {
      /**
       * 近いときはゴールへ合わせ込む
       */
      Eigen::Matrix<float, BATCH, STEP> distances =
        (trajectories.x.colwise() -
         Eigen::Vector<float, BATCH>::Constant(trajectories.x.rows(), goal.pos.x()))
          .array()
          .square() +
        (trajectories.y.colwise() -
         Eigen::Vector<float, BATCH>::Constant(trajectories.y.rows(), goal.pos.y()))
          .array()
          .square();
      std::cout << "x: " << trajectories.x << std::endl;
      auto a = Eigen::Vector<float, BATCH>::Constant(trajectories.x.rows(), goal.pos.x());
      std::cout << "a: " << a << std::endl;

      auto b = trajectories.x.colwise() - a;
      std::cout << "b: " << b << std::endl;

      std::cout << "distances: " << distances << std::endl;

      // TODO(HansRobo): 角度関連の計算にバグがある。足すとdistancesがnanになる
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
      //      distances =
      //        distances +
      //        Eigen::Matrix<float, BATCH, STEP>(
      // get_diff_angle(trajectories.yaws, Eigen::Matrix<float, BATCH, STEP>::Constant(goal.theta))
      //            .array()
      //            .abs());

      // 距離の平均を計算し、コストに追加
      float power_ = 1;
      float weight_ = 10.f;
      auto meanDistances = distances.rowwise().mean().transpose();  // 列ごとの平均を取得
      std::cout << "meanDistances: " << meanDistances << std::endl;
      costs = costs + Eigen::Vector<float, BATCH>(meanDistances.array().pow(power_)) * weight_;
    }
    //      } else {
    //        /**
    //         * 遠いときはPathへ合わせ込む
    //         */
    //        std::vector<float> path_cumulative_distance(path.x.size(), 0.);
    //        for (int i = 1; i < path.x.size(); i++) {
    //          path_cumulative_distance[i] =
    //            path_cumulative_distance[i - 1] +
    //            std::hypot(path.x[i] - path.x[i - 1], path.y[i] - path.y[i - 1]);
    //        }
    //
    //        constexpr int SAMPLING_INTERVAL = 4;
    //        for (int i = 0; i < BATCH; i++) {
    //          float traj_cumulative_distance = 0.f;
    //          float sample_sum = 0.f;
    //          float distance_sum_to_ref_path = 0.f;
    //          int closest_path_pt_index = 0;
    //          for (int j = SAMPLING_INTERVAL; j < STEP; j += SAMPLING_INTERVAL) {
    //            traj_cumulative_distance += std::hypot(
    //              trajectories.x(i, j) - trajectories.x(i, j - SAMPLING_INTERVAL),
    //              trajectories.y(i, j) - trajectories.y(i, j - SAMPLING_INTERVAL));
    //// 最も近いパス上の点を探す
    //// 参照パスと比較パスは始点が同じであるため、
    /// 直接距離を計算せずとも累積距離が近い点が近い点とみなすことが出来る。
    //            closest_path_pt_index = [&](int start_index, float matching_distance) -> int {
    //              // 二分探索で調べる
    //          auto iter = std::lower_bound(
    //           path_cumulative_distance.begin() + start_index, path_cumulative_distance.end(),
    //                matching_distance);
    //              if (iter == path_cumulative_distance.begin() + start_index) {
    //                return 0;
    //              }
    //              if (matching_distance - *(iter - 1) < *iter - matching_distance) {
    //                return iter - 1 - path_cumulative_distance.begin();
    //              }
    //              return iter - path_cumulative_distance.begin();
    //            }(closest_path_pt_index, traj_cumulative_distance);
    //
    //            if (closest_path_pt_index < path.x.size()) {
    //              sample_sum += 1.f;
    //              distance_sum_to_ref_path += std::hypot(
    //                trajectories.x(i, j) - path.x[closest_path_pt_index],
    //                trajectories.y(i, j) - path.y[closest_path_pt_index]);
    //            }
    //          }
    //
    //          // サンプル数によってコストが変わらないように正規化
    //   distance_sum_to_ref_path = sample_sum > 0 ? distance_sum_to_ref_path / sample_sum : 0.f;
    //          float power_ = 1;
    //          float weight_ = 10.f;
    //          costs[i] += std::pow(distance_sum_to_ref_path, power_) * weight_;
    //        }
    //      }
    /*
       * コストマップの情報をコストに追加
       */
    {
      for (int i = 0; i < BATCH; i++) {
        float cost = 0.f;
        for (int j = 0; j < STEP; j++) {
          grid_map::Index index;
          grid_map::Position pos(trajectories.x(i, j), trajectories.y(i, j));
          map.getIndex(pos, index);
          if (map.isInside(pos)) {
            auto map_cost = map.at(layer, index);
            cost += map_cost;
            if (map_cost >= 1.f) {
              cost += 1000.f;
            } else {
              cost += map_cost;
            }
          } else {
            cost += 1000.f;
          }
        }
        float power_ = 1;
        float weight_ = 10.f;
        //        costs[i] += std::pow(cost, power_) * weight_;
      }
    }
    return costs;
  }
};
}  // namespace crane

#endif  // CRANE_LOCAL_PLANNER__MPPI_HPP_
