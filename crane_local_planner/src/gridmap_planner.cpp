// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/gridmap_planner.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // tf2::getYaw

#include <nav_msgs/msg/path.hpp>
#include <random>

constexpr static int debug_id = -1;

namespace crane
{
namespace models
{
struct State
{
  Pose2D pose;
  Pose2D velocity;
  Eigen::MatrixXf vx, vy, wz;     // 車両の速度
  Eigen::MatrixXf cvx, cvy, cwz;  // 制御速度
  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    vx = Eigen::MatrixXf::Zero(batch_size, time_steps);
    vy = Eigen::MatrixXf::Zero(batch_size, time_steps);
    wz = Eigen::MatrixXf::Zero(batch_size, time_steps);
    cvx = Eigen::MatrixXf::Zero(batch_size, time_steps);
    cvy = Eigen::MatrixXf::Zero(batch_size, time_steps);
    cwz = Eigen::MatrixXf::Zero(batch_size, time_steps);
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

struct Trajectories
{
  Eigen::MatrixXf x;
  Eigen::MatrixXf y;
  Eigen::MatrixXf yaws;

  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    x = Eigen::MatrixXf::Zero(batch_size, time_steps);
    y = Eigen::MatrixXf::Zero(batch_size, time_steps);
    yaws = Eigen::MatrixXf::Zero(batch_size, time_steps);
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

struct ControlSequence
{
  Eigen::VectorXf vx;
  Eigen::VectorXf vy;
  Eigen::VectorXf wz;

  void reset(unsigned int time_steps)
  {
    vx = Eigen::VectorXf::Zero(time_steps);
    vy = Eigen::VectorXf::Zero(time_steps);
    wz = Eigen::VectorXf::Zero(time_steps);
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
class MotionModel
{
public:
  virtual void predict(models::State & state) {}
  virtual void applyConstraints(models::ControlSequence & /*control_sequence*/) {}
};

class OmniMotionModel : public MotionModel
{
public:
  void predict(models::State & state) override
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

struct NoiseGenerator
{
  std::mt19937 gen;
  std::normal_distribution<float> dist_vx;
  std::normal_distribution<float> dist_vy;
  std::normal_distribution<float> dist_wz;
  Eigen::MatrixXf noises_vx_;
  Eigen::MatrixXf noises_vy_;
  Eigen::MatrixXf noises_wz_;
  void initialize(OptimizerSettings & s)
  {
    gen = std::mt19937(std::random_device()());
    dist_vx = std::normal_distribution<float>(0.0f, s.VX_STD);
    dist_vy = std::normal_distribution<float>(0.0f, s.VY_STD);
    dist_wz = std::normal_distribution<float>(0.0f, s.WZ_STD);
  }

  void generateNoisedControls(OptimizerSettings & s)
  {
    noises_vx_ = Eigen::MatrixXf(s.BATCH_SIZE, s.TIME_STEPS);
    noises_vy_ = Eigen::MatrixXf(s.BATCH_SIZE, s.TIME_STEPS);
    noises_wz_ = Eigen::MatrixXf(s.BATCH_SIZE, s.TIME_STEPS);

    for (int i = 0; i < s.BATCH_SIZE; ++i) {
      for (int j = 0; j < s.TIME_STEPS; ++j) {
        noises_vx_(i, j) = dist_vx(gen);
        noises_vy_(i, j) = dist_vy(gen);
        noises_wz_(i, j) = dist_wz(gen);
      }
    }
  }

  models::ControlSequence getNoised(const models::ControlSequence & control_sequence)
  {
    models::ControlSequence noised_control_sequence;
    noised_control_sequence.vx = control_sequence.vx.array() + noises_vx_.array();
    noised_control_sequence.vy = control_sequence.vy.array() + noises_vy_.array();
    noised_control_sequence.wz = control_sequence.wz.array() + noises_wz_.array();
    return noised_control_sequence;
  }
};

class Optimizer
{
private:
  OptimizerSettings settings;

  NoiseGenerator noise_generator;

  OmniMotionModel motion_model;

  models::ControlSequence control_sequence;

public:
  Optimizer() { noise_generator.initialize(settings); }
  void prepare() {}

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
    optimize();
    auto control = getControlFromSequenceAsTwist();
    return control;
  }

  void getControlFromSequenceAsTwist()
  {
    unsigned int offset = settings_.shift_control_sequence ? 1 : 0;

    auto vx = control_sequence_.vx(offset);
    auto vy = control_sequence_.vy(offset);
    auto wz = control_sequence_.wz(offset);

    return utils::toTwistStamped(vx, vy, wz, stamp, costmap_ros_->getBaseFrameID());
  }

  void integrateStateVelocities(models::Trajectories & trajectories, const models::State & state)
  {
    const float initial_yaw = state.pose.theta;

    // wzに基づくyawの累積和を計算する
    Eigen::MatrixXf cumulative_wz = Eigen::MatrixXf::Zero(state.wz.rows(), state.wz.cols());
    cumulative_wz.col(0).setConstant(initial_yaw);
    for (int i = 1; i < state.wz.cols(); ++i) {
      cumulative_wz.col(i) = cumulative_wz.col(i - 1) + state.wz.col(i - 1) * settings.DT;
    }
    trajectories.yaws = cumulative_wz;

    // yawのコサインとサインを計算する
    Eigen::MatrixXf yaw_cos =
      Eigen::MatrixXf::Zero(trajectories.yaws.rows(), trajectories.yaws.cols());
    Eigen::MatrixXf yaw_sin =
      Eigen::MatrixXf::Zero(trajectories.yaws.rows(), trajectories.yaws.cols());
    yaw_cos = trajectories.yaws.array().cos();
    yaw_sin = trajectories.yaws.array().sin();

    // 初期のyaw_cosとyaw_sinの値を設定
    yaw_cos.col(0).setConstant(std::cos(initial_yaw));
    yaw_sin.col(0).setConstant(std::sin(initial_yaw));

    // dx, dyの計算
    Eigen::MatrixXf dx = state.vx.array() * yaw_cos.array() - state.vy.array() * yaw_sin.array();
    Eigen::MatrixXf dy = state.vx.array() * yaw_sin.array() + state.vy.array() * yaw_cos.array();

    // x, yの累積和を計算する
    Eigen::MatrixXf cumulative_dx = Eigen::MatrixXf::Zero(dx.rows(), dx.cols());
    Eigen::MatrixXf cumulative_dy = Eigen::MatrixXf::Zero(dy.rows(), dy.cols());
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

  std::pair<models::State, models::Trajectories> generateNoisedTrajectories()
  {
    models::State state;
    state.reset(settings.BATCH_SIZE, settings.TIME_STEPS);
    {
      noise_generator.generateNoisedControls(settings);
      auto noised = noise_generator.getNoised(control_sequence);

      state.cvx = noised.vx;
      state.cvy = noised.vy;
      state.cwz = noised.wz;
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
    models::Trajectories trajectories;
    integrateStateVelocities(trajectories, state);
    return {state, trajectories};
  }

  void updateControlSequence(const models::State & state)
  {
    Eigen::VectorXf costs_ = Eigen::VectorXf::Zero(settings.BATCH_SIZE);
    const auto & s = settings;

    Eigen::VectorXf bounded_noises_vx = state.cvx - control_sequence.vx;
    Eigen::VectorXf bounded_noises_vy = state.cvy - control_sequence.vy;
    Eigen::VectorXf bounded_noises_wz = state.cwz - control_sequence.wz;

    // コストの更新
    costs_ +=
      s.GAMMA / std::pow(s.VX_STD, 2) * (control_sequence.vx.transpose() * bounded_noises_vx);
    costs_ +=
      s.GAMMA / std::pow(s.VY_STD, 2) * (control_sequence.vy.transpose() * bounded_noises_vy);
    costs_ +=
      s.GAMMA / std::pow(s.WZ_STD, 2) * (control_sequence.wz.transpose() * bounded_noises_wz);

    // コストの正規化とソフトマックスの計算
    Eigen::VectorXf costs_normalized =
      costs_ - Eigen::VectorXf::Constant(costs_.size(), costs_.minCoeff());
    Eigen::VectorXf exponents = (-1.0f / s.TEMPERATURE * costs_normalized).array().exp();
    Eigen::VectorXf softmaxes = exponents / exponents.sum();

    // ソフトマックスを用いたコントロールシーケンスの更新
    control_sequence.vx = (state.cvx.array().colwise() * softmaxes.array()).rowwise().sum();
    control_sequence.vy = (state.cvy.array().colwise() * softmaxes.array()).rowwise().sum();
    control_sequence.wz = (state.cwz.array().colwise() * softmaxes.array()).rowwise().sum();

    // applyControlSequenceConstraints();
    {
      // 最大最小値制約を適用
      control_sequence.vx = control_sequence.vx.cwiseMax(s.V_MIN).cwiseMin(s.V_MAX);
      control_sequence.vy = control_sequence.vy.cwiseMax(s.V_MIN).cwiseMin(s.V_MAX);
      control_sequence.wz = control_sequence.wz.cwiseMax(-s.W_MAX).cwiseMin(s.W_MAX);
    }
  }

  double getScore(
    const models::State & state, const models::Trajectories & trajectories,
    const models::Path & path, Pose2D goal)
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

GridMapPlanner::GridMapPlanner(rclcpp::Node & node)
: map({"penalty", "ball_placement", "theirs", "ours", "ball", "path/0"})
{
  node.declare_parameter("map_resolution", MAP_RESOLUTION);
  MAP_RESOLUTION = node.get_parameter("map_resolution").as_double();

  gridmap_publisher =
    node.create_publisher<grid_map_msgs::msg::GridMap>("local_planner/grid_map", 1);
  map.setFrameId("map");
  map.setGeometry(grid_map::Length(1.2, 2.0), MAP_RESOLUTION, grid_map::Position(0.0, 0.0));

  path_publisher = node.create_publisher<nav_msgs::msg::Path>("local_planner/path", 1);
}

std::vector<grid_map::Index> GridMapPlanner::findPathAStar(
  const Point & start_point, const Point & goal_point, const std::string & layer,
  const uint8_t robot_id)
{
  auto isMapInside = [&](const grid_map::Index & index) -> bool {
    grid_map::Position p;
    return map.getPosition(index, p);
  };

  auto isObstacle = [&](const grid_map::Index & index) -> bool {
    return map.at(layer, index) >= 0.5f;
  };

  // 注意：コストでソートするためにAstarNodeをKeyにしている
  std::multimap<AStarNode, grid_map::Index> openSet;
  std::unordered_map<grid_map::Index, AStarNode, EigenArrayHash, EigenArrayEqual> closedSet;

  AStarNode start;
  map.getIndex(start_point, start.index);
  if (not isMapInside(start.index)) {
    if (robot_id == debug_id) {
      std::cout << "start is not in the map" << std::endl;
    }
    return {};
  }

  // 脱出モード：障害物の中にいる場合障害物の外に出るまで、障害物関係なく経路を探索する
  bool escape_mode = isObstacle(start.index);

  AStarNode goal;
  map.getIndex(goal_point, goal.index);

  // ゴールが障害物内にある場合、最寄りの障害物外の点を探索してゴールとする
  auto find_alternative_goal = [&](double search_radius) -> grid_map::Index {
    for (grid_map::SpiralIterator goal_candidate(map, goal_point, search_radius);
         !goal_candidate.isPastEnd(); ++goal_candidate) {
      if (isMapInside(*goal_candidate) and not isObstacle(*goal_candidate)) {
        return *goal_candidate;
      }
    }
    return goal.index;
  };

  if (not isMapInside(goal.index)) {
    if (robot_id == debug_id) {
      std::cout << "goal is not in the map. replace goal" << std::endl;
    }
    auto alternative_goal = find_alternative_goal(1.0);
    if (alternative_goal.x() != goal.index.x() or alternative_goal.y() != goal.index.y()) {
      goal.index = alternative_goal;
    } else {
      if (robot_id == debug_id) {
        std::cout << "failed to find alternative goal" << std::endl;
      }
      return {};
    }
  } else if (isObstacle(goal.index)) {
    if (robot_id == debug_id) {
      std::cout << "goal is in obstacle" << std::endl;
    }
    auto alternative_goal = find_alternative_goal(1.0);
    if (alternative_goal.x() != goal.index.x() or alternative_goal.y() != goal.index.y()) {
      goal.index = alternative_goal;
    } else {
      if (robot_id == debug_id) {
        std::cout << "failed to find alternative goal" << std::endl;
      }
      return {};
    }
  }

  start.h = start.calcHeuristic(goal.index);
  start.g = 0.;
  openSet.emplace(start, start.index);

  if (not map.exists("closed")) {
    map.add("closed");
  }
  map["closed"].setZero();

  while (!openSet.empty()) {
    // openリストの先頭の要素を取得＆pop
    auto current = openSet.begin()->first;
    openSet.erase(openSet.begin());

    closedSet[current.index] = current;

    // ゴール判定
    if (current.index.x() == goal.index.x() && current.index.y() == goal.index.y()) {
      // ゴールからスタートまでの経路を取得
      std::vector<grid_map::Index> path;
      path.emplace_back(current.index);
      while (current.parent_index) {
        path.push_back(current.parent_index.value());
        current = closedSet[current.parent_index.value()];
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    // 8方のマスをOpen
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;  // 自分はスキップ
        AStarNode next;
        next.index = current.index + grid_map::Index(dx, dy);
        next.parent_index = current.index;
        next.g = current.g + std::hypot(dx, dy);
        next.h = next.calcHeuristic(goal.index);

        // マップ外ならスキップ
        if (not isMapInside(next.index)) continue;

        // 障害物以外なら進む
        if (not escape_mode && isObstacle(next.index)) continue;

        // 脱出モードを更新
        if (escape_mode) {
          escape_mode = isObstacle(next.index);
        }

        // closedSetとopenSetに含まれていない場合のみ追加
        if (
          closedSet.count(next.index) == 0 &&
          std::find_if(openSet.begin(), openSet.end(), [index = next.index](const auto & elem) {
            return elem.second.x() == index.x() && elem.second.y() == index.y();
          }) == openSet.end()) {
          openSet.emplace(next, next.index);
        }
      }
    }
  }
  std::cout << "openSet is empty" << std::endl;
  return {};
}
crane_msgs::msg::RobotCommands GridMapPlanner::calculateRobotCommand(
  const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model)
{
  // update map size

  static Vector2 defense_area_size;

  if (
    map.getLength().x() != world_model->field_size.x() + world_model->getFieldMargin() * 2. ||
    map.getLength().y() != world_model->field_size.y() + world_model->getFieldMargin() * 2.) {
    map.clearAll();
    map.setGeometry(
      grid_map::Length(
        world_model->field_size.x() + world_model->getFieldMargin() * 2.,
        world_model->field_size.y() + world_model->getFieldMargin() * 2),
      MAP_RESOLUTION);
    defense_area_size << 0, 0;
  }

  // DefenseSize更新時にdefense_areaを更新する
  if (
    defense_area_size.x() != world_model->defense_area_size.x() ||
    defense_area_size.y() != world_model->defense_area_size.y()) {
    std::cout << "update defense_area" << std::endl;
    defense_area_size = world_model->defense_area_size;
    if (not map.exists("defense_area")) {
      map.add("defense_area");
    }
    map["defense_area"].setZero();

    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
      grid_map::Position position;
      map.getPosition(*iterator, position);
      map.at("defense_area", *iterator) = world_model->isDefenseArea(position) ? 1.f : 0.f;
    }
  }

  // ボールプレイスメントMap
  if (not map.exists("ball_placement")) {
    map.add("ball_placement");
  }
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    map.getPosition(*iterator, position);
    map.at("ball_placement", *iterator) = world_model->isBallPlacementArea(position, 0.2);
  }

  // 味方ロボットMap
  if (not map.exists("friend_robot")) {
    map.add("friend_robot");
  }
  map["friend_robot"].setZero();
  for (const auto & robot : world_model->ours.getAvailableRobots()) {
    for (grid_map::CircleIterator iterator(map, robot->pose.pos, 0.3); !iterator.isPastEnd();
         ++iterator) {
      map.at("friend_robot", *iterator) = 1.0;
    }
  }

  // 敵ロボットMap
  if (not map.exists("enemy_robot")) {
    map.add("enemy_robot");
  }
  map["enemy_robot"].setZero();
  for (const auto & robot : world_model->theirs.getAvailableRobots()) {
    for (grid_map::CircleIterator iterator(map, robot->pose.pos, 0.3); !iterator.isPastEnd();
         ++iterator) {
      map.at("enemy_robot", *iterator) = 1.0;
    }
  }

  // ボールMap
  if (not map.exists("ball")) {
    map.add("ball");
  }
  map["ball"].setZero();
  for (grid_map::CircleIterator iterator(map, world_model->ball.pos, 0.2); !iterator.isPastEnd();
       ++iterator) {
    map.at("ball", *iterator) = 1.0;
  }

  // ボールMap (時間)
  if (not map.exists("ball_time")) {
    map.add("ball_time");
  }
  map["ball_time"].setZero();
  Vector2 ball_vel_unit = world_model->ball.vel.normalized() * MAP_RESOLUTION;
  Point ball_pos = world_model->ball.pos;
  float time = 0.f;
  const double TIME_STEP = MAP_RESOLUTION / world_model->ball.vel.norm();
  map["ball_time"].setConstant(100.0);
  for (int i = 0; i < 100; ++i) {
    for (grid_map::CircleIterator iterator(map, ball_pos, 0.05); !iterator.isPastEnd();
         ++iterator) {
      map.at("ball_time", *iterator) = std::min(map.at("ball_time", *iterator), time);
    }
    ball_pos += ball_vel_unit;
    time += TIME_STEP;
  }
  std::unique_ptr<grid_map_msgs::msg::GridMap> message;
  message = grid_map::GridMapRosConverter::toMessage(map);

  gridmap_publisher->publish(std::move(message));

  crane_msgs::msg::RobotCommands commands = msg;
  for (auto & command : commands.robot_commands) {
    if ((not command.target_x.empty()) && (not command.target_y.empty())) {
      auto robot = world_model->getOurRobot(command.robot_id);
      Point target;
      target << command.target_x.front(), command.target_y.front();
      std::string map_name = "cost/" + std::to_string(command.robot_id);
      if (not map.exists(map_name)) {
        map.add(map_name);
      }
      map[map_name].setZero();

      if (not command.local_planner_config.disable_collision_avoidance) {
        map[map_name] += map.get("friend_robot");
        // delete current robot position
        for (grid_map::CircleIterator iterator(map, robot->pose.pos, 0.31); !iterator.isPastEnd();
             ++iterator) {
          map.at(map_name, *iterator) = 0.;
        }

        map[map_name] += map["enemy_robot"];
      }

      if (not command.local_planner_config.disable_ball_avoidance) {
        map[map_name] += map["ball"];
      }

      if (not command.local_planner_config.disable_goal_area_avoidance) {
        map[map_name] += map["defense_area"];
      }

      if (not command.local_planner_config.disable_placement_avoidance) {
        map[map_name] += map["ball_placement"];
      }

      auto route = findPathAStar(robot->pose.pos, target, map_name, command.robot_id);

      std::vector<Point> path;
      for (const auto & node : route) {
        Point p;
        map.getPosition(node, p);
        path.push_back(p);
      }

      if (path.size() < 2) {
        path.push_back(robot->pose.pos);
        path.push_back(target);
      }

      // ゴール地点の量子化誤差を除去
      if ((path.back() - target).norm() < 0.05) {
        path.back() = target;
      }

      if (command.robot_id == debug_id) {
        if (not map.exists("path/0")) {
          map.add("path/0", 0.f);
        }
        map.get("path/0").setZero();
        float cost = 0.5f;
        for (const auto & p : path) {
          grid_map::Index index;
          map.getIndex(p, index);
          map.at("path/0", index) = cost;
          cost += 0.1f;
        }
      }

      const double a = 0.5;
      const double b = 0.8;

      auto smooth_path = path;

      for (int l = 0; l < 3; l++) {
        for (int i = 0; i < static_cast<int>(smooth_path.size()); i++) {
          smooth_path[i] = smooth_path[i] - a * (smooth_path[i] - path[i]);
        }
        for (int i = 1; i < static_cast<int>(smooth_path.size()) - 1; i++) {
          smooth_path[i] =
            smooth_path[i] - b * (2 * smooth_path[i] - smooth_path[i - 1] - smooth_path[i + 1]);
        }
      }

      if (command.robot_id == debug_id) {
        nav_msgs::msg::Path path_msg;
        for (const auto & p : smooth_path) {
          geometry_msgs::msg::PoseStamped pose;
          pose.pose.position.x = p.x();
          pose.pose.position.y = p.y();
          path_msg.poses.push_back(pose);
        }
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = rclcpp::Time(0);
        path_publisher->publish(path_msg);
      }

      std::vector<double> velocity(smooth_path.size(), 0.0);
      velocity[0] = robot->vel.linear.norm();
      velocity.back() = command.local_planner_config.terminal_velocity;

      // 最終速度を考慮した速度
      for (int i = static_cast<int>(smooth_path.size()) - 2; i > 0; i--) {
        double distance = (smooth_path[i + 1] - smooth_path[i]).norm();
        velocity[i] = std::min(
          std::sqrt(
            velocity[i + 1] * velocity[i + 1] +
            2 * command.local_planner_config.max_acceleration * distance),
          static_cast<double>(command.local_planner_config.max_velocity));
      }

      // 現在速度を考慮した速度
      for (int i = 1; i < static_cast<int>(smooth_path.size()); i++) {
        double distance = (smooth_path[i] - smooth_path[i - 1]).norm();
        velocity[i] = std::min(
          velocity[i], std::sqrt(
                         velocity[i - 1] * velocity[i - 1] +
                         2 * command.local_planner_config.max_acceleration * distance));
      }

      command.target_x.clear();
      command.target_y.clear();
      command.target_x.push_back(smooth_path[1].x());
      command.target_y.push_back(smooth_path[1].y());
      Velocity global_vel = (smooth_path[1] - robot->pose.pos).normalized() * velocity[1];

      command.target_velocity.x = global_vel.x();
      command.target_velocity.y = global_vel.y();
    }
  }
  return commands;
}
}  // namespace crane
