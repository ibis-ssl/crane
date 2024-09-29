// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/vo_planner.hpp"

#include <matplotlibcpp17/common.h>
#include <matplotlibcpp17/patches.h>
#include <matplotlibcpp17/pyplot.h>
#include <osqp/osqp.h>
#include <pybind11/pybind11.h>

#include <Eigen/Sparse>

namespace matplotlibcpp17::patches
{
/**
 * @brief A wrapper class for matplotlib.patches.Arrow
 **/
struct DECL_STRUCT_ATTR Arrow : public BaseWrapper
{
public:
  Arrow(
    const pybind11::tuple & args = pybind11::tuple(),
    const pybind11::dict & kwargs = pybind11::dict())
  {
    arrow_attr = pybind11::module::import("matplotlib.patches").attr("Arrow");
    self = arrow_attr(*args, **kwargs);
  }

private:
  pybind11::object arrow_attr;
};
}  // namespace matplotlibcpp17::patches

namespace crane
{
class OsqpSolver
{
public:
  OsqpSolver()
  {
    settings_ = std::make_unique<OSQPSettings>();
    data_ = std::make_unique<OSQPData>();
    // デフォルトの設定をロード
    osqp_set_default_settings(settings_.get());
    settings_->verbose = false;
  }

  ~OsqpSolver()
  {
    if (workspace_) {
      osqp_cleanup(workspace_);
    }
  }

  void initialize(
    const Eigen::SparseMatrix<double> & P, const Eigen::VectorXd & q,
    const Eigen::SparseMatrix<double> & A, const Eigen::VectorXd & l, const Eigen::VectorXd & u)
  {
    // データ構造を初期化
    data_->n = P.cols();  // 変数の数
    data_->m = A.rows();  // 制約の数

    // Eigenの行列をOSQPの形式に変換 (CSC形式)
    data_->P = eigenToCSC(P);
    data_->q = const_cast<double *>(q.data());
    data_->A = eigenToCSC(A);
    data_->l = const_cast<double *>(l.data());
    data_->u = const_cast<double *>(u.data());

    // OSQP問題を初期化
    if (osqp_setup(&workspace_, data_.get(), settings_.get()) != 0) {
      throw std::runtime_error("OSQPの初期化に失敗しました。");
    }
  }

  Eigen::VectorXd solve()
  {
    if (!workspace_) {
      throw std::runtime_error("ソルバーが初期化されていません。");
    }

    // OSQPを解く
    osqp_solve(workspace_);

    // 最適化が成功したかをチェック
    if (workspace_->info->status_val != OSQP_SOLVED) {
      throw std::runtime_error("最適化に失敗しました。");
    }

    // 解を取得
    return Eigen::Map<Eigen::VectorXd>(workspace_->solution->x, data_->n);
  }

private:
  std::unique_ptr<OSQPSettings> settings_;
  std::unique_ptr<OSQPData> data_;
  OSQPWorkspace * workspace_ = nullptr;

  // Eigen::SparseMatrixをCSC形式に変換するヘルパー関数
  csc * eigenToCSC(const Eigen::SparseMatrix<double> & mat)
  {
    // 非ゼロ要素の数
    int nnz = mat.nonZeros();

    // OSQPのCSC形式に合わせてメモリを確保
    csc * csc_matrix = static_cast<csc *>(c_malloc(sizeof(csc)));
    csc_matrix->m = mat.rows();
    csc_matrix->n = mat.cols();
    csc_matrix->nzmax = nnz;
    csc_matrix->x = static_cast<c_float *>(c_malloc(nnz * sizeof(c_float)));
    csc_matrix->i = static_cast<c_int *>(c_malloc(nnz * sizeof(c_int)));
    csc_matrix->p = static_cast<c_int *>(c_malloc((mat.cols() + 1) * sizeof(c_int)));

    // EigenのデータをCSC形式にコピー
    std::copy(mat.valuePtr(), mat.valuePtr() + nnz, csc_matrix->x);
    std::copy(mat.innerIndexPtr(), mat.innerIndexPtr() + nnz, csc_matrix->i);
    std::copy(mat.outerIndexPtr(), mat.outerIndexPtr() + mat.cols() + 1, csc_matrix->p);

    return csc_matrix;
  }
};
Eigen::Vector2d optimizeVelocity(
  const RobotInfo::SharedPtr & robot, const Eigen::Vector2d & goal_pos,
  const std::vector<RobotInfo::SharedPtr> & other_robots, double d_min)
{
  const int n = 2;  // 2次元ベクトル

  // 目的地への希望速度
  Velocity v_des = (goal_pos - robot->pose.pos).normalized() * 1.0;  // スケーリング係数は1.0

  // 目的関数の二次項：min (1/2) * x' * P * x + q' * x で P = I, q = -v_des
  Eigen::SparseMatrix<double> P(n, n);
  P.setIdentity();             // 単位行列
  Eigen::VectorXd q = -v_des;  // 希望速度に基づく目的関数の線形項

  // 制約の数（他のロボットとの衝突回避制約）
  int num_constraints = other_robots.size();

  // 制約行列 A * x <= b を構築
  Eigen::SparseMatrix<double> A(num_constraints, n);
  Eigen::VectorXd b(num_constraints);

  // 他のロボットとの相対位置と速度を基に制約を設定
  for (int i = 0; i < num_constraints; ++i) {
    const auto & other_robot = other_robots[i];
    Eigen::Vector2d p_ij = robot->pose.pos - other_robot->pose.pos;
    Eigen::Vector2d v_ij = robot->vel.linear - other_robot->vel.linear;

    Eigen::Vector2d normal = p_ij.normalized();
    A.insert(i, 0) = normal(0);
    A.insert(i, 1) = normal(1);
    b(i) = d_min + normal.dot(v_ij);
  }

  // 制約の下限と上限
  Eigen::VectorXd lower_bound = Eigen::VectorXd::Constant(num_constraints, -OSQP_INFTY);
  Eigen::VectorXd upper_bound = b;

  // ソルバーを初期化し、解を求める
  OsqpSolver solver;
  solver.initialize(P, q, A, lower_bound, upper_bound);

  try {
    Eigen::VectorXd optimal_solution = solver.solve();
    return optimal_solution;
  } catch (const std::exception & e) {
    std::cerr << "最適化に失敗しました: " << e.what() << std::endl;
    return Eigen::Vector2d::Zero();  // エラー時はゼロ速度を返す
  }
}
}  // namespace crane

int main()
{
  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();
  using matplotlibcpp17::pyplot::PyPlot;

  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node("node");
  auto world_model = std::make_shared<crane::WorldModelWrapper>(node);

  crane_msgs::msg::WorldModel wm_msg;
  wm_msg.is_yellow = true;
  wm_msg.field_info.x = 9.0;
  wm_msg.field_info.y = 6.0;
  wm_msg.goal_size.x = 0.18;
  wm_msg.goal_size.y = 1.0;
  wm_msg.ball_info.pose.x = 0.;
  wm_msg.ball_info.pose.y = 0.;
  wm_msg.ball_info.pose.theta = 0.;

  auto get_robot_patch = [](auto robot_info, std::string color = "g") {
    return matplotlibcpp17::patches::Circle(
      Args(py::make_tuple(robot_info.pose.x, robot_info.pose.y), 0.1),
      Kwargs("fc"_a = color, "ec"_a = "black"));
  };

  auto ax = plt.axes();

  crane_msgs::msg::RobotInfoOurs our_robot;
  our_robot.disappeared = false;

  our_robot.id = 0;
  our_robot.pose.x = 1.0;
  our_robot.pose.y = 0.0;
  wm_msg.robot_info_ours.push_back(our_robot);
  ax.add_patch(Args(get_robot_patch(our_robot, "b").unwrap()));

  our_robot.id = 1;
  our_robot.pose.x = 1.5;
  our_robot.pose.y = 0.5;
  wm_msg.robot_info_ours.push_back(our_robot);
  ax.add_patch(Args(get_robot_patch(our_robot, "b").unwrap()));

  our_robot.id = 2;
  our_robot.pose.x = 1.5;
  our_robot.pose.y = -0.5;
  wm_msg.robot_info_ours.push_back(our_robot);
  ax.add_patch(Args(get_robot_patch(our_robot, "b").unwrap()));

  our_robot.id = 4;
  our_robot.pose.x = 2.0;
  our_robot.pose.y = 0.0;
  wm_msg.robot_info_ours.push_back(our_robot);
  ax.add_patch(Args(get_robot_patch(our_robot, "b").unwrap()));

  world_model->update(wm_msg);

  auto vel = crane::optimizeVelocity(
    world_model->getOurRobot(4), Point(-1.0, 0), world_model->ours.getAvailableRobots(4), 0.2);

  // auto vel_line = matplotlibcpp17::patches::Arrow(
  //   Args(py::make_tuple(1.0, 0.0), py::make_tuple(vel.x(), vel.y())));
  // ax.add_patch(vel_line.unwrap());

  plt.axis(Args("scaled"));
  ax.set_aspect(Args("equal"));
  /// user code
  // plt.plot(Args(std::vector<int>({1, 3, 2, 4})),
  //          Kwargs("color"_a = "blue", "linewidth"_a = 1.0));
  plt.show();

  return 0;
}
