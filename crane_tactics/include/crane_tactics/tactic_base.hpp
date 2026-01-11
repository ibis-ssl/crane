// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__TACTIC_BASE_HPP_
#define CRANE_TACTICS__TACTIC_BASE_HPP_

#include <algorithm>
#include <crane_geometry/vector2d_adapter.hpp>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/position_commands.hpp>
#include <crane_physics/position_assignments.hpp>
#include <crane_utils/stream.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace crane
{
using TacticParameterType = std::variant<double, bool, int, std::string>;

struct RobotRole
{
  std::string tactic_name;
  std::string role_name;
  double score = 0.;
};

class TacticBase
{
public:
  using SharedPtr = std::shared_ptr<TacticBase>;

  using UniquePtr = std::unique_ptr<TacticBase>;

  enum class Status {
    SUCCESS,
    FAILURE,
    RUNNING,
  };

  explicit TacticBase(const std::string & name, WorldModelWrapper::SharedPtr & world_model)
  : name(name),
    world_model(world_model),
    visualizer(std::make_shared<VisualizerMessageBuilder>("tactic_coordinator/" + name))
  {
  }

  virtual ~TacticBase() { visualizer->clearBuffer(); }

  /**
   * @brief GlobalRobotAllocatorが選択したロボットを直接設定する
   *
   * @param robot_ids 割り当てられたロボットIDリスト
   */
  void setAllocatedRobots(const std::vector<uint8_t> & robot_ids)
  {
    // ロボットIDが変更されたかチェック
    bool robots_changed =
      robots.size() != robot_ids.size() ||
      !std::ranges::equal(robots, robot_ids, [](const auto & r, uint8_t id) { return r.id == id; });

    robots.clear();
    for (auto id : robot_ids) {
      RobotIdentifier robot_id{true, id};
      robots.emplace_back(robot_id);
    }

    // 変更があれば通知
    if (robots_changed) {
      onRobotsChanged();
    }
  }

  auto getPositionCommands() -> crane_msgs::msg::PositionCommands
  {
    auto [latest_status, position_commands] = calculatePositionCommand(robots);
    auto wrong_ids =
      position_commands |
      // remove position_command.robot_id is included in robots
      ranges::views::filter([&](const auto & command) {
        return std::ranges::find_if(robots, [&](const auto & robot) {
                 return robot.id == command.robot_id;
               }) == robots.end();
      }) |
      ranges::views::transform([](const auto & command) { return command.robot_id; }) |
      ranges::to<std::vector>();
    if (not wrong_ids.empty()) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("TacticBase"),
        "PositionCommands from " << name << " tactic includes wrong robot_id : " << wrong_ids);
    }
    status = latest_status;
    crane_msgs::msg::PositionCommands msg;
    msg.is_yellow = world_model->isYellow();
    msg.on_positive_half = world_model->onPositiveHalf();
    for (auto command : position_commands) {
      // WorldModelから現在の速度情報を取得して設定
      auto robot = world_model->getOurRobot(command.robot_id);
      if (robot) {
        command.current_velocity.x = robot->vel.linear.x();
        command.current_velocity.y = robot->vel.linear.y();
        command.current_velocity.theta = robot->vel.omega;
      }
      msg.robot_commands.emplace_back(command);
    }
    visualizer->flush();
    return msg;
  }

  bool isSameConfiguration(TacticBase * other_tactic)
  {
    // 名前が異なる場合は別の設定
    if (name != other_tactic->name) {
      return false;
    }

    // どちらかのrobotsが空の場合は、名前のみで判定（ロボット割り当て前の比較用）
    if (robots.empty() || other_tactic->robots.empty()) {
      return true;
    }

    // 両方ともrobotsが設定されている場合は、ロボット構成も比較
    return robots.size() == other_tactic->robots.size() && [&]() {
      std::vector<RobotIdentifier> ours = this->robots;
      std::vector<RobotIdentifier> others = other_tactic->robots;
      std::ranges::sort(ours, [](const auto & a, const auto & b) -> bool { return a.id < b.id; });
      std::ranges::sort(others, [](const auto & a, const auto & b) -> bool { return a.id < b.id; });
      return ours == others;
    }();
  }

  Status getStatus() const { return status; }

  const std::vector<RobotIdentifier> & getRobots() const { return robots; }

  const std::string name;

  // セッションパラメータ管理
  void setTacticParameters(const std::unordered_map<std::string, TacticParameterType> & params)
  {
    tactic_params_ = params;
  }

  template <typename T>
  T getTacticParameter(const std::string & key, const T & default_value) const
  {
    auto it = tactic_params_.find(key);
    if (it != tactic_params_.end()) {
      if (auto * val = std::get_if<T>(&it->second)) {
        return *val;
      }
    }
    return default_value;
  }

  /**
   * @brief ロボット適性評価関数を取得（GlobalRobotAllocator用）
   *
   * 各Tacticに適したロボットを評価するための関数を返す。
   * 返される関数は、ロボット情報を受け取り、適性コスト（小さいほど適している）を返す。
   *
   * @return ロボット適性評価関数
   */
  virtual auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)>
  {
    // デフォルト実装: すべてのロボットを等価とみなす（コスト0）
    return [](const std::shared_ptr<RobotInfo> &) { return 0.0; };
  }

  /**
   * @brief ハード制約Tacticかどうかを返す
   *
   * trueを返すTacticは優先的にロボットが割り当てられる（キーパーなど）。
   *
   * @return ハード制約の場合true
   */
  virtual bool isHardConstraint() const { return false; }

  /**
   * @brief 現在の状況に基づく推奨ロボット数を返す（動的ロボット割当用）
   *
   * Tacticが状況に応じて必要なロボット数を動的に決定するためのメソッド。
   * 返される値は呼び出し側でmin_robots〜max_robotsの範囲にクランプされる。
   *
   * @param min_robots YAML設定の最小ロボット数
   * @param max_robots YAML設定の最大ロボット数
   * @return 推奨ロボット数。デフォルトはmax_robots
   */
  virtual int getDesiredRobotNumber(int min_robots, int max_robots) const { return max_robots; }

protected:
  /**
   * @brief ロボット割り当てが変更された時に呼ばれるコールバック
   *
   * サブクラスでオーバーライドして、ロボットID変更時の処理を実装する。
   * 例: スキルオブジェクトの再作成など
   */
  virtual void onRobotsChanged() {}
  // ロボットを最適な位置に割り当て、位置コマンドを生成する共通メソッド
  auto assignRobotsToPoints(
    const std::vector<RobotIdentifier> & robots, const std::vector<Point> & target_points,
    const std::string & command_name, const Point & look_at_point,
    const std::function<void(std::shared_ptr<PositionCommandWrapper> &)> & customize_command =
      [](std::shared_ptr<PositionCommandWrapper> &) {})
    -> std::vector<crane_msgs::msg::PositionCommand>
  {
    if (robots.empty() || target_points.empty()) {
      return {};
    }

    // ロボットの現在位置を収集
    std::vector<Point> robot_points;
    for (const auto & robot_id : robots) {
      robot_points.emplace_back(world_model->getRobot(robot_id)->pose.pos);
    }

    // 最適割り当てを計算
    auto solution = getOptimalAssignments(robot_points, target_points);

    // 各ロボットに位置コマンドを生成
    std::vector<crane_msgs::msg::PositionCommand> position_commands;
    for (auto robot_id = robots.begin(); robot_id != robots.end(); ++robot_id) {
      int index = std::distance(robots.begin(), robot_id);
      Point target_point = target_points[solution[index]];

      auto command =
        std::make_shared<PositionCommandWrapper>(command_name, robot_id->id, world_model);

      command->setTargetPosition(target_point);
      command->setTargetTheta(getAngle(look_at_point - target_point));

      // カスタム設定を適用
      customize_command(command);

      position_commands.emplace_back(command->getMsg());
    }

    return position_commands;
  }

  std::vector<RobotIdentifier> robots;

  WorldModelWrapper::SharedPtr world_model;

  std::unordered_map<std::string, TacticParameterType> tactic_params_;

  virtual std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) = 0;

  Status status = Status::RUNNING;

  VisualizerMessageBuilder::SharedPtr visualizer;
};

}  // namespace crane
#endif  // CRANE_TACTICS__TACTIC_BASE_HPP_
