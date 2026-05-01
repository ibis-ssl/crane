// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__SESSION_BASE_HPP_
#define CRANE_SESSIONS__SESSION_BASE_HPP_

#include <algorithm>
#include <crane_geometry/vector2d_adapter.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/game_analysis.hpp>
#include <crane_msgs/msg/play_situation.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_physics/position_assignments.hpp>
#include <crane_utils/stream.hpp>
#include <crane_visualization_interfaces/crane_visualizer_wrapper.hpp>
#include <functional>
#include <memory>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace crane
{
using SessionParameterType = std::variant<double, bool, int, std::string>;

struct RobotRole
{
  std::string session_name;
  std::string role_name;
  double score = 0.;
};

class SessionBase
{
public:
  using SharedPtr = std::shared_ptr<SessionBase>;

  using UniquePtr = std::unique_ptr<SessionBase>;

  enum class Status {
    SUCCESS,
    FAILURE,
    RUNNING,
  };

  /// ゴールキーパーをロボット選択から除外するためのコスト値
  static constexpr double GOALIE_EXCLUSION_COST = 1e9;

  /// 推奨位置が存在しない場合の高コスト値
  static constexpr double NO_SUITABLE_POSITION_COST = 1e9;

  explicit SessionBase(const std::string & name, WorldModelWrapper::SharedPtr & world_model)
  : name(name),
    world_model(world_model),
    visualizer(std::make_shared<VisualizerMessageBuilder>("session/" + name))
  {
    // セッション用ビジュアライザのデフォルトduration: 0.5秒
    visualizer->withDuration(0.5);
  }

  virtual ~SessionBase() { visualizer->clearBuffer(); }

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

  auto getPositionCommands() -> crane_msgs::msg::RobotCommands
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
        rclcpp::get_logger("SessionBase"),
        "RobotCommands from " << name << " session includes wrong robot_id : " << wrong_ids);
    }
    status = latest_status;
    crane_msgs::msg::RobotCommands msg;
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

  bool isSameConfiguration(SessionBase * other_session)
  {
    // 名前が異なる場合は別の設定
    if (name != other_session->name) {
      return false;
    }

    // どちらかのrobotsが空の場合は、名前のみで判定（ロボット割り当て前の比較用）
    if (robots.empty() || other_session->robots.empty()) {
      return true;
    }

    // 両方ともrobotsが設定されている場合は、ロボット構成も比較
    return robots.size() == other_session->robots.size() && [&]() {
      std::vector<RobotIdentifier> ours = this->robots;
      std::vector<RobotIdentifier> others = other_session->robots;
      std::ranges::sort(ours, [](const auto & a, const auto & b) -> bool { return a.id < b.id; });
      std::ranges::sort(others, [](const auto & a, const auto & b) -> bool { return a.id < b.id; });
      return ours == others;
    }();
  }

  Status getStatus() const { return status; }

  const std::vector<RobotIdentifier> & getRobots() const { return robots; }

  const std::string name;

  // セッションパラメータ管理
  void setSessionParameters(const std::unordered_map<std::string, SessionParameterType> & params)
  {
    session_params_ = params;
  }

  template <typename T>
  T getSessionParameter(const std::string & key, const T & default_value) const
  {
    auto it = session_params_.find(key);
    if (it != session_params_.end()) {
      if (auto * val = std::get_if<T>(&it->second)) {
        return *val;
      }
    }
    return default_value;
  }

  void setFixedRobots(const std::vector<uint8_t> & ids) { fixed_robots_ = ids; }
  const std::vector<uint8_t> & getFixedRobots() const { return fixed_robots_; }

  void setUseCandidateRobots(bool flag) { use_candidate_robots_ = flag; }
  bool usesCandidateRobots() const { return use_candidate_robots_; }

  void setCandidateRobots(const std::vector<uint8_t> & ids) { candidate_robots_ = ids; }
  const std::vector<uint8_t> & getCandidateRobots() const { return candidate_robots_; }

  virtual void onDeactivated(const crane_msgs::msg::PlaySituation &) {}

  /**
   * @brief YAML 由来の固定割当ロボットIDを設定する（RobotAllocator が呼ぶ）
   *
   * 設定されたID列は allocator の固定割当処理や Session 内部の補助ロジックから参照できる。
   */
  void setFixedRobots(const std::vector<uint8_t> & ids) { fixed_robots_ = ids; }

  /**
   * @brief 固定割当ロボットIDを取得する
   */
  const std::vector<uint8_t> & getFixedRobots() const { return fixed_robots_; }

  /**
   * @brief このセッションが「候補ロボットプール」を利用することを宣言する
   *
   * 派生クラス（例: AttackerHeatRotationSession）のコンストラクタで true を渡すことで、
   * 「このセッションは YAML の candidate_robots: [...] を Session 内のロジックで
   * 使う」ことを明示する。allocator は候補プールには関与しない。
   */
  void setUseCandidateRobots(bool flag) { use_candidate_robots_ = flag; }

  /**
   * @brief 候補ロボットプール利用モードかどうか
   */
  bool usesCandidateRobots() const { return use_candidate_robots_; }

  /**
   * @brief YAML 由来の候補ロボットIDプールを設定する（RobotAllocator が呼ぶ）
   *
   * Session 自身が suitability 関数等の中で参照する想定。allocator 自体は使わない。
   */
  void setCandidateRobots(const std::vector<uint8_t> & ids) { candidate_robots_ = ids; }

  /**
   * @brief 候補ロボットIDプールを取得する
   */
  const std::vector<uint8_t> & getCandidateRobots() const { return candidate_robots_; }

  /**
   * @brief ロボット適性評価関数を取得（GlobalRobotAllocator用）
   *
   * 各Sessionに適したロボットを評価するための関数を返す。
   * 返される関数は、ロボット情報を受け取り、適性コスト（小さいほど適している）を返す。
   *
   * @return ロボット適性評価関数
   */
  virtual auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> = 0;

  /**
   * @brief 現在の状況に基づく推奨ロボット数を返す（動的ロボット割当用）
   *
   * Sessionが状況に応じて必要なロボット数を動的に決定するためのメソッド。
   * 返される値は呼び出し側で0〜max_robotsの範囲にクランプされる。
   *
   * @param max_robots YAML設定の最大ロボット数
   * @return 推奨ロボット数。デフォルトはmax_robots
   */
  virtual int getDesiredRobotNumber(int max_robots) const { return max_robots; }

  /**
   * @brief セッションがアクティブ集合から外れる直前のフック
   *
   * 最新の PlaySituation を受け取り、セッション固有の終了処理に使う。
   * 例: AutoRef/Referee 信号に基づく履歴の確定。
   */
  virtual void onDeactivated(const crane_msgs::msg::PlaySituation &) {}

  /**
   * @brief GameAnalysisメッセージを設定する
   *
   * SessionCoordinatorから呼ばれ、game_analyzerが計算した分析結果を受け取る。
   *
   * @param game_analysis GameAnalysisメッセージ
   */
  void setGameAnalysis(const crane_msgs::msg::GameAnalysis & game_analysis)
  {
    game_analysis_ = game_analysis;
  }

  /**
   * @brief GameAnalysisメッセージを取得する
   *
   * @return GameAnalysisメッセージの参照
   */
  const crane_msgs::msg::GameAnalysis & getGameAnalysis() const { return game_analysis_; }

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
      [](std::shared_ptr<PositionCommandWrapper> &) {},
    double position_tolerance = -1.0) -> std::vector<crane_msgs::msg::RobotCommand>
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
    std::vector<crane_msgs::msg::RobotCommand> position_commands;
    for (auto robot_id = robots.begin(); robot_id != robots.end(); ++robot_id) {
      int index = std::distance(robots.begin(), robot_id);
      Point target_point = target_points[solution[index]];

      auto command =
        std::make_shared<PositionCommandWrapper>(command_name, robot_id->id, world_model);

      if (position_tolerance < 0.) {
        command->setTargetPosition(target_point);
      } else {
        command->setTargetPosition(target_point, position_tolerance);
      }
      command->setTargetTheta(getAngle(look_at_point - target_point));

      // カスタム設定を適用
      customize_command(command);

      position_commands.emplace_back(command->getMsg());
    }

    return position_commands;
  }

  std::vector<RobotIdentifier> robots;

  WorldModelWrapper::SharedPtr world_model;

  std::unordered_map<std::string, SessionParameterType> session_params_;

  bool use_candidate_robots_ = false;

  std::vector<uint8_t> fixed_robots_;

  std::vector<uint8_t> candidate_robots_;

  virtual std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) = 0;

  Status status = Status::RUNNING;

  VisualizerMessageBuilder::SharedPtr visualizer;

private:
  crane_msgs::msg::GameAnalysis game_analysis_;
  std::vector<uint8_t> fixed_robots_;
  std::vector<uint8_t> candidate_robots_;
  bool use_candidate_robots_ = false;
};

}  // namespace crane
#endif  // CRANE_SESSIONS__SESSION_BASE_HPP_
