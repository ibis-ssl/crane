// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PHYSICS__SLACK_TIME_CONFIG_HPP_
#define CRANE_PHYSICS__SLACK_TIME_CONFIG_HPP_

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>

namespace crane
{

/**
 * @brief slack時間計算の設定を一元管理する構造体
 *
 * この構造体は、ロボットがボールに到達できるか（slack時間）を計算する際に
 * 使用される全てのパラメータを保持します。
 *
 * slack時間の定義:
 *   slack時間 = ボールがある地点に到達する時間 - ロボットがその地点に到達する時間
 *   - 正の値: ロボットが間に合う（余裕がある）
 *   - 負の値: ロボットが間に合わない
 */
struct SlackTimeConfig
{
  // ===== ロボット物理パラメータ =====

  /// @brief ロボットの最大加速度 [m/s^2]
  /// @details slack時間計算で仮定する台形速度プロファイルの加速度
  double robot_max_acceleration = 2.5;

  /// @brief ロボットの最大速度 [m/s]
  /// @details slack時間計算で仮定する台形速度プロファイルの最大速度
  double robot_max_velocity = 5.0;

  // ===== slack時間計算パラメータ =====

  /// @brief 予測時間範囲 [s]
  /// @details ボール軌道をどこまで先まで予測するか
  double time_horizon = 3.0;

  /// @brief 時間刻み [s]
  /// @details ボール軌道をサンプリングする時間間隔
  double time_step = 0.1;

  /// @brief slack時間オフセット [s]
  /// @details 計算されたslack時間に加算する安全マージン
  /// 正の値を設定すると、より早めに動き始める
  double slack_time_offset = 0.5;

  /// @brief 距離ホライズン [m]
  /// @details この距離以内のボール軌道のみを評価対象とする
  double distance_horizon = 100.0;

  /// @brief 速度判定閾値 [m/s]
  /// @details ボール速度がこの値以下の場合、静止していると判定
  double velocity_epsilon = 0.2;

  /**
   * @brief ROS2ノードからパラメータを読み込む
   *
   * @param node ROS2ノード
   * @param prefix パラメータ名のプレフィックス（例: "slack."）
   * @return SlackTimeConfig 読み込んだ設定
   *
   * @details パラメータが存在しない場合はデフォルト値を使用します。
   *
   * 想定されるパラメータ名:
   * - {prefix}robot_max_acceleration
   * - {prefix}robot_max_velocity
   * - {prefix}time_horizon
   * - {prefix}time_step
   * - {prefix}slack_time_offset
   * - {prefix}distance_horizon
   * - {prefix}velocity_epsilon
   */
  static SlackTimeConfig fromNode(rclcpp::Node & node, const std::string & prefix = "slack.")
  {
    SlackTimeConfig config;

    config.robot_max_acceleration =
      node.get_parameter_or(prefix + "robot_max_acceleration", config.robot_max_acceleration);
    config.robot_max_velocity =
      node.get_parameter_or(prefix + "robot_max_velocity", config.robot_max_velocity);
    config.time_horizon =
      node.get_parameter_or(prefix + "time_horizon", config.time_horizon);
    config.time_step =
      node.get_parameter_or(prefix + "time_step", config.time_step);
    config.slack_time_offset =
      node.get_parameter_or(prefix + "slack_time_offset", config.slack_time_offset);
    config.distance_horizon =
      node.get_parameter_or(prefix + "distance_horizon", config.distance_horizon);
    config.velocity_epsilon =
      node.get_parameter_or(prefix + "velocity_epsilon", config.velocity_epsilon);

    return config;
  }

  /**
   * @brief ROS2ノードにパラメータを宣言
   *
   * @param node ROS2ノード
   * @param prefix パラメータ名のプレフィックス（例: "slack."）
   *
   * @details declare_parameter を使用してパラメータをノードに登録します。
   * これにより、launch ファイルからパラメータを設定可能になります。
   */
  void declareParameters(rclcpp::Node & node, const std::string & prefix = "slack.") const
  {
    node.declare_parameter(prefix + "robot_max_acceleration", robot_max_acceleration);
    node.declare_parameter(prefix + "robot_max_velocity", robot_max_velocity);
    node.declare_parameter(prefix + "time_horizon", time_horizon);
    node.declare_parameter(prefix + "time_step", time_step);
    node.declare_parameter(prefix + "slack_time_offset", slack_time_offset);
    node.declare_parameter(prefix + "distance_horizon", distance_horizon);
    node.declare_parameter(prefix + "velocity_epsilon", velocity_epsilon);
  }

  /**
   * @brief 設定内容をデバッグ出力
   *
   * @param os 出力ストリーム
   */
  void print(std::ostream & os = std::cout) const
  {
    os << "[SlackTimeConfig]\n";
    os << "  robot_max_acceleration: " << robot_max_acceleration << " m/s^2\n";
    os << "  robot_max_velocity: " << robot_max_velocity << " m/s\n";
    os << "  time_horizon: " << time_horizon << " s\n";
    os << "  time_step: " << time_step << " s\n";
    os << "  slack_time_offset: " << slack_time_offset << " s\n";
    os << "  distance_horizon: " << distance_horizon << " m\n";
    os << "  velocity_epsilon: " << velocity_epsilon << " m/s\n";
  }
};

}  // namespace crane

#endif  // CRANE_PHYSICS__SLACK_TIME_CONFIG_HPP_
