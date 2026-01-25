// Copyright (c) 2025 ibis-ssl
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

#include <algorithm>
#include <chrono>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <sstream>
#include <vector>

using json = nlohmann::json;
namespace fs = std::filesystem;

class StateHistoryLogger : public rclcpp::Node
{
public:
  StateHistoryLogger() : Node("state_history_logger")
  {
    // パラメータ宣言
    this->declare_parameter("enabled", true);
    this->declare_parameter("output_dir", "~/.ros/log/state_history/");
    this->declare_parameter("buffer_size", 1000);
    this->declare_parameter("flush_interval_ms", 1000);
    this->declare_parameter("filter.robot_ids", std::vector<int64_t>{});
    this->declare_parameter("filter.planner_names", std::vector<std::string>{});
    this->declare_parameter("filter.skill_names", std::vector<std::string>{});

    // パラメータ取得
    enabled_ = this->get_parameter("enabled").as_bool();
    if (!enabled_) {
      RCLCPP_INFO(this->get_logger(), "StateHistoryLogger is disabled");
      return;
    }

    // 出力ディレクトリの設定
    std::string output_dir_param = this->get_parameter("output_dir").as_string();
    std::string output_dir = expandPath(output_dir_param);

    // ディレクトリ作成
    if (!fs::exists(output_dir)) {
      fs::create_directories(output_dir);
    }

    // ファイル名生成（タイムスタンプベース）
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << output_dir << "/state_history_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
       << ".jsonl";
    log_file_path_ = ss.str();

    // ファイルオープン
    log_file_.open(log_file_path_, std::ios::out | std::ios::trunc);
    if (!log_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", log_file_path_.c_str());
      enabled_ = false;
      return;
    }

    RCLCPP_INFO(
      this->get_logger(), "StateHistoryLogger started. Output: %s", log_file_path_.c_str());

    // フィルタパラメータ取得
    auto robot_ids_param = this->get_parameter("filter.robot_ids").as_integer_array();
    for (const auto & id : robot_ids_param) {
      filter_robot_ids_.insert(static_cast<uint8_t>(id));
    }

    filter_planner_names_ = this->get_parameter("filter.planner_names").as_string_array();
    filter_skill_names_ = this->get_parameter("filter.skill_names").as_string_array();

    // フィルタ情報をログ出力
    if (!filter_robot_ids_.empty()) {
      std::stringstream ids_ss;
      for (const auto & id : filter_robot_ids_) {
        ids_ss << static_cast<int>(id) << " ";
      }
      RCLCPP_INFO(this->get_logger(), "Filter robot IDs: %s", ids_ss.str().c_str());
    }
    if (!filter_planner_names_.empty()) {
      RCLCPP_INFO(
        this->get_logger(), "Filter planner names: %zu patterns", filter_planner_names_.size());
    }
    if (!filter_skill_names_.empty()) {
      RCLCPP_INFO(
        this->get_logger(), "Filter skill names: %zu patterns", filter_skill_names_.size());
    }

    // サブスクライバー作成
    robot_commands_sub_ = this->create_subscription<crane_msgs::msg::VelocityCommands>(
      "/robot_commands", 10,
      std::bind(&StateHistoryLogger::robotCommandsCallback, this, std::placeholders::_1));

    // WorldModel購読（ボール位置取得用）
    world_model_sub_ = this->create_subscription<crane_msgs::msg::WorldModel>(
      "/world_model", 10, [this](const crane_msgs::msg::WorldModel::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(world_model_mutex_);
        latest_world_model_ = *msg;
        world_model_received_ = true;
      });
  }

  ~StateHistoryLogger()
  {
    if (log_file_.is_open()) {
      log_file_.close();
      RCLCPP_INFO(
        this->get_logger(), "StateHistoryLogger stopped. Log saved to: %s", log_file_path_.c_str());
    }
  }

private:
  void robotCommandsCallback(const crane_msgs::msg::RobotCommands::SharedPtr msg)
  {
    if (!enabled_ || !log_file_.is_open()) {
      return;
    }

    // タイムスタンプ取得（UNIX時間、秒.ナノ秒）
    auto now = this->get_clock()->now();
    double timestamp = now.seconds();

    // ボール位置を取得（ロック内でコピー）
    double ball_x = 0.0, ball_y = 0.0;
    bool has_ball_info = false;
    {
      std::lock_guard<std::mutex> lock(world_model_mutex_);
      if (world_model_received_) {
        ball_x = latest_world_model_.ball_info.position.x;
        ball_y = latest_world_model_.ball_info.position.y;
        has_ball_info = true;
      }
    }

    for (const auto & cmd : msg->robot_commands) {
      // フィルタチェック
      if (!passesFilter(cmd)) {
        continue;
      }

      // JSON生成（可読性重視のフィールド名）
      json j;
      j["timestamp"] = timestamp;
      j["robot_id"] = cmd.robot_id;
      j["planner"] = cmd.planner_name;

      // states配列（フルネーム）
      json states_array = json::array();
      for (const auto & factor : cmd.planning_factors) {
        states_array.push_back({{"name", factor.name}, {"value", factor.value}});
      }
      j["states"] = states_array;

      // ロボット状態
      j["robot"]["pose"]["x"] = cmd.current_pose.x;
      j["robot"]["pose"]["y"] = cmd.current_pose.y;
      j["robot"]["pose"]["theta"] = cmd.current_pose.theta;
      j["robot"]["velocity"]["vx"] = cmd.current_velocity.x;
      j["robot"]["velocity"]["vy"] = cmd.current_velocity.y;
      j["robot"]["velocity"]["omega"] = cmd.current_velocity.theta;

      // 制御パラメータ（local planner後なのでcontrol_modeは常にPOLAR_VELOCITY_TARGET）
      j["control"]["kick_power"] = cmd.kick_power;
      j["control"]["dribble_power"] = cmd.dribble_power;
      j["control"]["chip_enable"] = cmd.chip_enable;
      j["control"]["stop_flag"] = cmd.stop_flag;

      // 目標速度（POLAR_VELOCITY_TARGET）
      if (!cmd.polar_velocity_target_mode.empty()) {
        j["target"]["velocity_r"] = cmd.polar_velocity_target_mode[0].target_velocity_r;
        j["target"]["velocity_theta"] = cmd.polar_velocity_target_mode[0].target_velocity_theta;
      }

      // ボール位置（WorldModelから）
      if (has_ball_info) {
        j["ball"]["x"] = ball_x;
        j["ball"]["y"] = ball_y;
      }

      // 1行として書き込み
      log_file_ << j.dump() << std::endl;
    }
  }

  bool passesFilter(const crane_msgs::msg::RobotCommand & cmd) const
  {
    // ロボットIDフィルタ
    if (!filter_robot_ids_.empty()) {
      if (filter_robot_ids_.find(cmd.robot_id) == filter_robot_ids_.end()) {
        return false;
      }
    }

    // プランナー名フィルタ
    if (!filter_planner_names_.empty()) {
      if (
        std::find(filter_planner_names_.begin(), filter_planner_names_.end(), cmd.planner_name) ==
        filter_planner_names_.end()) {
        return false;
      }
    }

    // スキル名フィルタ
    if (!filter_skill_names_.empty()) {
      bool found = false;
      for (const auto & factor : cmd.planning_factors) {
        if (
          std::find(filter_skill_names_.begin(), filter_skill_names_.end(), factor.name) !=
          filter_skill_names_.end()) {
          found = true;
          break;
        }
      }
      if (!found) {
        return false;
      }
    }

    return true;
  }

  std::string expandPath(const std::string & path) const
  {
    if (path.empty()) {
      return path;
    }

    if (path[0] == '~') {
      const char * home = std::getenv("HOME");
      if (home) {
        return std::string(home) + path.substr(1);
      }
    }

    return path;
  }

  bool enabled_;
  std::string log_file_path_;
  std::ofstream log_file_;

  // フィルタ設定
  std::set<uint8_t> filter_robot_ids_;
  std::vector<std::string> filter_planner_names_;
  std::vector<std::string> filter_skill_names_;

  // WorldModel関連
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr world_model_sub_;
  crane_msgs::msg::WorldModel latest_world_model_;
  std::mutex world_model_mutex_;
  bool world_model_received_ = false;

  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr robot_commands_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateHistoryLogger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
