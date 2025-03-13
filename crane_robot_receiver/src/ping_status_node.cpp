// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <arpa/inet.h>
#include <ifaddrs.h>

#include <array>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msgs/msg/ping_status_array.hpp>
#include <crane_msgs/msg/robot_feedback.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <cstdlib>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <format>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>

auto getRobotIP(uint8_t id) -> std::string { return std::format("192.168.20.{}", 100 + id); }

class PingNode : public rclcpp::Node
{
public:
  PingNode() : Node("ping_node")
  {
    // 各ロボットのIP情報を初期化
    for (int i = 0; i < 11; ++i) {
      ping_statuses[i] = std::make_pair(getRobotIP(i), 0.0);

      // 各ロボットごとにdiagnostic_updaterを初期化
      auto updater = std::make_unique<diagnostic_updater::Updater>(this);
      updater->setHardwareIDf("Robot%i/CM4", i);

      // 診断タスクを追加
      auto task = std::bind(&PingNode::checkPingStatus, this, std::placeholders::_1, i);
      updater->add(std::format("Robot{} Communication", i), task);

      diagnostic_updaters.push_back(std::move(updater));
    }

    timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PingNode::pingHosts, this));
  }

private:
  // 診断情報を更新するコールバック関数
  void checkPingStatus(diagnostic_updater::DiagnosticStatusWrapper & stat, int robot_id)
  {
    const auto & [ip, ping_time] = ping_statuses[robot_id];

    if (ping_time < 0) {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        std::format("Robot {} ({}) is not responding", robot_id, ip));
    } else if (ping_time > 5.0) {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        std::format("Robot {} ({}) has high latency: {:.2f}ms", robot_id, ip, ping_time));
    } else {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        std::format("Robot {} ({}) communication is normal", robot_id, ip));
    }

    stat.add("Ping (ms)", ping_time);
    stat.add("IP Address", ip);
    stat.add("Last Check", this->now().seconds());
  }

  void pingHosts()
  {
    // 各ロボットにpingを送信して結果を処理
    for (int id = 0; id < 11; ++id) {
      std::string ip = ping_statuses[id].first;
      std::string command = "ping -c 1 -W 1 " + ip + " | grep 'time='";
      std::array<char, 128> buffer;
      std::string result;

      // pingコマンドを実行
      std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
      if (!pipe) {
        RCLCPP_ERROR(this->get_logger(), "Failed to run ping command");
        ping_statuses[id].second = -1.0;  // エラーを示す値
        continue;
      }

      while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
      }

      // 結果を解析
      if (!result.empty()) {
        double ping_ms = std::stod(
          result.substr(result.find("time=") + 5, result.find("ms") - result.find("time=") - 5));

        ping_statuses[id].second = ping_ms;
        RCLCPP_DEBUG(this->get_logger(), "Robot %d ping: %.2f ms", id, ping_ms);
      } else {
        // pingが失敗した場合
        ping_statuses[id].second = -1.0;
        RCLCPP_WARN(this->get_logger(), "Robot %d (%s) not responding", id, ip.c_str());
      }

      // 診断情報を更新
      diagnostic_updaters[id]->force_update();
      diagnostic_updaters[id]->
    }
  }

  rclcpp::TimerBase::SharedPtr timer;

  // ロボットIDに対応するIP情報とping時間を保持
  std::unordered_map<int, std::pair<std::string, double>> ping_statuses;

  // 各ロボット用の診断情報更新クラス
  std::vector<std::unique_ptr<diagnostic_updater::Updater>> diagnostic_updaters;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;
  auto node = std::make_shared<PingNode>();
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
