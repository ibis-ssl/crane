// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <arpa/inet.h>
#include <ifaddrs.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_feedback.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <format>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

// ロボットの状態を表す列挙型
enum class RobotState {
  ACTIVE,    // アクティブで診断情報を発行すべき
  INACTIVE,  // 一時的に非アクティブ（フィールド外など）
  REMOVED    // 完全に削除された（診断情報から除外）
};

// ロボット情報を格納する構造体
struct RobotData
{
  // 基本情報
  uint8_t robot_id;
  RobotState state = RobotState::INACTIVE;
  rclcpp::Time last_update_time;

  // 診断用のコンポーネント
  std::unique_ptr<diagnostic_updater::Updater> updater;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr direct_publisher;

  // ハードウェア状態
  std::map<int, MotorData> motors;  // モーターID -> モーターデータ

  // バッテリー状態
  double battery_voltage = 12.8;
  double battery_percentage = 85.0;

  // 通信状態
  double comm_latency_ms = 8.5;
  int packet_loss_percent = 2;

  // センサー状態
  bool imu_ok = true;
  bool encoder_ok = true;

  // コンストラクタ
  RobotData(const std::string & id) : robot_id(id)
  {
    // モーターデータを初期化
    for (int i = 0; i < 4; i++) {
      motors[i] = MotorData{
        .temperature = 45.0 - (i % 4),  // 少しずつ異なる初期値を設定
        .current = 1.2 - (i * 0.1)};
    }
  }
};

// 動的なロボット管理に対応した診断ノード
class DiagnosticPublisherNode : public rclcpp::Node
{
public:
  DiagnosticPublisherNode() : Node("dynamic_multi_robot_diagnostic_node")
  {
    RCLCPP_INFO(get_logger(), "Starting dynamic multi-robot diagnostic node");

    // パラメータの宣言
    declare_parameter("max_robot_id", 12);  // サポートする最大ロボットID
    int max_robot_id = get_parameter("max_robot_id").as_int();

    // 最大ロボット数分の構造体を事前に準備

    for (int i = 0; i <= max_robot_id; ++i) {
      std::string robot_id = std::to_string(i);
      // 初期状態では空のRobotDataを作成
      robots_data.emplace_back(std::make_unique<RobotData>(robot_id));
    }

    world_model = std::make_unique<crane::WorldModelWrapper>(*this);

    world_model->addCallback([&]() {
      auto available_robot_ids = world_model->ours.getAvailableRobotIds();
      for (int id = 0; id < robots_data.size(); ++id) {
        auto & data = robots_data.at(id);

        // 状態を更新
        auto pre_state = std::exchange(
          data->state, available_robot_ids.find(id) != available_robot_ids.end()
                         ? RobotState::ACTIVE
                         : RobotState::INACTIVE);
        data->last_update_time = this->now();

        // 断情報を更新
        if (pre_state != data->state) {
          switch (data->state) {
            case RobotState::ACTIVE:
              RCLCPP_INFO(get_logger(), "Adding robot %d to diagnostics", id);

              // 診断アップデーターが存在しない場合は新規作成
              if (!data->updater) {
                initializeRobotDiagnostics(id);
              }

              break;
            case RobotState::INACTIVE:
              // ここで重要：このロボットの診断トピックに最終メッセージを送信してSTALEを防ぐ
              data->direct_publisher->publish(getClearDiagnostics(data));
              auto clear_msg = getClearDiagnostics(data);
              break;
            case RobotState::REMOVED:
              std::lock_guard<std::mutex> lock(mutex_);
              RCLCPP_INFO(get_logger(), "Removing robot %d from diagnostics", id);

              // ロボットのクリア診断メッセージを送信
              data->direct_publisher->publish(getClearDiagnostics(data));

              // アップデーターをリセット（メモリを解放）
              data->updater.reset();
              data->direct_publisher.reset();
              break;
          }
          data->updater->force_update();
        }
      }
    });

    // 診断更新タイマーの設定（アクティブなロボットのみ診断を更新）
    timer = create_wall_timer(
      std::chrono::milliseconds(500),  // 0.5秒ごとに更新
      [this]() { this->updateActiveDiagnostics(); });
  }

private:
  std::unique_ptr<crane::WorldModelWrapper> world_model;

  // ロボットデータの管理（ロボットID -> ロボットデータ）
  std::vector<std::unique_ptr<RobotData>> robots_data;

  // 診断更新用タイマー
  rclcpp::TimerBase::SharedPtr timer;

  // ロボット状態更新用サブスクライバー
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;

  // スレッドセーフな操作のためのミューテックス
  std::mutex mutex_;

  // 一つのロボットの診断システムを初期化する
  void initializeRobotDiagnostics(const uint8_t robot_id)
  {
    auto & robot = robots_data[robot_id];


    // 診断アップデーターの初期化
    // 重要: 診断トピックは /{team_color}/robot_{id}/diagnostics という形式になる
    std::string diagnostics_topic = "/" + team_color_ + "/robot_" + robot_id + "/diagnostics";

    robot->updater = std::make_unique<diagnostic_updater::Updater>(
      this,
      rclcpp::NodeOptions().arguments({"--ros-args", "-r", "~/diagnostics:=" + diagnostics_topic}));

    // 診断アップデーターの基本情報を設定
    robot->updater->setHardwareID("RoboCup_SSL_Robot_" + robot_id);

    // モーター診断用のタスクを追加
    for (int i = 0; i < 4; i++) {
      std::string motor_name = "motor_" + std::to_string(i);
      robot->updater->add(
        motor_name,
        [this, robot, i, motor_name](diagnostic_updater::DiagnosticStatusWrapper & stat) {
          this->checkMotor(robot, i, motor_name, stat);
        });
    }

    // バッテリー診断用のタスクを追加
    robot->updater->add(
      "battery", [this, robot](diagnostic_updater::DiagnosticStatusWrapper & stat) {
        this->checkBattery(robot, stat);
      });

    // 通信状態の診断用のタスクを追加
    robot->updater->add(
      "communication", [this, robot](diagnostic_updater::DiagnosticStatusWrapper & stat) {
        this->checkCommunication(robot, stat);
      });

    // センサー診断用のタスクを追加
    robot->updater->add(
      "sensors", [this, robot](diagnostic_updater::DiagnosticStatusWrapper & stat) {
        this->checkSensors(robot, stat);
      });

    // 直接パブリッシュするためのパブリッシャーも作成
    robot->direct_publisher =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic, 10);

    RCLCPP_INFO(get_logger(), "Initialized diagnostics for robot %d", robot->robot_id);
  }

  // アクティブなロボットのみ診断を更新
  void updateActiveDiagnostics()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // 必要に応じてここでハードウェアからの最新データを取得
    updateRobotsData();

    // アクティブなロボットの診断を更新
    for (auto & robot_data : robots_data) {
      if (robot_data->state == RobotState::ACTIVE && robot_data->updater) {
        robot_data->updater->force_update();
        robot_data->last_update_time = now();
      }
    }
  }

  // ロボットデータの更新（実際の実装ではハードウェアから取得）
  void updateRobotsData()
  {
    // アクティブなロボットのみデータを更新
    for (auto & robot_data : robots_data) {
      if (robot_data->state == RobotState::ACTIVE) {
        // モーター温度の更新（簡易的なシミュレーション）
        for (auto & [motor_id, motor_data] : robot_data->motors) {
          motor_data.temperature += (rand() % 100 - 50) / 100.0;
          motor_data.current += (rand() % 100 - 50) / 200.0;
        }

        // その他のデータ更新...
        // 実際の実装ではハードウェアやシミュレーションからのデータ取得に置き換える
      }
    }
  }

  // モーター診断関数
  void checkMotor(
    const std::unique_ptr<RobotData> & robot, int motor_id, const std::string & motor_name,
    diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    if (robot->state != RobotState::ACTIVE) {
      // 非アクティブなロボットの場合はOKとして報告
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Robot inactive");
      return;
    }

    double temp = robot->motors[motor_id].temperature;
    double current = robot->motors[motor_id].current;

    if (temp > 70.0) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Motor temperature too high");
    } else if (temp > 60.0) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Motor temperature high");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Motor operating normally");
    }

    stat.add("temperature", temp);
    stat.add("current", current);
    stat.add("robot_id", robot->robot_id);
  }

  // バッテリー診断関数
  void checkBattery(
    const std::unique_ptr<RobotData> & robot, diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    if (robot->state != RobotState::ACTIVE) {
      // 非アクティブなロボットの場合はOKとして報告
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Robot inactive");
      return;
    }

    double percentage = robot->battery_percentage;

    if (percentage < 20.0) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Battery critically low");
    } else if (percentage < 30.0) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Battery low");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Battery OK");
    }

    stat.add("voltage", robot->battery_voltage);
    stat.add("percentage", percentage);
    stat.add("robot_id", robot->robot_id);
  }

  // 通信診断関数
  void checkCommunication(
    const std::unique_ptr<RobotData> & robot, diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    if (robot->state != RobotState::ACTIVE) {
      // 非アクティブなロボットの場合はOKとして報告
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Robot inactive");
      return;
    }

    int packet_loss = robot->packet_loss_percent;
    double latency = robot->comm_latency_ms;

    if (packet_loss > 10) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "High packet loss");
    } else if (latency > 20.0) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Communication latency high");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Communication OK");
    }

    stat.add("latency_ms", latency);
    stat.add("packet_loss_percent", packet_loss);
    stat.add("robot_id", robot->robot_id);
  }

  // センサー診断関数
  void checkSensors(
    const std::unique_ptr<RobotData> & robot, diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    if (robot->state != RobotState::ACTIVE) {
      // 非アクティブなロボットの場合はOKとして報告
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Robot inactive");
      return;
    }

    bool imu_status = robot->imu_ok;
    bool encoder_status = robot->encoder_ok;

    if (!imu_status || !encoder_status) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Sensor failure detected");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "All sensors OK");
    }

    stat.add("imu_ok", imu_status);
    stat.add("encoder_ok", encoder_status);
    stat.add("robot_id", robot->robot_id);
  }

  // ロボットが非アクティブになったときにSTALEを防ぐためのクリア診断メッセージを送信
  auto getClearDiagnostics(const std::unique_ptr<RobotData> & robot)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "summary";
    status.hardware_id = "RoboCup_SSL_Robot_" + std::to_string(robot->robot_id);
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Robot inactive";

    status.values.push_back(diagnostic_msgs::msg::KeyValue());
    status.values.back().key = "robot_id";
    status.values.back().value = robot->robot_id;

    status.values.push_back(diagnostic_msgs::msg::KeyValue());
    status.values.back().key = "state";
    status.values.back().value = "inactive";

    array.status.push_back(status);

    // モーター状態のクリアメッセージも追加
    for (int i = 0; i < 4; i++) {
      diagnostic_msgs::msg::DiagnosticStatus motor_status;
      motor_status.name = "motor_" + std::to_string(i);
      motor_status.hardware_id = "RoboCup_SSL_Robot_" + std::to_string(robot->robot_id)
      motor_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      motor_status.message = "Robot inactive";

      motor_status.values.push_back(diagnostic_msgs::msg::KeyValue());
      motor_status.values.back().key = "robot_id";
      motor_status.values.back().value = robot->robot_id;

      array.status.push_back(motor_status);
    }

    // バッテリー状態のクリアメッセージ
    diagnostic_msgs::msg::DiagnosticStatus battery_status;
    battery_status.name = "battery";
    battery_status.hardware_id = "RoboCup_SSL_Robot_" + std::to_string(robot->robot_id);
    battery_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    battery_status.message = "Robot inactive";

    battery_status.values.push_back(diagnostic_msgs::msg::KeyValue());
    battery_status.values.back().key = "robot_id";
    battery_status.values.back().value = robot->robot_id;

    array.status.push_back(battery_status);
    return array;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;
  auto node = std::make_shared<RobotReceiverNode>();
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
