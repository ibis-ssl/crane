// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cmath>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/ping_status_array.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <cstring>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <map>

namespace crane
{
// POWERエラー定義
enum PowerErrorCode {
  POWER_NONE = 0,
  POWER_UNDER_VOLTAGE = 0x0001,
  POWER_OVER_VOLTAGE = 0x0002,
  POWER_OVER_CURRENT = 0x0004,
  POWER_SHORT_CIRCUIT = 0x0008,
  POWER_CHARGE_TIME = 0x0010,
  POWER_CHARGE_POWER = 0x0020,
  POWER_DISCHARGE = 0x0040,
  POWER_PARAMETER = 0x0080,
  POWER_COMMAND = 0x0100,
  POWER_NO_CAP = 0x0200,
  POWER_DISCHARGE_FAIL = 0x0400,
  POWER_GD_POWER_FAIL = 0x0800,
  POWER_COIL_OVER_HEAT = 0x1000,
  POWER_FET_OVER_HEAT = 0x2000
};

// BLDCエラー定義
enum BldcErrorCode {
  BLDC_NONE = 0,
  BLDC_UNDER_VOLTAGE = 0x0001,
  BLDC_OVER_CURRENT = 0x0002,
  BLDC_MOTOR_OVER_HEAT = 0x0004,
  BLDC_OVER_LOAD = 0x0008,
  BLDC_ENC_ERROR = 0x0010,
  BLDC_OVER_VOLTAGE = 0x0020,
  BLDC_FET_OVER_HEAT = 0x0040
};

// エラー情報をテキストに変換する関数
std::string convertErrorDataToStr(uint16_t id, uint16_t info)
{
  std::string result;

  if (id == 100) {  // POWERエラー
    result = "POWER : ";
    switch (info) {
      case POWER_NONE:
        result += "no error";
        break;
      case POWER_UNDER_VOLTAGE:
        result += "UNDER_VOLTAGE";
        break;
      case POWER_OVER_VOLTAGE:
        result += "OVER_VOLTAGE";
        break;
      case POWER_OVER_CURRENT:
        result += "OVER_CURRENT";
        break;
      case POWER_SHORT_CIRCUIT:
        result += "SHORT_CIRCUIT";
        break;
      case POWER_CHARGE_TIME:
        result += "CHARGE_TIME";
        break;
      case POWER_CHARGE_POWER:
        result += "CHARGE_POWER";
        break;
      case POWER_DISCHARGE:
        result += "DISCHARGE";
        break;
      case POWER_PARAMETER:
        result += "PARAMETER";
        break;
      case POWER_COMMAND:
        result += "COMMAND";
        break;
      case POWER_NO_CAP:
        result += "NO_CAP";
        break;
      case POWER_DISCHARGE_FAIL:
        result += "DISCHARGE_FAIL";
        break;
      case POWER_GD_POWER_FAIL:
        result += "GD_POWER_FAIL";
        break;
      case POWER_COIL_OVER_HEAT:
        result += "COIL_OVER_HEAT";
        break;
      case POWER_FET_OVER_HEAT:
        result += "FET_OVER_HEAT";
        break;
      default:
        result += "unknown info " + std::to_string(info);
        break;
    }
  } else {  // BLDCエラー
    if (id == 0) {
      result = "BLDC-RF : ";
    } else if (id == 1) {
      result = "BLDC-RB : ";
    } else if (id == 2) {
      result = "BLDC-LB : ";
    } else if (id == 3) {
      result = "BLDC-LF : ";
    } else {
      return "BLDC unknown id : " + std::to_string(id) + " info : " + std::to_string(info);
    }

    switch (info) {
      case BLDC_NONE:
        result += "no error";
        break;
      case BLDC_UNDER_VOLTAGE:
        result += "UNDER_VOLTAGE";
        break;
      case BLDC_OVER_CURRENT:
        result += "OVER_CURRENT";
        break;
      case BLDC_MOTOR_OVER_HEAT:
        result += "MOTOR_OVER_HEAT";
        break;
      case BLDC_OVER_LOAD:
        result += "OVER_LOAD";
        break;
      case BLDC_ENC_ERROR:
        result += "ENC_ERROR";
        break;
      case BLDC_OVER_VOLTAGE:
        result += "OVER_VOLTAGE";
        break;
      case BLDC_FET_OVER_HEAT:
        result += "FET_OVER_HEAT";
        break;
      default:
        result += "unknown info " + std::to_string(info);
        break;
    }
  }

  return result;
}

// temperature配列のインデックスから名前への変換
std::string getTemperatureLabel(int index)
{
  switch (index) {
    case 0:
      return "Motor 1";
    case 1:
      return "Motor 2";
    case 2:
      return "Motor 3";
    case 3:
      return "Motor 4";
    case 4:
      return "FET";
    case 5:
      return "Coil 1";
    case 6:
      return "Coil 2";
    default:
      return "Unknown (" + std::to_string(index) + ")";
  }
}

enum class RobotState {
  ACTIVE,    // アクティブで診断情報を発行すべき
  INACTIVE,  // 一時的に非アクティブ（フィールド外など）
};

// エラー情報の構造体
struct ErrorInfo {
  std::string type;        // エラータイプ (robot_error, communication, battery など)
  std::string message;     // エラーメッセージ
  int level;               // エラーレベル (0: OK, 1: WARN, 2: ERROR, 3: STALE)
  rclcpp::Time timestamp;  // エラーが発生した時刻
};

struct RobotData
{
  uint8_t robot_id;

  RobotState state = RobotState::INACTIVE;

  rclcpp::Time last_update_time;

  std::unique_ptr<diagnostic_updater::Updater> updater;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr direct_publisher;

  // エラー情報を保存するマップ (エラータイプ => エラー情報)
  std::map<std::string, ErrorInfo> error_map;

  // 前回のエラー状態と比較して変化があるかをチェックするフラグ
  bool has_error_changed = false;

  explicit RobotData(const uint8_t & id) : robot_id(id) {}
};

class DiagnosticPublisherNode : public rclcpp::Node
{
public:
  DiagnosticPublisherNode() : Node("diagnostic_publisher_node")
  {
    declare_parameter("max_robot_id", 12);  // サポートする最大ロボットID
    int max_robot_id = get_parameter("max_robot_id").as_int();

    // 可視化用のCraneVisualizerBufferを初期化
    CraneVisualizerBuffer::activate(*this);
    visualizer_error = std::make_shared<VisualizerMessageBuilder>("diagnostic_errors");

    // 最大ロボット数分の構造体を事前に準備
    for (int i = 0; i <= max_robot_id; ++i) {
      // 初期状態では空のRobotDataを作成
      robots_data.emplace_back(std::make_shared<RobotData>(i));
      initializeRobotDiagnostics(i);
      // ロボット位置の初期化
      robot_positions[i] = {0.0, 0.0, 0.0, false};
    }

    ping_subscription = create_subscription<crane_msgs::msg::PingStatusArray>(
      "/ping", 10, [&](const crane_msgs::msg::PingStatusArray & msg) { latest_ping_msg = msg; });

    feedback_subscription = create_subscription<crane_msgs::msg::RobotFeedbackArray>(
      "/robot_feedback", 10,
      [&](const crane_msgs::msg::RobotFeedbackArray & msg) { latest_feedback_msg = msg; });

    world_model = std::make_unique<WorldModelWrapper>(*this);
    world_model->addCallback([&]() {
      auto available_robot_ids = world_model->ours.getAvailableRobotIds();
      
      // ロボットの位置情報を更新
      for (const auto& robot : world_model->ours.getAvailableRobots()) {
        robot_positions[robot->id] = {
          robot->pose.pos.x(),
          robot->pose.pos.y(),
          robot->pose.theta,
          true
        };
      }
      
      for (int id = 0; id < robots_data.size(); ++id) {
        auto & data = robots_data.at(id);
        // 状態を更新
        data->state =
          ranges::contains(available_robot_ids, id) ? RobotState::ACTIVE : RobotState::INACTIVE;
      }
    });
    
    // 診断情報更新タイマー
    timer = this->create_wall_timer(std::chrono::seconds(1), [&]() {
      for (auto & robot_data : robots_data) {
        if (robot_data->updater) {
          robot_data->updater->force_update();
          robot_data->last_update_time = now();
        }
      }
    });
    
    // 可視化用タイマー - より高頻度で更新
    visualization_timer = this->create_wall_timer(std::chrono::milliseconds(100), [&]() {
      visualizeRobotErrors();
    });
  }
  
  // ロボットエラーの可視化を行う関数
  void visualizeRobotErrors()
  {
    visualizer_error->clear();
    
    // 現在のタイムスタンプ
    rclcpp::Time now = this->now();
    
    // 画面上部にエラーまとめの表示用変数
    int error_count = 0;
    int error_display_offset = 0;

    std::array<double, 20> offset = {};
    
    // 各ロボットのエラー情報を可視化
    for (const auto& robot_data : robots_data) {
      if (robot_data->state != RobotState::ACTIVE) {
        continue;  // 非アクティブなロボットはスキップ
      }
      
      // エラーがあるかチェック
      bool has_errors = false;
      for (const auto& [error_type, error_info] : robot_data->error_map) {
        // エラーと警告のみを表示 (0: OK, 1: WARN, 2: ERROR, 3: STALE)
        if (error_info.level > 0) {
          has_errors = true;
          
          // 10秒以上経過したエラーは表示しない
          if ((now - error_info.timestamp).seconds() > 10.0) {
            continue;
          }
          
          // ロボットIDを取得
          uint8_t robot_id = robot_data->robot_id;
          
          // エラーレベルに応じた色を設定
          std::string color;
          double opacity = 0.8;
          
          switch (error_info.level) {
            case 1:  // WARN
              color = "yellow";
              break;
            case 2:  // ERROR
              color = "red";
              break;
            case 3:  // STALE
              color = "grey";
              break;
            default:
              color = "white";
          }
          
          // ロボット位置が有効な場合、エラーマーカーをロボット位置に表示
          if (robot_positions.count(robot_id) > 0 && robot_positions[robot_id].valid) {
            // ロボット周囲に円形マーカーを表示
            visualizer_error->circle()
              .center(robot_positions[robot_id].x, robot_positions[robot_id].y)
              .radius(0.15)  // ロボットより少し大きい円
              .stroke(color, opacity)
              .strokeWidth(2.0)
              .fill("none")
              .build();

            // ロボットIDとエラーメッセージを表示
            visualizer_error->text()
              .position(robot_positions[robot_id].x, robot_positions[robot_id].y + offset[robot_id] + 0.15)
              .text(error_info.message)
              .fill(color, opacity)
              .fontSize(50.0)
              .textAnchor("middle")
              .build();
            offset[robot_id] += 0.08;
          }
        }
      }
    }
    
    // 可視化情報を送信
    visualizer_error->flush();
    CraneVisualizerBuffer::publish();
  }

private:
  WorldModelWrapper::UniquePtr world_model;

  std::vector<std::shared_ptr<RobotData>> robots_data;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::TimerBase::SharedPtr visualization_timer; // 可視化用のタイマー

  rclcpp::Subscription<crane_msgs::msg::PingStatusArray>::SharedPtr ping_subscription;

  crane_msgs::msg::PingStatusArray latest_ping_msg;

  rclcpp::Subscription<crane_msgs::msg::RobotFeedbackArray>::SharedPtr feedback_subscription;

  crane_msgs::msg::RobotFeedbackArray latest_feedback_msg;
  
  // 可視化ラッパー
  std::shared_ptr<VisualizerMessageBuilder> visualizer_error;
  
  // エラーが発生したロボットの位置情報を保持
  struct RobotPosition {
    double x;
    double y;
    double theta;
    bool valid;
  };
  std::map<uint8_t, RobotPosition> robot_positions;

  void initializeRobotDiagnostics(const uint8_t robot_id)
  {
    auto & data = robots_data.at(robot_id);

    data->updater = std::make_unique<diagnostic_updater::Updater>(this);
    data->updater->setHardwareID("robot_" + std::to_string(robot_id));
    data->updater->add(
      "communication",
      [this, robot_id = data->robot_id](diagnostic_updater::DiagnosticStatusWrapper & stat) {
        auto & data = robots_data.at(robot_id);
        if (data->state != RobotState::ACTIVE) {
          // 非アクティブなロボットの場合はOKとして報告
          stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Robot inactive");
          
          // エラーマップから削除
          if (data->error_map.count("communication") > 0) {
            data->error_map.erase("communication");
            data->has_error_changed = true;
          }
          return;
        }

        if (auto ping = ranges::find_if(
              latest_ping_msg.ping, [&](const auto msg) { return msg.robot_id == data->robot_id; });
            ping != latest_ping_msg.ping.end()) {
          std::string message;
          int level;
          
          if (ping->ping_ms > 50.0) {
            message = "Ping time high";
            level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
          } else if (ping->ping_ms > 10.0) {
            message = "Communication latency medium";
            level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
          } else {
            message = "Communication OK";
            level = diagnostic_msgs::msg::DiagnosticStatus::OK;
          }
          
          stat.summary(level, message);
          stat.add("ping_ms", ping->ping_ms);
          
          // エラーマップに登録または更新
          if (level > 0) {
            bool is_new = (data->error_map.count("communication") == 0) || 
                         (data->error_map["communication"].level != level) ||
                         (data->error_map["communication"].message != message);
            
            if (is_new) {
              data->error_map["communication"] = {
                "communication",  // タイプ
                message,          // メッセージ
                level,            // レベル
                now()             // タイムスタンプ
              };
              data->has_error_changed = true;
            }
          } else if (data->error_map.count("communication") > 0) {
            // エラーが解消された場合はマップから削除
            data->error_map.erase("communication");
            data->has_error_changed = true;
          }
        } else {
          std::string message = "No ping data received";
          int level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
          
          stat.summary(level, message);
          
          // エラーマップに登録
          bool is_new = (data->error_map.count("communication") == 0) || 
                       (data->error_map["communication"].message != message);
          
          if (is_new) {
            data->error_map["communication"] = {
              "communication",  // タイプ
              message,          // メッセージ
              level,            // レベル
              now()             // タイムスタンプ
            };
            data->has_error_changed = true;
          }
        }
      });

    data->updater->add(
      "battery",
      [this, robot_id = data->robot_id](diagnostic_updater::DiagnosticStatusWrapper & stat) {
        auto & data = robots_data.at(robot_id);
        if (data->state != RobotState::ACTIVE) {
          stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Robot inactive");
          
          // エラーマップから削除
          if (data->error_map.count("battery") > 0) {
            data->error_map.erase("battery");
            data->has_error_changed = true;
          }
        } else {
          if (auto feedback = ranges::find_if(
                latest_feedback_msg.feedback,
                [&](const auto & msg) { return msg.robot_id == data->robot_id; });
              feedback != latest_feedback_msg.feedback.end()) {
            std::string message;
            int level;
            
            if (feedback->voltage[0] < 22.0) {
              message = "Low battery voltage";
              level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            } else if (feedback->voltage[0] < 23.0) {
              message = "Battery voltage medium";
              level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            } else {
              message = "Battery voltage high";
              level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            }
            
            stat.summary(level, message);
            stat.add("voltage", feedback->voltage[0]);
            
            // エラーマップに登録または更新
            if (level > 0) {
              bool is_new = (data->error_map.count("battery") == 0) || 
                           (data->error_map["battery"].level != level) ||
                           (data->error_map["battery"].message != message);
              
              if (is_new) {
                data->error_map["battery"] = {
                  "battery",      // タイプ
                  message,        // メッセージ
                  level,          // レベル
                  now()           // タイムスタンプ
                };
                data->has_error_changed = true;
              }
            } else if (data->error_map.count("battery") > 0) {
              // エラーが解消された場合はマップから削除
              data->error_map.erase("battery");
              data->has_error_changed = true;
            }
          } else {
            std::string message = "No robot feedback received";
            int level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            
            stat.summary(level, message);
            
            // エラーマップに登録
            bool is_new = (data->error_map.count("battery") == 0) || 
                         (data->error_map["battery"].message != message);
            
            if (is_new) {
              data->error_map["battery"] = {
                "battery",      // タイプ
                message,        // メッセージ
                level,          // レベル
                now()           // タイムスタンプ
              };
              data->has_error_changed = true;
            }
          }
        }
      });

    // ロボットエラー表示の改良（より詳細なエラー情報を表示）
    data->updater->add(
      "robot_error",
      [this, robot_id = data->robot_id](diagnostic_updater::DiagnosticStatusWrapper & stat) {
        auto & data = robots_data.at(robot_id);
        if (data->state != RobotState::ACTIVE) {
          stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Robot inactive");
          
          // エラーマップから削除
          if (data->error_map.count("robot_error") > 0) {
            data->error_map.erase("robot_error");
            data->has_error_changed = true;
          }
        } else {
          if (auto feedback = ranges::find_if(
                latest_feedback_msg.feedback,
                [&](const auto & msg) { return msg.robot_id == data->robot_id; });
              feedback != latest_feedback_msg.feedback.end()) {
            if (feedback->error_id != 0 || feedback->error_info != 0) {
              // エラー情報を人間が読める形式に変換して表示
              std::string error_str =
                convertErrorDataToStr(feedback->error_id, feedback->error_info);
              stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error_str);

              stat.add("error_id", feedback->error_id);
              stat.add("error_info", feedback->error_info);
              stat.add("error_value", feedback->error_value);
              stat.add("error_description", error_str);
              
              // エラーマップに登録
              bool is_new = (data->error_map.count("robot_error") == 0) || 
                           (data->error_map["robot_error"].message != error_str);
              
              if (is_new) {
                data->error_map["robot_error"] = {
                  "robot_error",   // タイプ
                  error_str,       // メッセージ
                  diagnostic_msgs::msg::DiagnosticStatus::ERROR,  // レベル
                  now()            // タイムスタンプ
                };
                data->has_error_changed = true;
              }
            } else {
              stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No error");
              
              // エラーが解消された場合はマップから削除
              if (data->error_map.count("robot_error") > 0) {
                data->error_map.erase("robot_error");
                data->has_error_changed = true;
              }
            }
          } else {
            std::string message = "No robot feedback received";
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
            
            // エラーマップに登録
            bool is_new = (data->error_map.count("robot_error") == 0) || 
                         (data->error_map["robot_error"].message != message);
            
            if (is_new) {
              data->error_map["robot_error"] = {
                "robot_error",   // タイプ
                message,         // メッセージ
                diagnostic_msgs::msg::DiagnosticStatus::ERROR,  // レベル
                now()            // タイムスタンプ
              };
              data->has_error_changed = true;
            }
          }
        }
      });
    data->direct_publisher = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics/robot_" + std::to_string(robot_id), 10);
  }
};
}  // namespace crane

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<crane::DiagnosticPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
