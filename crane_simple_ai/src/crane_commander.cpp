// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_commander.hpp"

#include <crane_robot_skills/skills.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>

#include "ui_qt_form.h"

namespace crane
{
template <typename T>
std::string getStringFromArray(const std::vector<T> & array)
{
  std::stringstream ss;
  for (const auto & e : array) {
    // uint8_tがcharとして出力されるの防ぐ
    if constexpr (std::is_same_v<T, uint8_t>) {
      ss << static_cast<int>(e) << ", ";
    } else {
      ss << e << ", ";
    }
  }
  // 最後のカンマを取り除く
  if (ss.str().size() > 2) {
    return ss.str().substr(0, ss.str().size() - 2);
  } else {
    return ss.str();
  }
}

CraneCommander::CraneCommander(QWidget * parent) : QMainWindow(parent), ui(new Ui::CraneCommander)
{
  ui->setupUi(this);
  setupROS2();
  // set default task
  setUpSkillDictionary<skills::CmdKickWithChip>();
  default_task_dict["CmdKickWithChip"].alias = "アウラ、チップキックしろ";
  setUpSkillDictionary<skills::CmdKickStraight>();
  default_task_dict["CmdKickStraight"].alias = "アウラ、ストレートキックしろ";
  setUpSkillDictionary<skills::CmdDribble>();
  default_task_dict["CmdDribble"].alias = "アウラ、ドリブルしろ";
  //  setUpSkillDictionary<skills::CmdSetVelocity>();
//  default_task_dict["CmdSetVelocity"].alias = "アウラ、この速度で動け";
  setUpSkillDictionary<skills::CmdSetTargetPosition>();
  default_task_dict["CmdSetTargetPosition"].alias = "アウラ、この位置にまで移動しろ";
  setUpSkillDictionary<skills::CmdSetDribblerTargetPosition>();
        default_task_dict["CmdSetDribblerTargetPosition"].alias = "アウラ、この位置にドリブラーを合わせろ";
  setUpSkillDictionary<skills::CmdSetTargetTheta>();
  default_task_dict["CmdSetTargetTheta"].alias = "アウラ、この角度に向け";
  setUpSkillDictionary<skills::CmdStopHere>();
  default_task_dict["CmdStopHere"].alias = "アウラ、止まれ";
  //  setUpSkillDictionary<skills::CmdDisablePlacementAvoidance>();
  //  default_task_dict["CmdDisablePlacementAvoidance"].alias = "アウラ、ボールプレイスメントエリアを気にするな";
  //  setUpSkillDictionary<skills::CmdEnablePlacementAvoidance>();
        //  default_task_dict["CmdEnablePlacementAvoidance"].alias = "アウラ、ボールプレイスメントエリアを避けろ";
  //  setUpSkillDictionary<skills::CmdDisableBallAvoidance>();
        //  default_task_dict["CmdDisableBallAvoidance"].alias = "アウラ、ボールを気にするな;
  //  setUpSkillDictionary<skills::CmdEnableBallAvoidance>();
    //  default_task_dict["CmdEnableBallAvoidance"].alias = "アウラ、ボールを避けろ";
  //  setUpSkillDictionary<skills::CmdDisableCollisionAvoidance>();
        //  default_task_dict["CmdDisableCollisionAvoidance"].alias = "アウラ、他のロボットを気にするな";
  //  setUpSkillDictionary<skills::CmdEnableCollisionAvoidance>();
    //  default_task_dict["CmdEnableCollisionAvoidance"].alias = "アウラ、他のロボットを避け��";
  //  setUpSkillDictionary<skills::CmdDisableGoalAreaAvoidance>();
        //  default_task_dict["CmdDisableGoalAreaAvoidance"].alias = "アウラ、ゴールエリアを気にするな";
  //  setUpSkillDictionary<skills::CmdEnableGoalAreaAvoidance>();
          //  default_task_dict["CmdEnableGoalAreaAvoidance"].alias = "アウラ、ゴールエリアを避けろ";
  //  setUpSkillDictionary<skills::CmdSetGoalieDefault>();
        //  default_task_dict["CmdSetGoalieDefault"].alias = "アウラ、ゴールキーパーになれ";
  //  setUpSkillDictionary<skills::CmdEnableBallCenteringControl>();
  //  default_task_dict["CmdEnableBallCenteringControl"].alias = "アウラ、自分でボールをセンタリングしろ";
  //  setUpSkillDictionary<skills::CmdDisableBallCenteringControl>();
  //  default_task_dict["CmdDisableBallCenteringControl"].alias = "アウラ、勝手にでボールをセンタリングするな";
  //  setUpSkillDictionary<skills::CmdEnableLocalGoalie>();
  //  default_task_dict["CmdEnableLocalGoalie"].alias = "アウラ、ローカルゴールキーパーになれ";
  //  setUpSkillDictionary<skills::CmdDisableLocalGoalie>();
  //  default_task_dict["CmdDisableLocalGoalie"].alias = "アウラ、ローカルゴールキーパーをやめろ";
  setUpSkillDictionary<skills::CmdSetMaxVelocity>();
  default_task_dict["CmdSetMaxVelocity"].alias = "アウラ、これ以上速くなるな";
  //  setUpSkillDictionary<skills::CmdSetMaxAcceleration>();
  //  default_task_dict["CmdSetMaxAcceleration"].alias = "アウラ、これ以上加速するな";
  //  setUpSkillDictionary<skills::CmdSetMaxOmega>();
  //  default_task_dict["CmdSetMaxOmega"].alias = "アウラ、これ以上回転するな";
  //  setUpSkillDictionary<skills::CmdSetTerminalVelocity>();
  //  default_task_dict["CmdSetTerminalVelocity"].alias = "アウラ、到着地点でこの速度になれ";
  setUpSkillDictionary<skills::CmdEnableStopFlag>();
  default_task_dict["CmdEnableStopFlag"].alias = "アウラ、緊急アウラ、やめろしろ";
  setUpSkillDictionary<skills::CmdDisableStopFlag>();
  default_task_dict["CmdDisableStopFlag"].alias = "アウラ、緊急アウラ、やめろをやめろ";
  setUpSkillDictionary<skills::CmdLiftUpDribbler>();
  default_task_dict["CmdLiftUpDribbler"].alias = "アウラ、ドリブラーを上げろ";
  setUpSkillDictionary<skills::CmdLookAt>();
  default_task_dict["CmdLookAt"].alias = "アウラ、あそこを見ろ";
  setUpSkillDictionary<skills::CmdLookAtBall>();
  default_task_dict["CmdLookAtBall"].alias = "アウラ、ボールを見ろ";
  setUpSkillDictionary<skills::CmdLookAtBallFrom>();
    default_task_dict["CmdLookAtBallFrom"].alias = "アウラ、ボールをあそこから見ろ";
  setUpSkillDictionary<skills::GetBallContact>();
  default_task_dict["GetBallContact"].alias = "アウラ、ボールに触れろ";
  //  setUpSkillDictionary<skills::Idle>();
  //  default_task_dict["Idle"].alias = "アウラ、何もするな";
  setUpSkillDictionary<skills::Goalie>();
  default_task_dict["Goalie"].alias = "アウラ、ゴールキーパーをやれ";
  //  setUpSkillDictionary<skills::MoveToGeometry>();
  //  default_task_dict["MoveToGeometry"].alias = "アウラ、ここに移動しろ";
  setUpSkillDictionary<skills::MoveWithBall>();
  default_task_dict["MoveWithBall"].alias = "アウラ、ボールを持って移動しろ";
  //  setUpSkillDictionary<skills::TurnAroundPoint>();
  //  default_task_dict["TurnAroundPoint"].alias = "アウラ、ここを中心に回れ";
  setUpSkillDictionary<skills::Sleep>();
  default_task_dict["Sleep"].alias = "アウラ、寝ろ";
  setUpSkillDictionary<skills::GoOverBall>();
  default_task_dict["GoOverBall"].alias = "アウラ、ボールの向こうへ行け";
  setUpSkillDictionary<skills::SimpleAttacker>();
  default_task_dict["SimpleAttacker"].alias = "アウラ、脳死で攻めろ";
  setUpSkillDictionary<skills::Receiver>();
  default_task_dict["Receiver"].alias = "アウラ、ボールを受け取れ";
  setUpSkillDictionary<skills::Marker>();
  default_task_dict["Marker"].alias = "アウラ、敵をマークしろ";
  setUpSkillDictionary<skills::SingleBallPlacement>();
  default_task_dict["SingleBallPlacement"].alias = "アウラ、ボールを配置しろ";
  setUpSkillDictionary<skills::KickoffAttack>();
  setUpSkillDictionary<skills::KickoffSupport>();


  ui->commandComboBox->clear();
  for (const auto & task : default_task_dict) {
    ui->commandComboBox->addItem(QString::fromStdString(task.second.name));
  }

  // 100ms / 10Hz
  task_execution_timer.setInterval(33);
  QObject::connect(&task_execution_timer, &QTimer::timeout, [&]() {
    auto robot_feedback_array = ros_node->robot_feedback_array;
    crane_msgs::msg::RobotFeedback feedback;
    feedback.error_info.push_back(0);
    feedback.error_info.push_back(1);
    feedback.error_info.push_back(2);
    feedback.error_info.push_back(3);
    for (const auto & robot_feedback : robot_feedback_array.feedback) {
      if (robot_feedback.robot_id == ros_node->robot_id) {
        feedback = robot_feedback;
        break;
      }
    }

    ui->robotErrorsLabel->setText(
      QString::fromStdString("エラー：" + getStringFromArray(feedback.error_info)));
    ui->robotCurrentLabel->setText(
      QString::fromStdString("電流：" + getStringFromArray(feedback.motor_current)));
    ui->robotBallDetectionLabel->setText(
      QString::fromStdString("ボール検知：" + getStringFromArray(feedback.ball_detection)));
    ui->robotVelocityOdomLabel->setText(
      QString::fromStdString("オドメトリ(速度)：" + getStringFromArray(feedback.odom_speed)));
    ui->robotPositionOdomLabel->setText(
      QString::fromStdString("オドメトリ(位置)：" + getStringFromArray(feedback.odom)));
    ui->robotMouseSensorLabel->setText(
      QString::fromStdString("マウスセンサ：" + getStringFromArray(feedback.mouse_raw)));
    ui->robotVoltageLabel->setText(
      QString::fromStdString("電圧：" + getStringFromArray(feedback.voltage)));
    ui->robotTemperatureLabel->setText(
      QString::fromStdString("温度：" + getStringFromArray(feedback.temperatures)));
    ui->robotKickStateLabel->setText(
      QString::fromStdString("キック：" + std::to_string(feedback.kick_state)));
    ui->robotYawLabel->setText(
      QString::fromStdString("Yaw：" + std::to_string(feedback.yaw_angle)));
    ui->robotYawDiffLabel->setText(
      QString::fromStdString("YawDiff：" + std::to_string(feedback.diff_angle)));

    if (task_queue_execution.empty() or ui->executionPushButton->text() == "アウラ、やれ") {
      return;
    } else {
      auto & task = task_queue_execution.front();
      if (task.skill == nullptr) {
        task.skill = skill_generators[task.name](ros_node->robot_id, ros_node->world_model);
        task.start_time = std::chrono::steady_clock::now();
      }

      skills::Status task_result;
      try {
        task_result = task.skill->run(ros_node->visualizer, task.parameters);
        ros_node->latest_msg = task.skill->getRobotCommand();
        std::stringstream ss;
        ss << "...ありえない...この私が...";
        task.skill->print(ss);
        ui->logTextBrowser->append(QString::fromStdString(ss.str()));
      } catch (std::exception & e) {
        ui->logTextBrowser->append(QString::fromStdString(e.what()));
        task_queue_execution.pop_front();
        if (task_queue_execution.empty()) {
          onQueueToBeEmpty();
        }
        return;
      }

      if (task_result != skills::Status::RUNNING) {
        if (not task.retry()) {
          task_queue_execution.pop_front();
        } else {
          ui->logTextBrowser->append(QString::fromStdString(
            task.name + "を再実行します。残り時間[s]：" + std::to_string(task.getRestTime())));
        }
        if (task_result == skills::Status::FAILURE) {
          ui->logTextBrowser->append(QString::fromStdString("Task " + task.name + " failed"));
        } else if (task_result == skills::Status::SUCCESS) {
          ui->logTextBrowser->append(QString::fromStdString("Task " + task.name + " succeeded"));
        }
        if (task_queue_execution.empty()) {
          ui->commandQueuePlainTextEdit->clear();
        }
      }
    }
  });
  task_execution_timer.start();
}

void CraneCommander::onQueueToBeEmpty()
{
  ui->commandQueuePlainTextEdit->clear();
  ui->commandQueuePlainTextEdit->setEnabled(true);
  ui->executionPushButton->setText("アウラ、やれ");
}

CraneCommander::~CraneCommander()
{
  finishROS2();
  delete ui;
}

// 追加ボタンでテーブルを読み取って追加する
void CraneCommander::on_commandAddPushButton_clicked()
{
  auto default_params =
    default_task_dict.at(ui->commandComboBox->currentText().toStdString()).parameters;
  Task task;
  task.name = ui->commandComboBox->currentText().toStdString();
  task.alias = default_task_dict.at(ui->commandComboBox->currentText().toStdString()).alias;
  task.retry_time = ui->continuousTimeDoubleSpinBox->value();
  ui->continuousTimeDoubleSpinBox->setValue(0.0);
  for (int i = 0; i < ui->parametersTableWidget->rowCount(); i++) {
    std::string name = ui->parametersTableWidget->item(i, 0)->text().toStdString();
    std::string value = ui->parametersTableWidget->item(i, 1)->text().toStdString();
    std::string type = ui->parametersTableWidget->item(i, 2)->text().toStdString();
    if (type == "double") {
      task.parameters[name] = std::stod(value);
    } else if (type == "bool") {
      task.parameters[name] = static_cast<bool>(value == "true");
    } else if (type == "int") {
      task.parameters[name] = std::stoi(value);
    } else if (type == "string") {
      task.parameters[name] = value;
    }
  }

  task_queue.emplace_back(task);
}

void CraneCommander::on_executionPushButton_clicked()
{
  if (ui->executionPushButton->text() == "アウラ、やれ") {
    task_queue_execution.clear();
    task_queue_execution = task_queue;
    if (not task_queue_execution.empty()) {
      ui->executionPushButton->setText("アウラ、やめろ");
    }
  } else if (ui->executionPushButton->text() == "アウラ、やめろ") {
    ui->executionPushButton->setText("アウラ、やれ");
  }
}

// ROS 2の更新と表示
void CraneCommander::setupROS2()
{
  ros_node = std::make_shared<ROSNode>();
  ros_update_timer.setInterval(10);  // 100 Hz
  QObject::connect(&ros_update_timer, &QTimer::timeout, [&]() {
    if (not task_queue.empty()) {
      // task_queueを表示
      std::stringstream ss;
      for (const auto & task : task_queue) {
        ss << task.alias << std::endl;
      }
      ui->commandQueuePlainTextEdit->setPlainText(QString::fromStdString(ss.str()));
    } else {
      ui->commandQueuePlainTextEdit->clear();
    }

    if (task_queue_execution.empty()) {
      ui->executionPushButton->setText("アウラ、やれ");
    }
    rclcpp::spin_some(ros_node);
  });
  ros_update_timer.start();
}

void CraneCommander::on_robotIDSpinBox_valueChanged(int arg1)
{
  ui->logTextBrowser->append(QString::fromStdString("ID changed to " + std::to_string(arg1)));
  ros_node->changeID(arg1);
}

// コマンドが変わったらテーブルにデフォルト値を入れる
void CraneCommander::on_commandComboBox_currentTextChanged(const QString & command_name)
{
  // テーブルをリセット
  ui->parametersTableWidget->clear();
  while (ui->parametersTableWidget->rowCount() > 0) {
    ui->parametersTableWidget->removeRow(0);
  }
  // ヘッダの設定
  ui->parametersTableWidget->setColumnCount(3);
  QStringList header_list;
  header_list << "Name"
              << "Value"
              << "Type";
  ui->parametersTableWidget->setHorizontalHeaderLabels(header_list);

  auto default_params = default_task_dict[command_name.toStdString()].parameters;
  for (auto parameter : default_params) {
    // add new row
    ui->parametersTableWidget->insertRow(ui->parametersTableWidget->rowCount());
    // set name
    auto name_item = new QTableWidgetItem(QString::fromStdString(parameter.first));
    name_item->setFlags(name_item->flags() & ~Qt::ItemIsEditable);
    ui->parametersTableWidget->setItem(ui->parametersTableWidget->rowCount() - 1, 0, name_item);
    std::visit(
      overloaded{
        [&](const double e) {
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 1, new QTableWidgetItem(QString::number(e)));
          auto type_item = new QTableWidgetItem("double");
          type_item->setFlags(type_item->flags() & ~Qt::ItemIsEditable);
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 2, type_item);
        },
        [&](const bool e) {
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 1,
            new QTableWidgetItem(e ? "true" : "false"));
          auto type_item = new QTableWidgetItem("bool");
          type_item->setFlags(type_item->flags() & ~Qt::ItemIsEditable);
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 2, type_item);
        },
        [&](const int e) {
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 1, new QTableWidgetItem(QString::number(e)));
          auto type_item = new QTableWidgetItem("int");
          type_item->setFlags(type_item->flags() & ~Qt::ItemIsEditable);
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 2, type_item);
        },
        [&](const std::string & e) {
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 1,
            new QTableWidgetItem(QString::fromStdString(e)));
          auto type_item = new QTableWidgetItem("string");
          type_item->setFlags(type_item->flags() & ~Qt::ItemIsEditable);
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 2, type_item);
        }},
      parameter.second);
  }
}

void CraneCommander::on_queueClearPushButton_clicked()
{
  task_queue.clear();
  ui->commandQueuePlainTextEdit->clear();
  ui->logTextBrowser->append("コマンドキューをクリアしました");
}

template <class SkillType>
void CraneCommander::setUpSkillDictionary()
{
  auto skill = std::make_shared<SkillType>(0, ros_node->world_model);
  Task default_task;
  default_task.name = skill->name;
  default_task.alias = skill->name;
  default_task.parameters = skill->getParameters();
  default_task_dict[skill->name] = default_task;
  skill_generators[skill->name] = [](uint8_t id, WorldModelWrapper::SharedPtr & world_model)
    -> std::shared_ptr<skills::SkillInterface> {
    return std::make_shared<SkillType>(id, world_model);
  };
}
}  // namespace crane
