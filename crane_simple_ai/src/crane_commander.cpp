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
CraneCommander::CraneCommander(QWidget * parent) : QMainWindow(parent), ui(new Ui::CraneCommander)
{
  ui->setupUi(this);
  setupROS2();
  // set default task
  setUpSkillDictionary<CmdKickWithChip>();
  setUpSkillDictionary<CmdKickStraight>();
  setUpSkillDictionary<CmdDribble>();
  setUpSkillDictionary<CmdSetVelocity>();
  setUpSkillDictionary<CmdSetTargetPosition>();
  setUpSkillDictionary<CmdSetDribblerTargetPosition>();
  setUpSkillDictionary<CmdSetTargetTheta>();
  setUpSkillDictionary<CmdStopHere>();
  setUpSkillDictionary<CmdDisablePlacementAvoidance>();
  setUpSkillDictionary<CmdEnablePlacementAvoidance>();
  setUpSkillDictionary<CmdDisableBallAvoidance>();
  setUpSkillDictionary<CmdEnableBallAvoidance>();
  setUpSkillDictionary<CmdDisableCollisionAvoidance>();
  setUpSkillDictionary<CmdEnableCollisionAvoidance>();
  setUpSkillDictionary<CmdDisableGoalAreaAvoidance>();
  setUpSkillDictionary<CmdEnableGoalAreaAvoidance>();
  setUpSkillDictionary<CmdSetGoalieDefault>();
  setUpSkillDictionary<CmdEnableBallCenteringControl>();
  setUpSkillDictionary<CmdEnableLocalGoalie>();
  setUpSkillDictionary<CmdSetMaxVelocity>();
  setUpSkillDictionary<CmdSetMaxAcceleration>();
  setUpSkillDictionary<CmdSetMaxOmega>();
  setUpSkillDictionary<CmdSetTerminalVelocity>();
  setUpSkillDictionary<CmdLookAt>();
  setUpSkillDictionary<CmdLookAtBall>();
  setUpSkillDictionary<CmdLookAtBallFrom>();
  setUpSkillDictionary<GetBallContact>();
  setUpSkillDictionary<Idle>();
  //  setUpSkillDictionary<MoveToGeometry>();
  setUpSkillDictionary<MoveWithBall>();
  setUpSkillDictionary<TurnAroundPoint>();
  setUpSkillDictionary<Sleep>();

  ui->commandComboBox->clear();
  for (const auto & task : default_task_dict) {
    ui->commandComboBox->addItem(QString::fromStdString(task.second.name));
  }

  task_execution_timer.setInterval(100);
  QObject::connect(&task_execution_timer, &QTimer::timeout, [&]() {
    if (task_queue.empty() or ui->executionPushButton->text() == "実行") {
      return;
    } else {
      auto & task = task_queue.front();
      if (task.skill == nullptr) {
        task.skill = skill_generators[task.name](
          ros_node->commander->getMsg().robot_id, ros_node->world_model);
      }
      ui->logTextBrowser->append(QString::fromStdString(task.getText()));

      SkillBase<>::Status task_result;
      try {
        task_result = task.skill->run(*ros_node->commander, task.parameters);
      } catch (std::exception & e) {
        ui->logTextBrowser->append(QString::fromStdString(e.what()));
        task_queue.pop_front();
        if (task_queue.empty()) {
          onQueueToBeEmpty();
        }
        return;
      }

      if (task_result != SkillBase<>::Status::RUNNING) {
        task_queue.pop_front();
        if (task_result == SkillBase<>::Status::FAILURE) {
          ui->logTextBrowser->append(QString::fromStdString("Task " + task.name + " failed"));
        } else if (task_result == SkillBase<>::Status::SUCCESS) {
          ui->logTextBrowser->append(QString::fromStdString("Task " + task.name + " succeeded"));
        }
        if (task_queue.empty()) {
          ui->commandQueuePlainTextEdit->clear();
        }
      }
    }
  });
  task_execution_timer.start();

  installEventFilter(this);
}
void CraneCommander::onQueueToBeEmpty()
{
  ui->commandQueuePlainTextEdit->clear();
  ui->commandQueuePlainTextEdit->setEnabled(true);
  ui->executionPushButton->setText("実行");
}

CraneCommander::~CraneCommander()
{
  finishROS2();
  delete ui;
}

bool CraneCommander::eventKeyPress(QKeyEvent * event) { return true; }

bool CraneCommander::eventKeyRelease(QKeyEvent * event) { return true; }

bool CraneCommander::eventFilter(QObject *, QEvent * event)
{
  bool bRtn = false;
  return bRtn;
}

// 追加ボタンでテーブルを読み取って追加する
void CraneCommander::on_commandAddPushButton_clicked()
{
  auto default_params =
    default_task_dict.at(ui->commandComboBox->currentText().toStdString()).parameters;
  Task task;
  task.name = ui->commandComboBox->currentText().toStdString();
  for (int i = 0; i < ui->parametersTableWidget->rowCount(); i++) {
    std::string name = ui->parametersTableWidget->item(i, 0)->text().toStdString();
    std::string value = ui->parametersTableWidget->item(i, 1)->text().toStdString();
    std::string type = ui->parametersTableWidget->item(i, 2)->text().toStdString();
    if (type == "double") {
      task.parameters[name] = std::stod(value);
    } else if (type == "bool") {
      task.parameters[name] = bool(value == "true");
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
  if (ui->executionPushButton->text() == "実行") {
    if (not task_queue.empty()) {
      ui->executionPushButton->setText("停止");
    }
  } else if (ui->executionPushButton->text() == "停止") {
    ui->executionPushButton->setText("実行");
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
        ss << task.getText() << std::endl;
      }
      ui->commandQueuePlainTextEdit->setPlainText(QString::fromStdString(ss.str()));
    } else {
      ui->commandQueuePlainTextEdit->clear();
      ui->executionPushButton->setText("実行");
    }
    rclcpp::spin_some(ros_node);
  });
  ros_update_timer.start();
}

void CraneCommander::on_robotIDSpinBox_valueChanged(int arg1)
{
  ui->logTextBrowser->append(QString::fromStdString("ID changed to " + std::to_string(arg1)));
  ros_node->commander->setID(arg1);
  ros_node->commander->stopHere();
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
        [&](double e) {
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 1, new QTableWidgetItem(QString::number(e)));
          auto type_item = new QTableWidgetItem("double");
          type_item->setFlags(type_item->flags() & ~Qt::ItemIsEditable);
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 2, type_item);
        },
        [&](bool e) {
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 1,
            new QTableWidgetItem(e ? "true" : "false"));
          auto type_item = new QTableWidgetItem("bool");
          type_item->setFlags(type_item->flags() & ~Qt::ItemIsEditable);
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 2, type_item);
        },
        [&](int e) {
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 1, new QTableWidgetItem(QString::number(e)));
          auto type_item = new QTableWidgetItem("int");
          type_item->setFlags(type_item->flags() & ~Qt::ItemIsEditable);
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 2, type_item);
        },
        [&](std::string e) {
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

template <class SkillType>
void CraneCommander::setUpSkillDictionary()
{
  auto skill = std::make_shared<SkillType>(0, ros_node->world_model);
  Task default_task;
  default_task.name = skill->name;
  default_task.parameters = skill->getParameters();
  default_task_dict[skill->name] = default_task;
  skill_generators[skill->name] =
    [](uint8_t id, WorldModelWrapper::SharedPtr & world_model) -> std::shared_ptr<SkillBase<>> {
    return std::make_shared<SkillType>(id, world_model);
  };
}
}  // namespace crane
