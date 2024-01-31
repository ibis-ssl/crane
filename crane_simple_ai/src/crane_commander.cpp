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

  //  task_dict["MoveTo"] = [](const Task & task, crane::RobotCommandWrapper::SharedPtr commander) {
  ////    if (task.args.size() < 3) {
  ////      throw std::runtime_error("MoveTo needs 3 arguments");
  ////    }
  ////    double x = task.args[0];
  ////    double y = task.args[1];
  ////    double theta = task.args[2];
  //    commander->setTargetPosition(x, y, theta);
  //    if (commander->world_model->getDistanceFromRobot(commander->robot->id, {x, y}) < 0.1) {
  //      return true;
  //    } else {
  //      return false;
  //    }
  //  };
  //
  //  task_dict["SetStraightKick"] =
  //    [](const Task & task, crane::RobotCommandWrapper::SharedPtr commander) {
  //      if (task.args.size() < 1) {
  //        throw std::runtime_error("SetStraightKick needs 1 argument");
  //      }
  //      double power = task.args[0];
  //      commander->kickStraight(power);
  //      return true;
  //    };
  //
  //  task_dict["SetChipKick"] = [](
  //                               const Task & task, crane::RobotCommandWrapper::SharedPtr commander) {
  //    if (task.args.size() < 1) {
  //      throw std::runtime_error("SetChipKick needs 1 argument");
  //    }
  //    double power = task.args[0];
  //    commander->kickWithChip(power);
  //    return true;
  //  };
  //
  //  task_dict["SetDribblePower"] =
  //    [](const Task & task, crane::RobotCommandWrapper::SharedPtr commander) {
  //      if (task.args.size() < 1) {
  //        throw std::runtime_error("SetDribblePower needs 1 argument");
  //      }
  //      double power = task.args[0];
  //      commander->dribble(power);
  //      return true;
  //    };
  //
  //  task_dict["LookAtBall"] = [](const Task & task, crane::RobotCommandWrapper::SharedPtr commander) {
  //    commander->lookAtBall();
  //    return true;
  //  };

  ui->commandComboBox->clear();
  for (const auto & task : default_task_dict) {
    ui->commandComboBox->addItem(QString::fromStdString(task.second.name));
  }

  task_execution_timer.setInterval(100);
  QObject::connect(&task_execution_timer, &QTimer::timeout, [&]() {
    if (task_queue.empty() or ui->executionPushButton->text() == "実行"){
      return;
    }else {
      auto task = task_queue.front();
      if (task.skill == nullptr) {
        task.skill = skill_generators[task.name](
          ros_node->commander->getMsg().robot_id, ros_node->world_model);
      }
      ui->logTextBrowser->append(QString::fromStdString(task.getText()));

      bool task_result;
      try {
        task_result =
          (task.skill->run(*ros_node->commander, task.parameters) == SkillBase<>::Status::SUCCESS);
      } catch (std::exception & e) {
        ui->logTextBrowser->append(QString::fromStdString(e.what()));
        task_queue.pop_front();
        if (task_queue.empty()) {
          onQueueToBeEmpty();
        }
        return;
      }

      if (task_result) {
        task_queue.pop_front();
        if (task_queue.empty()) {
        }
      }
    }
  });
  task_execution_timer.start();

  //    QObject::connect(&thread_time, SIGNAL(data_update(int)),this, SLOT(timer_callback(int)) );
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

  //    if(event->type() ==  QEvent::WindowDeactivate){
  //        ai_cmd.local_target_speed[0]=0.0;
  //        ai_cmd.local_target_speed[1]=0.0;
  //    }
  //    else{
  //        if (event->type() == QEvent::KeyPress) {
  //            bRtn = eventKeyPress(static_cast<QKeyEvent *>(event));
  //        }
  //        else if (event->type() == QEvent::KeyRelease) {
  //            bRtn = eventKeyRelease(static_cast<QKeyEvent *>(event));
  //        }
  //    }

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
  ui->parametersTableWidget->clear();
  for (int i = 0; i < ui->parametersTableWidget->rowCount(); i++) {
    ui->parametersTableWidget->removeRow(i);
  }
  ui->parametersTableWidget->setColumnCount(3);
  QStringList headerlist;
  headerlist << "Name"
             << "Value"
             << "Type";
  ui->parametersTableWidget->setHorizontalHeaderLabels(headerlist);
  auto default_params = default_task_dict[command_name.toStdString()].parameters;
  for (auto parameter : default_params) {
    // add new row
    ui->parametersTableWidget->insertRow(ui->parametersTableWidget->rowCount());
    // set name
    ui->parametersTableWidget->setItem(
      ui->parametersTableWidget->rowCount() - 1, 0,
      new QTableWidgetItem(QString::fromStdString(parameter.first)));

    std::visit(
      overloaded{
        [&](double e) {
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 1, new QTableWidgetItem(QString::number(e)));
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 2, new QTableWidgetItem("double"));
        },
        [&](bool e) {
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 1,
            new QTableWidgetItem(e ? "true" : "false"));
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 2, new QTableWidgetItem("bool"));
        },
        [&](int e) {
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 1, new QTableWidgetItem(QString::number(e)));
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 2, new QTableWidgetItem("int"));
        },
        [&](std::string e) {
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 1,
            new QTableWidgetItem(QString::fromStdString(e)));
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 2, new QTableWidgetItem("string"));
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
