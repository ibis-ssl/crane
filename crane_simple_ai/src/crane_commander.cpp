// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_commander.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>

#include "ui_qt_form.h"

CraneCommander::CraneCommander(QWidget * parent) : QMainWindow(parent), ui(new Ui::CraneCommander)
{
  ui->setupUi(this);

  task_dict["MoveTo"] = [](const Task & task, crane::RobotCommandWrapper::SharedPtr commander) {
    if (task.args.size() < 3) {
      throw std::runtime_error("MoveTo needs 3 arguments");
    }
    double x = task.args[0];
    double y = task.args[1];
    double theta = task.args[2];
    commander->setTargetPosition(x, y, theta);
    if (commander->world_model->getDistanceFromRobot(commander->robot->id, {x, y}) < 0.1) {
      return true;
    } else {
      return false;
    }
  };

  task_dict["SetStraightKick"] =
    [](const Task & task, crane::RobotCommandWrapper::SharedPtr commander) {
      if (task.args.size() < 1) {
        throw std::runtime_error("SetStraightKick needs 1 argument");
      }
      double power = task.args[0];
      commander->kickStraight(power);
      return true;
    };

  task_dict["SetChipKick"] = [](
                               const Task & task, crane::RobotCommandWrapper::SharedPtr commander) {
    if (task.args.size() < 1) {
      throw std::runtime_error("SetChipKick needs 1 argument");
    }
    double power = task.args[0];
    commander->kickWithChip(power);
    return true;
  };

  task_dict["SetDribblePower"] =
    [](const Task & task, crane::RobotCommandWrapper::SharedPtr commander) {
      if (task.args.size() < 1) {
        throw std::runtime_error("SetDribblePower needs 1 argument");
      }
      double power = task.args[0];
      commander->dribble(power);
      return true;
    };

  task_dict["LookAtBall"] = [](const Task & task, crane::RobotCommandWrapper::SharedPtr commander) {
    commander->lookAtBall();
    return true;
  };

  setupROS2();

  ui->commandComboBox->clear();
  for (const auto & task : task_dict) {
    ui->commandComboBox->addItem(QString::fromStdString(task.first));
  }

  task_execution_timer.setInterval(100);
  QObject::connect(&task_execution_timer, &QTimer::timeout, [&]() {
    if (task_queue.empty()) {
      return;
    }
    auto task = task_queue.front();
    decltype(task_dict)::mapped_type task_func;
    try {
      task_func = task_dict[task.name];
    } catch (std::exception & e) {
      ui->logTextBrowser->append(QString::fromStdString(e.what()));
      task_queue.pop_front();
      if (task_queue.empty()) {
        onQueueToBeEmpty();
      }
      return;
    }
    ui->logTextBrowser->append(QString::fromStdString(task.getText()));

    bool task_result;
    try {
      task_result = task_func(task, ros_node->commander);
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
        onQueueToBeEmpty();
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
void CraneCommander::on_commandAddPushButton_clicked()
{
  std::stringstream command_ss;
  command_ss << ui->commandComboBox->currentText().toStdString() << "(";

  if (ui->arg1LineEdit->text() != "") {
    command_ss << ui->arg1LineEdit->text().toStdString();
  }
  if (ui->arg2LineEdit->text() != "") {
    command_ss << ", " << ui->arg2LineEdit->text().toStdString();
  }
  if (ui->arg3LineEdit->text() != "") {
    command_ss << ", " << ui->arg3LineEdit->text().toStdString();
  }
  command_ss << ")\n";
  // 通常時はTextEditに追加
  if (task_queue.empty()) {
    auto command_queue_str = ui->commandQueuePlainTextEdit->toPlainText();
    command_queue_str += QString::fromStdString(command_ss.str());
    ui->commandQueuePlainTextEdit->setPlainText(command_queue_str);
    return;
  } else {
    // 実行中はqueueに直接追加
    task_queue.emplace_back(command_ss.str());
  }
}

void CraneCommander::on_executionPushButton_clicked()
{
  if (ui->executionPushButton->text() == "実行") {
    auto command_queue = ui->commandQueuePlainTextEdit->toPlainText().split("\n");
    for (const auto & command_str : command_queue) {
      if (command_str == "") {
        continue;
      }
      task_queue.emplace_back(command_str.toStdString());
    }
    ui->commandQueuePlainTextEdit->setEnabled(false);
  } else if (ui->executionPushButton->text() == "停止") {
    task_queue.clear();
    onQueueToBeEmpty();
  }
}

void CraneCommander::setupROS2()
{
  ros_node = std::make_shared<ROSNode>();
  ros_update_timer.setInterval(10);  // 100 Hz
  QObject::connect(&ros_update_timer, &QTimer::timeout, [&]() {
    if (not task_queue.empty()) {
      // print all task in queue
      ui->commandQueuePlainTextEdit->setReadOnly(true);
      ui->executionPushButton->setText("停止");
      std::stringstream ss;
      for (const auto & task : task_queue) {
        ss << task.getText() << "\n";
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
}
