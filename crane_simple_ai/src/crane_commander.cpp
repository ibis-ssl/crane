// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_commander.hpp"
#include "ui_qt_form.h"

#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>

CraneCommander::CraneCommander(QWidget * parent) : QMainWindow(parent), ui(new Ui::CraneCommander)
{
  ui->setupUi(this);
  setupROS2();

  //    QObject::connect(&thread_time, SIGNAL(data_update(int)),this, SLOT(timer_callback(int)) );
  installEventFilter(this);
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
    ui->commandQueuePlainTextEdit->setEnabled(true);
  }
}

void CraneCommander::setupROS2()
{
  ros_node = std::make_shared<ROSNode>();
  ros_update_timer.setInterval(10);  // 100 Hz
  QObject::connect(&ros_update_timer, &QTimer::timeout, [&]() {
    if (task_queue.empty()) {
      ui->commandQueuePlainTextEdit->setReadOnly(false);
      ui->executionPushButton->setText("実行");
    } else {
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
