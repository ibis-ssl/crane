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
  setUpSkillDictionary<skills::CmdKickStraight>();
  setUpSkillDictionary<skills::CmdDribble>();
  setUpSkillDictionary<skills::CmdSetVelocity>();
  setUpSkillDictionary<skills::CmdSetTargetPosition>();
  setUpSkillDictionary<skills::CmdSetDribblerTargetPosition>();
  setUpSkillDictionary<skills::CmdSetTargetTheta>();
  setUpSkillDictionary<skills::CmdStopHere>();
  setUpSkillDictionary<skills::Teleop>();
  //  setUpSkillDictionary<skills::CmdDisablePlacementAvoidance>();
  //  setUpSkillDictionary<skills::CmdEnablePlacementAvoidance>();
  //  setUpSkillDictionary<skills::CmdDisableBallAvoidance>();
  //  setUpSkillDictionary<skills::CmdEnableBallAvoidance>();
  //  setUpSkillDictionary<skills::CmdDisableCollisionAvoidance>();
  //  setUpSkillDictionary<skills::CmdEnableCollisionAvoidance>();
  //  setUpSkillDictionary<skills::CmdDisableGoalAreaAvoidance>();
  //  setUpSkillDictionary<skills::CmdEnableGoalAreaAvoidance>();
  //  setUpSkillDictionary<skills::CmdSetGoalieDefault>();
  //  setUpSkillDictionary<skills::CmdEnableBallCenteringControl>();
  //  setUpSkillDictionary<skills::CmdEnableLocalGoalie>();
  setUpSkillDictionary<skills::CmdSetMaxVelocity>();
  setUpSkillDictionary<skills::Attacker>();
  //  setUpSkillDictionary<skills::CmdSetMaxAcceleration>();
  //  setUpSkillDictionary<skills::CmdSetTerminalVelocity>();
  setUpSkillDictionary<skills::CmdEnableStopFlag>();
  setUpSkillDictionary<skills::CmdDisableStopFlag>();
  setUpSkillDictionary<skills::CmdLiftUpDribbler>();
  setUpSkillDictionary<skills::CmdLookAt>();
  setUpSkillDictionary<skills::CmdLookAtBall>();
  setUpSkillDictionary<skills::CmdLookAtBallFrom>();
  setUpSkillDictionary<skills::GetBallContact>();
  //  setUpSkillDictionary<skills::Idle>();
  setUpSkillDictionary<skills::Goalie>();
  setUpSkillDictionary<skills::GoalKick>();
  setUpSkillDictionary<skills::Kick>();
  //  setUpSkillDictionary<skills::MoveToGeometry>();
  setUpSkillDictionary<skills::MoveWithBall>();
  setUpSkillDictionary<skills::Sleep>();
  setUpSkillDictionary<skills::Receive>();
  setUpSkillDictionary<skills::GoOverBall>();
  setUpSkillDictionary<skills::SimpleKickOff>();
  setUpSkillDictionary<skills::StealBall>();
  setUpSkillDictionary<skills::SubAttacker>();
  setUpSkillDictionary<skills::TestMotionPosition>();
  setUpSkillDictionary<skills::Marker>();
  setUpSkillDictionary<skills::SingleBallPlacement>();
  //  setUpSkillDictionary<skills::KickoffAttack>();
  //  setUpSkillDictionary<skills::KickoffSupport>();
  setUpSkillDictionary<skills::EmplaceRobot>();

  skill_execution_client = rclcpp_action::create_client<SkillExecution>(
    ros_node->get_node_base_interface(), ros_node->get_node_graph_interface(),
    ros_node->get_node_logging_interface(), ros_node->get_node_waitables_interface(),
    "/simple_ai/skill_execution");

  ui->commandComboBox->clear();
  for (const auto & [name, task] : default_task_dict) {
    ui->commandComboBox->addItem(QString::fromStdString(task.name));
  }

  // 100ms / 10Hz
  task_execution_timer.setInterval(33);
  QObject::connect(&task_execution_timer, &QTimer::timeout, [&]() {
    // clientで状態確認して
    skills::Status task_result;
    // try {
    // } catch (std::exception & e) {
    //   ui->logTextBrowser->append(QString::fromStdString(e.what()));
    //   task_queue_execution.pop_front();
    //   if (task_queue_execution.empty()) {
    //     onQueueToBeEmpty();
    //   }
    //   return;
    // }

    if (task_result != skills::Status::RUNNING) {
      if (not task.retry()) {
        ui->executioncheckBox->setCheckState(Qt::CheckState::Unchecked);
      } else {
        ui->logTextBrowser->append(QString::fromStdString(std::format(
          "{}を再実行します。残り時間[s]：{}", task.name, std::to_string(task.getRestTime()))));
      }
      if (task_result == skills::Status::FAILURE) {
        ui->logTextBrowser->append(QString::fromStdString("Task " + task.name + " failed"));
      } else if (task_result == skills::Status::SUCCESS) {
        ui->logTextBrowser->append(QString::fromStdString("Task " + task.name + " succeeded"));
      }
    }
  });
  task_execution_timer.start();
}

void CraneCommander::postSkill(
  const std::string & name,
  const std::unordered_map<std::string, skills::ParameterType> & parameters)
{
  std::cout << "Sending skill: " << name << std::endl;
  auto goal = SkillExecution::Goal();
  goal.name = name;
  goal.robot_id = robot_id;
  for (const auto & [name, parameter] : parameters) {
    std::visit(
      overloaded{
        [&](const double e) {
          crane_msgs::msg::NamedFloat msg;
          msg.name = name;
          msg.value = e;
          goal.parameter.float_values.push_back(msg);
        },
        [&](const bool e) {
          crane_msgs::msg::NamedBool msg;
          msg.name = name;
          msg.value = e;
          goal.parameter.bool_values.push_back(msg);
        },
        [&](const int e) {
          crane_msgs::msg::NamedInt msg;
          msg.name = name;
          msg.value = e;
          goal.parameter.int_values.push_back(msg);
        },
        [&](const std::string & e) {
          crane_msgs::msg::NamedString msg;
          msg.name = name;
          msg.value = e;
          goal.parameter.string_values.push_back(msg);
        },
        [&](const Point & e) {
          crane_msgs::msg::NamedPosition msg;
          msg.name = name;
          msg.x = e.x();
          msg.y = e.y();
          goal.parameter.position_values.push_back(msg);
        }},
      parameter);
  }

  auto goal_option = rclcpp_action::Client<SkillExecution>::SendGoalOptions();
  goal_option.feedback_callback =
    [this](
      rclcpp_action::ClientGoalHandle<SkillExecution>::SharedPtr goal_handle,
      const std::shared_ptr<const SkillExecution::Feedback> feedback) {
      std::cout << "Feedback received: " << feedback->message << std::endl;
      ui->logTextBrowser->append(QString::fromStdString(feedback->message));
    };
  goal_option.goal_response_callback =
    [](rclcpp_action::ClientGoalHandle<SkillExecution>::SharedPtr goal_handle) {
      std::cout << "Goal response received: " << goal_handle->get_status() << std::endl;
      // if (goal_handle->get_status() == rclcpp_action::GoalStatus::) {}
    };
  goal_option.result_callback =
    [&](const rclcpp_action::ClientGoalHandle<SkillExecution>::WrappedResult result) {
      ui->executioncheckBox->setCheckState(Qt::CheckState::Unchecked);

      if (result.result->result == static_cast<int>(skills::Status::FAILURE)) {
        ui->logTextBrowser->append(QString::fromStdString("Task " + task.name + " failed"));
      } else if (result.result->result == static_cast<int>(skills::Status::SUCCESS)) {
        ui->logTextBrowser->append(QString::fromStdString("Task " + task.name + " succeeded"));
      }

      std::cout << "Result: " << result.result->result << std::endl;
      if (task.retry()) {
        /*
        ui->logTextBrowser->append(QString::fromStdString(std::format(
          "{}を再実行します。残り時間[s]：{}", task.name, std::to_string(task.getRestTime()))));
        postSkill(name, parameters);
        */
      }
    };
  skill_execution_client->async_send_goal(goal, goal_option);
}

CraneCommander::~CraneCommander()
{
  finishROS2();
  delete ui;
}

void CraneCommander::on_executioncheckBox_stateChanged(int state)
{
  if (state == Qt::Checked) {
    std::cout << "実行ボタンが有効になりました" << std::endl;
    task.start_time = std::chrono::steady_clock::now();
    postSkill(task.name, task.parameters);
  } else if (state == Qt::Unchecked) {
    std::cout << "実行ボタンが無効になりました" << std::endl;
    skill_execution_client->async_cancel_all_goals();
  }
}

// 追加ボタンでテーブルを読み取って追加する
void CraneCommander::createSkill()
{
  auto default_params =
    default_task_dict.at(ui->commandComboBox->currentText().toStdString()).parameters;
  Task task;
  task.name = ui->commandComboBox->currentText().toStdString();
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
    } else if (type == "Point") {
      std::string x_str = value.substr(0, value.find(","));
      std::string y_str = value.substr(value.find(",") + 1);
      task.parameters[name] = Point(std::stod(x_str), std::stod(y_str));
    }
  }
}

// ROS 2の更新と表示
void CraneCommander::setupROS2()
{
  ros_node = std::make_shared<ROSNode>();
  ros_update_timer.setInterval(10);  // 100 Hz
  QObject::connect(&ros_update_timer, &QTimer::timeout, [&]() {
    rclcpp::spin_some(ros_node);

    {
      ui->contextTableWidget->clear();
      ui->contextTableWidget->setColumnCount(3);
      QStringList header_list;
      header_list << "Name"
                  << "Value"
                  << "Type";
      ui->contextTableWidget->setHorizontalHeaderLabels(header_list);
      // if (not task_queue_execution.empty()) {
      //   const auto & task = task_queue_execution.front();
      // Contextの表示
      // if (task.skill) {
      //   auto contexts = task.skill->getContexts();
      //   ui->contextTableWidget->setRowCount(contexts.size());
      //   for (size_t index = 0; const auto & [name, context] : contexts) {
      //     ui->contextTableWidget->setItem(
      //       index, 0, new QTableWidgetItem(QString::fromStdString(name)));
      //     ui->contextTableWidget->setItem(
      //       index, 1,
      //       new QTableWidgetItem(QString::fromStdString(skills::getTypeString(context))));
      //     ui->contextTableWidget->setItem(
      //       index, 2,
      //       new QTableWidgetItem(QString::fromStdString(skills::getValueString(context))));
      //     ++index;
      //   }
      // }
      // }
    }
  });
  ros_update_timer.start();
}

void CraneCommander::on_robotIDSpinBox_valueChanged(int arg1)
{
  ui->logTextBrowser->append(
    QString::fromStdString(std::format("ID changed to {}", std::to_string(arg1))));
  robot_id = arg1;
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
  for (const auto & [name, parameter] : default_params) {
    // add new row
    ui->parametersTableWidget->insertRow(ui->parametersTableWidget->rowCount());
    // set name
    auto name_item = new QTableWidgetItem(QString::fromStdString(name));
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
        },
        [&](const Point & e) {
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 1,
            new QTableWidgetItem(
              QString::fromStdString(std::to_string(e.x()) + ", " + std::to_string(e.y()))));
          auto type_item = new QTableWidgetItem("Point");
          type_item->setFlags(type_item->flags() & ~Qt::ItemIsEditable);
          ui->parametersTableWidget->setItem(
            ui->parametersTableWidget->rowCount() - 1, 2, type_item);
        }},
      parameter);
  }
}

template <class SkillType>
void CraneCommander::setUpSkillDictionary()
{
  auto command_base =
    std::make_shared<RobotCommandWrapperBase>("simple_ai", robot_id, ros_node->world_model);
  auto skill = std::make_shared<SkillType>(command_base);
  Task default_task;
  default_task.name = skill->name;
  default_task.parameters = skill->getParameters();
  default_task_dict[skill->name] = default_task;
}
}  // namespace crane
