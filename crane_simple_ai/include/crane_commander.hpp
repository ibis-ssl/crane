// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SIMPLE_AI__CRANE_COMMANDER_HPP_
#define CRANE_SIMPLE_AI__CRANE_COMMANDER_HPP_

#include <QDebug>
#include <QFile>
#include <QGraphicsScene>
#include <QKeyEvent>
#include <QMainWindow>
#include <QTextStream>
#include <QThread>
#include <QTimer>
#include <QtGlobal>
#include <cmath>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <cstdio>
#include <queue>
#include <rclcpp/rclcpp.hpp>

QT_BEGIN_NAMESPACE
namespace Ui
{
class CraneCommander;
}
QT_END_NAMESPACE

namespace crane
{
struct Task
{
  std::string getText() const
  {
    // ex1: "move_to(1.0, 2.0, 3.0)"
    // ex1: "set_kicker_power(1.0)"
    std::string str = name + "(";
    //    for (auto arg : args) {
    //      str += std::to_string(arg) + ",";
    //    }
    // remove last ","
    //    if (args.size() > 0) {
    //      str = str.substr(0, str.size() - 1);
    //    }
    str += ")";
    return str;
  }
  std::string name;
  //  std::vector<double> args;
  using ParameterType = std::variant<double, bool, int, std::string>;
  std::map<std::string, ParameterType> parameters;
};

class ROSNode : public rclcpp::Node
{
public:
  ROSNode() : Node("crane_commander")
  {
    world_model = std::make_shared<crane::WorldModelWrapper>(*this);
    commander = std::make_shared<crane::RobotCommandWrapper>(0, world_model);
    publisher_robot_commands =
      create_publisher<crane_msgs::msg::RobotCommands>("/control_targets", 10);

    timer = create_wall_timer(std::chrono::milliseconds(100), [&]() {
      crane_msgs::msg::RobotCommands msg;
      msg.header = world_model->getMsg().header;
      msg.is_yellow = world_model->isYellow();
      msg.robot_commands.push_back(commander->getMsg());
      publisher_robot_commands->publish(msg);
    });
  }
  ~ROSNode() {}
  crane::WorldModelWrapper::SharedPtr world_model;
  crane::RobotCommandWrapper::SharedPtr commander;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<crane_msgs::msg::RobotCommands>::SharedPtr publisher_robot_commands;
};

class CraneCommander : public QMainWindow
{
  Q_OBJECT

public:
  CraneCommander(QWidget * parent = nullptr);
  ~CraneCommander();
  void setupROS2();

  void finishROS2() { rclcpp::shutdown(); }

public slots:
  //    void timer_callback( int time_counter)

private slots:
  void on_commandAddPushButton_clicked();
  void on_executionPushButton_clicked();
  void on_commandComboBox_currentTextChanged(const QString & command_name);

  void on_robotIDSpinBox_valueChanged(int arg1);

protected:
  bool eventFilter(QObject * object, QEvent * event);

private:
  bool eventKeyPress(QKeyEvent * event);
  bool eventKeyRelease(QKeyEvent * event);

  void onQueueToBeEmpty();

private:
  Ui::CraneCommander * ui;
  QTimer ros_update_timer;
  QTimer task_execution_timer;
  std::shared_ptr<ROSNode> ros_node;
  std::deque<Task> task_queue;
  std::unordered_map<
    std::string, std::function<bool(const Task &, crane::RobotCommandWrapper::SharedPtr)>>
    task_dict;
  std::unordered_map<std::string, Task> default_task_dict;

  template <class SkillType>
  void setUpSkillDictionary();
};
}  // namespace crane

#endif  // CRANE_SIMPLE_AI__CRANE_COMMANDER_HPP_
