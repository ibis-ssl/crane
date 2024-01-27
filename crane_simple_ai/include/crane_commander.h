#ifndef QT_COMMUNICATION_TESTER_H
#define QT_COMMUNICATION_TESTER_H

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
#include <cstdio>
#include <queue>
#include <rclcpp/rclcpp.hpp>

QT_BEGIN_NAMESPACE
namespace Ui
{
class CraneCommander;
}
QT_END_NAMESPACE

struct Task
{
  Task(std::string str)
  {
    // ex: "move_to(1.0, 2.0, 3.0)"
    name = str.substr(0, str.find("("));
    auto args_str = str.substr(str.find("(") + 1, str.find(")") - str.find("(") - 1);
    // into args(std::vector<double>)
    while (args_str.find(",") != std::string::npos) {
      auto arg_str = args_str.substr(0, args_str.find(","));
      args.push_back(std::stod(arg_str));
      args_str = args_str.substr(args_str.find(",") + 1);
    }
    // last arg from "<last arg>)"
    auto arg_str = args_str.substr(0, args_str.find(")"));
    args.push_back(std::stod(arg_str));
  }
  std::string getText() const
  {
    // ex1: "move_to(1.0, 2.0, 3.0)"
    // ex1: "set_kicker_power(1.0)"
    std::string str = name + "(";
    for (auto arg : args) {
      str += std::to_string(arg) + ",";
    }
    // remove last ","
    str = str.substr(0, str.size() - 1);
    str += ")";
    return str;
  }
  std::string name;
  std::vector<double> args;
};

class ROSNode : public rclcpp::Node
{
public:
  ROSNode() : Node("crane_commander") {}
  ~ROSNode() {}
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

protected:
  bool eventFilter(QObject * object, QEvent * event);

private:
  bool eventKeyPress(QKeyEvent * event);
  bool eventKeyRelease(QKeyEvent * event);

private:
  Ui::CraneCommander * ui;
  QTimer ros_update_timer;
  std::shared_ptr<ROSNode> ros_node;
  std::deque<Task> task_queue;
};

#endif  // QT_COMMUNICATION_TESTER_H
