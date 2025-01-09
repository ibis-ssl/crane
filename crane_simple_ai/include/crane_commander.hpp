// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_COMMANDER_HPP_
#define CRANE_COMMANDER_HPP_

#include <QDebug>
#include <QFile>
#include <QGraphicsScene>
#include <QKeyEvent>
#include <QMainWindow>
#include <QTextStream>
#include <QThread>
#include <QTimer>
#include <QtGlobal>
#include <algorithm>
#include <cmath>
#include <crane_msg_wrappers/consai_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msgs/action/skill_execution.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <cstdio>
#include <deque>
#include <map>
#include <memory>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

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

  std::unordered_map<std::string, skills::ParameterType> parameters;

  // std::shared_ptr<skills::SkillInterface> skill = nullptr;

  double retry_time = -1.0;

  std::chrono::time_point<std::chrono::steady_clock> start_time;

  bool retry() const
  {
    if (retry_time <= 0.0) {
      return false;
    }
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
    return duration.count() < retry_time * 1000;
  }

  double getRestTime() const
  {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
    return std::max(retry_time * 1000. - duration.count(), 0.0) / 1000;
  }
};

class ROSNode : public rclcpp::Node
{
public:
  ROSNode() : Node("crane_commander")
  {
    crane::ConsaiVisualizerBuffer::activate(*this);
    world_model = std::make_shared<crane::WorldModelWrapper>(*this);

    subscription_robot_feedback = create_subscription<crane_msgs::msg::RobotFeedbackArray>(
      "/robot_feedback", 10,
      [&](const crane_msgs::msg::RobotFeedbackArray & msg) { robot_feedback_array = msg; });
  }

  crane::WorldModelWrapper::SharedPtr world_model;

  rclcpp::Subscription<crane_msgs::msg::RobotFeedbackArray>::SharedPtr subscription_robot_feedback;

  crane_msgs::msg::RobotFeedbackArray robot_feedback_array;

  crane::ConsaiVisualizerBuffer::MessageBuilder::UniquePtr visualizer =
    std::make_unique<ConsaiVisualizerBuffer::MessageBuilder>("simple_ai");
};

class CraneCommander : public QMainWindow
{
  using SkillExecution = crane_msgs::action::SkillExecution;
  Q_OBJECT

public:
  explicit CraneCommander(QWidget * parent = nullptr);

  ~CraneCommander() override;

  void setupROS2();

  void finishROS2() { rclcpp::shutdown(); }

  void createSkill();

private slots:
  void on_commandComboBox_currentTextChanged(const QString & command_name);

  void on_robotIDSpinBox_valueChanged(int arg1);

  void on_executioncheckBox_stateChanged(int state);

private:
  void onQueueToBeEmpty();

  template <class SkillType>
  void setUpSkillDictionary();

private:
  Ui::CraneCommander * ui;

  QTimer ros_update_timer;

  QTimer task_execution_timer;

  std::shared_ptr<ROSNode> ros_node;

  std::unordered_map<std::string, Task> default_task_dict;

  uint8_t robot_id = 0;

  Task task;

  void postSkill(
    const std::string & name,
    const std::unordered_map<std::string, skills::ParameterType> & parameters);

  rclcpp_action::Client<SkillExecution>::SharedPtr skill_execution_client;
};
}  // namespace crane

#endif  // CRANE_COMMANDER_HPP_
