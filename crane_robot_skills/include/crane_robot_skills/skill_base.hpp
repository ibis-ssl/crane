// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SKILL_BASE_HPP_
#define CRANE_ROBOT_SKILLS__SKILL_BASE_HPP_

#include <crane_msg_wrappers/consai_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#undef DEFAULT

template <class... Ts>
struct overloaded : Ts...
{
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

namespace crane::skills
{
enum class DefaultStates {
  DEFAULT,
};

template <typename StatesType>
class StateMachine
{
public:
  struct Transition
  {
    StatesType from;
    StatesType to;
    std::function<bool()> condition;
    Transition(StatesType f, StatesType t, std::function<bool()> cond)
    : from(f), to(t), condition(cond)
    {
    }
  };

  explicit StateMachine(StatesType init_state) : current_state(init_state) {}

  void addTransition(
    const StatesType & from, const StatesType & to, std::function<bool()> condition)
  {
    transitions.emplace_back(from, to, condition);
  }

  void update()
  {
    if (auto it = std::find_if(
          transitions.begin(), transitions.end(),
          [this](const Transition & transition) {
            return transition.from == current_state && transition.condition();
          });
        it != transitions.end()) {
      current_state = it->to;
    }
  }

  StatesType getCurrentState() const { return current_state; }

protected:
  StatesType current_state;

  std::vector<Transition> transitions;
};

enum class Status {
  SUCCESS,
  FAILURE,
  RUNNING,
};

using ParameterType = std::variant<double, bool, int, std::string>;

class SkillInterface
{
public:
  SkillInterface(
    const std::string & name, uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
  : name(name), world_model(wm), robot(world_model->getOurRobot(id))
  {
  }
  const std::string name;

  virtual Status run(
    const ConsaiVisualizerWrapper::SharedPtr & visualizer,
    std::optional<std::unordered_map<std::string, ParameterType>> parameters_opt =
      std::nullopt) = 0;

  void setParameter(const std::string & key, bool value) { parameters[key] = value; }

  void setParameter(const std::string & key, int value) { parameters[key] = value; }

  void setParameter(const std::string & key, double value) { parameters[key] = value; }

  void setParameter(const std::string & key, const std::string & value) { parameters[key] = value; }

  virtual crane_msgs::msg::RobotCommand getRobotCommand() = 0;

  template <class T>
  auto getParameter(const std::string & key) const
  {
    try {
      return std::get<T>(parameters.at(key));
    } catch (const std::out_of_range & e) {
      throw std::out_of_range("Parameter " + key + " is not found");
    }
  }

  const auto & getParameters() const { return parameters; }

  void getParameterSchemaString(std::ostream & os) const
  {
    for (const auto & element : parameters) {
      os << element.first << ": ";
      std::visit(
        overloaded{
          [&](double e) { os << "double, " << e << std::endl; },
          [&](int e) { os << "int, " << e << std::endl; },
          [&](const std::string & e) { os << "string, " << e << std::endl; },
          [&](bool e) { os << "bool, " << e << std::endl; }},
        element.second);
    }
  }

  virtual void print(std::ostream &) const {}

  // operator<< がAのprivateメンバにアクセスできるようにfriend宣言
  friend std::ostream & operator<<(std::ostream & os, const SkillInterface & skill);

protected:
  std::shared_ptr<WorldModelWrapper> world_model;

  std::shared_ptr<RobotInfo> robot;

  std::unordered_map<std::string, ParameterType> parameters;

  Status status = Status::RUNNING;
};

template <typename StatesType = DefaultStates>
class SkillBase : public SkillInterface
{
public:
  using StateFunctionType = std::function<Status(ConsaiVisualizerWrapper::SharedPtr)>;

  SkillBase(
    const std::string & name, uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm,
    StatesType init_state, const std::shared_ptr<RobotCommandWrapper> & robot_command = nullptr)
  : SkillInterface(name, id, wm), state_machine(init_state)
  {
    if (robot_command) {
      command = robot_command;
    } else {
      command = std::make_shared<RobotCommandWrapper>(id, wm);
    }
  }

  void setCommander(const std::shared_ptr<RobotCommandWrapper> & commander)
  {
    this->command = commander;
  }

  Status run(
    const ConsaiVisualizerWrapper::SharedPtr & visualizer,
    std::optional<std::unordered_map<std::string, ParameterType>> parameters_opt =
      std::nullopt) override
  {
    if (parameters_opt) {
      parameters = parameters_opt.value();
    }
    state_machine.update();

    command->latest_msg.current_pose.x = robot->pose.pos.x();
    command->latest_msg.current_pose.y = robot->pose.pos.y();
    command->latest_msg.current_pose.theta = robot->pose.theta;

    return state_functions[state_machine.getCurrentState()](visualizer);
  }

  crane_msgs::msg::RobotCommand getRobotCommand() override { return command->getMsg(); }

  std::shared_ptr<RobotCommandWrapper> commander() const { return command; }

  void addStateFunction(const StatesType & state, StateFunctionType function)
  {
    if (state_functions.find(state) != state_functions.end()) {
      RCLCPP_WARN(
        rclcpp::get_logger("State: " + name),
        "State function already exists and is overwritten now.");
    }
    state_functions[state] = function;
  }

  void addTransitions(
    const StatesType & from,
    std::vector<std::pair<StatesType, std::function<bool()>>> transition_targets)
  {
    for (const auto & transition_target : transition_targets) {
      state_machine.addTransition(from, transition_target.first, transition_target.second);
    }
  }

  void addTransition(const StatesType from, const StatesType to, std::function<bool()> condition)
  {
    state_machine.addTransition(from, to, condition);
  }

  StatesType getCurrentState() const { return state_machine.getCurrentState(); }

protected:
  //    Status status = Status::RUNNING;

  StateMachine<StatesType> state_machine;

  std::unordered_map<StatesType, StateFunctionType> state_functions;

  // operator<< がAのprivateメンバにアクセスできるようにfriend宣言
  friend std::ostream & operator<<(std::ostream & os, const SkillBase<StatesType> & skill_base);

  std::shared_ptr<RobotCommandWrapper> command = nullptr;
};
}  // namespace crane::skills

inline std::ostream & operator<<(std::ostream & os, const crane::skills::SkillInterface & skill)
{
  skill.print(os);
  return os;
}

inline std::ostream & operator<<(
  std::ostream & os, const std::shared_ptr<crane::skills::SkillInterface> & skill)
{
  skill->print(os);
  return os;
}

template <typename StatesType>
inline std::ostream & operator<<(
  std::ostream & os, const crane::skills::SkillBase<StatesType> & skill)
{
  skill.print(os);
  return os;
}

template <typename StatesType>
inline std::ostream & operator<<(
  std::ostream & os, const std::shared_ptr<crane::skills::SkillBase<StatesType>> & skill)
{
  skill->print(os);
  return os;
}

#endif  // CRANE_ROBOT_SKILLS__SKILL_BASE_HPP_
