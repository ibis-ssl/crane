// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SKILL_BASE_HPP_
#define CRANE_ROBOT_SKILLS__SKILL_BASE_HPP_

#include <../magic_enum.hpp>
#include <crane_msg_wrappers/consai_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

template <class... Ts>
struct overloaded : Ts...
{
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

namespace crane::skills
{
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

using ParameterType = std::variant<double, bool, int, std::string, Point>;

using ContextType = std::variant<double, bool, int, std::string, Point, std::optional<Point>>;

inline std::string getTypeString(const ContextType & type)
{
  std::string type_string;
  std::visit(
    overloaded{
      [&type_string](const double) { type_string = "double"; },
      [&type_string](const bool) { type_string = "bool"; },
      [&type_string](const int) { type_string = "int"; },
      [&type_string](const std::string) { type_string = "string"; },
      [&type_string](const Point) { type_string = "Point"; },
      [&type_string](const std::optional<Point>) { type_string = "op<Point>"; }},
    type);
  return type_string;
}

inline std::string getValueString(const ContextType & type)
{
  std::string value_string;
  std::visit(
    overloaded{
      [&value_string](const double e) { value_string = std::to_string(e); },
      [&value_string](const bool e) { value_string = std::to_string(e); },
      [&value_string](const int e) { value_string = std::to_string(e); },
      [&value_string](const std::string e) { value_string = e; },
      [&value_string](const Point e) {
        value_string = "(" + std::to_string(e.x()) + ", " + std::to_string(e.y()) + ")";
      },
      [&value_string](const std::optional<Point> e) {
        if (e) {
          value_string = "(" + std::to_string(e->x()) + ", " + std::to_string(e->y()) + ")";
        } else {
          value_string = "nullopt";
        }
      }},
    type);
  return value_string;
}

class SkillInterface
{
public:
  SkillInterface(
    const std::string & name, uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
  : name(name), command_base(std::make_shared<RobotCommandWrapperBase>(name, id, wm))
  {
  }

  SkillInterface(const std::string & name, RobotCommandWrapperBase::SharedPtr command)
  : name(name), command_base(command)
  {
    command_base->latest_msg.skill_name = name;
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

  void setParameter(const std::string & key, const Point & value) { parameters[key] = value; }

  template <typename T>
  T & getEditableParameter(const std::string & key)
  {
    try {
      return std::get<T &>(parameters.at(key));
    } catch (const std::out_of_range & e) {
      throw std::out_of_range("Parameter " + key + " is not found");
    }
  }

  template <typename T>
  T & getContextReference(const std::string & key, const T initial_value = T())
  {
    // メモ：std::unordered_mapの要素への参照はリハッシュや要素の挿入などでは変化しない
    // 　　　（該当要素の削除は当然アウト）
    if (not contexts.contains(key)) {
      contexts.emplace(key, initial_value);
    }
    return get<T>(contexts.at(key));
  }

  auto getContexts() -> const std::unordered_map<std::string, ContextType> & { return contexts; }

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
          [&os](double e) { os << "double, " << e << std::endl; },
          [&os](int e) { os << "int, " << e << std::endl; },
          [&os](const std::string & e) { os << "string, " << e << std::endl; },
          [&os](bool e) { os << "bool, " << e << std::endl; },
          [&os](Point e) { os << "Point, " << e.x() << ", " << e.y() << std::endl; }},
        element.second);
    }
  }

  virtual void print(std::ostream &) const {}

  // operator<< がAのprivateメンバにアクセスできるようにfriend宣言
  friend std::ostream & operator<<(std::ostream & os, const SkillInterface & skill);

  uint8_t getID() const { return command_base->robot->id; }

protected:
  std::shared_ptr<RobotCommandWrapperBase> command_base;

  std::shared_ptr<WorldModelWrapper> world_model() const { return command_base->world_model; }

  std::shared_ptr<RobotInfo> robot() const { return command_base->robot; }

  std::unordered_map<std::string, ParameterType> parameters;

  std::unordered_map<std::string, ContextType> contexts;

  Status status = Status::RUNNING;
};

template <typename DefaultCommandT = RobotCommandWrapperPosition>
class SkillBase : public SkillInterface
{
public:
  SkillBase(const std::string & name, uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
  : SkillInterface(name, id, wm), command(this->command_base)
  {
  }

  explicit SkillBase(const std::string & name, RobotCommandWrapperBase::SharedPtr command)
  : SkillInterface(name, command), command(command)
  {
  }

  Status run(
    const ConsaiVisualizerWrapper::SharedPtr & visualizer,
    std::optional<std::unordered_map<std::string, ParameterType>> parameters_opt =
      std::nullopt) override
  {
    if (parameters_opt) {
      parameters = parameters_opt.value();
    }

    command_base->latest_msg.current_pose.x = command_base->robot->pose.pos.x();
    command_base->latest_msg.current_pose.y = command_base->robot->pose.pos.y();
    command_base->latest_msg.current_pose.theta = command_base->robot->pose.theta;

    return update(visualizer);
  }

  virtual Status update(const ConsaiVisualizerWrapper::SharedPtr & visualizer) = 0;

  crane_msgs::msg::RobotCommand getRobotCommand() override { return command.getMsg(); }

  DefaultCommandT & commander() { return command; }

protected:
  // operator<< がAのprivateメンバにアクセスできるようにfriend宣言
  template <typename T>
  friend std::ostream & operator<<(std::ostream & os, const SkillBase<T> & skill_base);

  DefaultCommandT command;
};

template <typename StatesType, typename DefaultCommandT = RobotCommandWrapperPosition>
class SkillBaseWithState : public SkillInterface
{
public:
  using StateFunctionType = std::function<Status(ConsaiVisualizerWrapper::SharedPtr)>;

  SkillBaseWithState(
    const std::string & name, uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm,
    StatesType init_state)
  : SkillInterface(name, id, wm), state_machine(init_state)
  {
  }

  SkillBaseWithState(
    const std::string & name, RobotCommandWrapperBase::SharedPtr command, StatesType init_state)
  : SkillInterface(name, command),
    state_machine(init_state),
    command(command),
    state_string(getContextReference<std::string>("state"))
  {
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
    state_string = magic_enum::enum_name(state_machine.getCurrentState());

    command_base->latest_msg.current_pose.x = command_base->robot->pose.pos.x();
    command_base->latest_msg.current_pose.y = command_base->robot->pose.pos.y();
    command_base->latest_msg.current_pose.theta = command_base->robot->pose.theta;

    return state_functions[state_machine.getCurrentState()](visualizer);
  }

  crane_msgs::msg::RobotCommand getRobotCommand() override { return command.getMsg(); }

  DefaultCommandT & commander() { return command; }

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

  std::string & state_string;

  // operator<< がAのprivateメンバにアクセスできるようにfriend宣言
  template <typename T, typename U>
  friend std::ostream & operator<<(std::ostream & os, const SkillBaseWithState<T, U> & skill_base);

  DefaultCommandT command;
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

template <typename StatesType, typename DefaultCommandT = crane::RobotCommandWrapperPosition>
inline std::ostream & operator<<(
  std::ostream & os, const crane::skills::SkillBaseWithState<StatesType, DefaultCommandT> & skill)
{
  skill.print(os);
  return os;
}

template <typename StatesType, typename DefaultCommandT = crane::RobotCommandWrapperPosition>
inline std::ostream & operator<<(
  std::ostream & os,
  const std::shared_ptr<crane::skills::SkillBaseWithState<StatesType, DefaultCommandT>> & skill)
{
  skill->print(os);
  return os;
}

#endif  // CRANE_ROBOT_SKILLS__SKILL_BASE_HPP_
