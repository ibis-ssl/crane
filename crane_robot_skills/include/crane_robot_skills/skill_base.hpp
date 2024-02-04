// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SKILL_BASE_HPP_
#define CRANE_ROBOT_SKILLS__SKILL_BASE_HPP_

#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#undef DEFAULT

template <class... Ts>
struct overloaded : Ts...
{
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

namespace crane
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
  };

  StateMachine(StatesType init_state) : current_state(init_state) {}

  void addTransition(
    const StatesType & from, const StatesType & to, std::function<bool()> condition)
  {
    transitions.emplace_back({from, to}, condition);
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

  StatesType getCurrentState() { return current_state; }

protected:
  StatesType current_state;

  std::vector<Transition> transitions;
};

template <typename StatesType = DefaultStates>
class SkillBase
{
public:
  using ParameterType = std::variant<double, bool, int, std::string>;
  enum class Status {
    SUCCESS,
    FAILURE,
    RUNNING,
  };

  using StateFunctionType = std::function<Status(
    const std::shared_ptr<WorldModelWrapper> &, const std::shared_ptr<RobotInfo> &,
    crane::RobotCommandWrapper &)>;

  SkillBase(
    const std::string & name, uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model,
    StatesType init_state)
  : name(name),
    world_model(world_model),
    robot(world_model->getOurRobot(id)),
    state_machine(init_state)
  {
  }

  //  SkillBase(
  //    const std::string & name, uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model) : SkillBase(name, id, world_model, DefaultStates::DEFAULT) {}

  const std::string name;

  Status run(
    RobotCommandWrapper & command,
    std::optional<std::unordered_map<std::string, ParameterType>> parameters_opt = std::nullopt)
  {
    if (parameters_opt) {
      parameters = parameters_opt.value();
    }
    state_machine.update();

    command.latest_msg.current_pose.x = robot->pose.pos.x();
    command.latest_msg.current_pose.y = robot->pose.pos.y();
    command.latest_msg.current_pose.theta = robot->pose.theta;

    return state_functions[state_machine.getCurrentState()](world_model, robot, command);
  }

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

  void setParameter(const std::string & key, bool value) { parameters[key] = value; }

  void setParameter(const std::string & key, int value) { parameters[key] = value; }

  void setParameter(const std::string & key, double value) { parameters[key] = value; }

  void setParameter(const std::string & key, const std::string & value) { parameters[key] = value; }

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

  virtual void print(std::ostream & os) const {}

  // operator<< がAのprivateメンバにアクセスできるようにfriend宣言
  friend std::ostream & operator<<(std::ostream & os, const SkillBase<> & skill);

protected:
  //  Status status = Status::RUNNING;

  std::shared_ptr<WorldModelWrapper> world_model;

  std::shared_ptr<RobotInfo> robot;

  StateMachine<StatesType> state_machine;

  std::unordered_map<StatesType, StateFunctionType> state_functions;

  std::unordered_map<std::string, ParameterType> parameters;
};
}  // namespace crane

inline std::ostream & operator<<(std::ostream & os, const crane::SkillBase<> & skill)
{
  skill.print(os);
  return os;
}

inline std::ostream & operator<<(
  std::ostream & os, const std::shared_ptr<crane::SkillBase<>> & skill)
{
  skill->print(os);
  return os;
}

#endif  // CRANE_ROBOT_SKILLS__SKILL_BASE_HPP_
