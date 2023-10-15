//
// Created by hans on 23/10/14.
//

#ifndef CRANE_PLANNER_PLUGINS_SKILL_BASE_HPP
#define CRANE_PLANNER_PLUGINS_SKILL_BASE_HPP

#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <functional>
#include <memory>
#include <string>
#include <vector>

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
  enum class Status {
    SUCCESS,
    FAILURE,
    RUNNING,
  };

  using StateFunctionType = std::function<Status(const std::shared_ptr<WorldModelWrapper> &, const std::shared_ptr<RobotInfo> &, crane::RobotCommandWrapper &)>;

  SkillBase(
    const std::string & name, uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model,
    StatesType & init_state)
  : name(name),
    world_model(world_model),
    robot(world_model->getRobot({true, id})),
    state_machine(init_state)
  {
  }


//  SkillBase(
//    const std::string & name, uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model) : SkillBase(name, id, world_model, DefaultStates::DEFAULT) {}

  const std::string name;

  Status run(RobotCommandWrapper & command){
    state_machine.update();
    return state_functions[state_machine.getCurrentState()](world_model, robot, command);
  }

  void addStateFunction(const StatesType & state, StateFunctionType function){
    if(state_functions.find(state) != state_functions.end()){
      RCLCPP_WARN(rclcpp::get_logger("State: " + name), "State function already exists and is overwritten now.");
    }
    state_functions[state] = function;
  }

  void addTransitions(const StatesType & from, std::vector<std::pair<StatesType, std::function<bool()>>> transition_targets){
    for(const auto &transition_target : transition_targets){
      state_machine.addTransition(from, transition_target.first, transition_target.second);
    }
  }


protected:
//  Status status = Status::RUNNING;

  std::shared_ptr<WorldModelWrapper> world_model;

  std::shared_ptr<RobotInfo> robot;

  StateMachine<StatesType> state_machine;

  std::unordered_map<StatesType, StateFunctionType> state_functions;
};
}  // namespace crane
#endif  //CRANE_PLANNER_PLUGINS_SKILL_BASE_HPP
