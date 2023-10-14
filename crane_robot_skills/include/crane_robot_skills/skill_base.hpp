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
  void addTransition(
    const StatesType & from, const StatesType & to, std::function<bool()> condition)
  {
    transitions.emplace_back({from, to}, condition);
  }
  void setInitialState(StatesType state)
  {
    initial_state = state;
    current_state = initial_state;
  }
  void setFinalState(StatesType state) { final_state = state; }
  void update()
  {
    while (currentState != finalState) {
      auto skill = states.at(currentState);
      auto status = skill->run();
      if (status == Status::SUCCESS) {
        current_state = transitions.at(currentState);
      }
    }
  }

protected:
  StatesType current_state;
  StatesType initial_state;
  StatesType final_state;
  std::vector<std::pair<StatesType, StatesType>, std::function<bool()>> transitions;
};

class SkillBase
{
public:
  enum class Status {
    SUCCESS,
    FAILURE,
    RUNNING,
  };
  SkillBase(const std::string & name, uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
  : name(name), world_model(world_model), robot(world_model->getRobot({true, id}))
  {
  }

  const std::string name;

  virtual Status run(const std::shared_ptr<WorldModelWrapper> &, crane::RobotCommandWrapper &) = 0;

protected:
  Status status = Status::RUNNING;
  std::shared_ptr<WorldModelWrapper> world_model;
  std::shared_ptr<RobotInfo> robot;
};
}  // namespace crane
#endif  //CRANE_PLANNER_PLUGINS_SKILL_BASE_HPP
