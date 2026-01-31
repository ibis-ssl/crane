// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SKILL_BASE_HPP_
#define CRANE_ROBOT_SKILLS__SKILL_BASE_HPP_

#include <../magic_enum.hpp>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <format>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace crane::skills
{
template <typename StatesType>
class StateMachine
{
public:
  struct Transition
  {
    StatesType to;
    std::function<bool()> condition;
  };

  explicit StateMachine(StatesType init_state) : current_state_(init_state) {}

  void addTransition(
    const StatesType & from, const StatesType & to, std::function<bool()> condition)
  {
    transitions_[from].push_back({to, std::move(condition)});
  }

  void update()
  {
    auto it = transitions_.find(current_state_);
    if (it == transitions_.end()) return;

    for (const auto & trans : it->second) {
      if (trans.condition()) {
        current_state_ = trans.to;
        // 遷移先が"ENTRY_POINT"の場合、すぐさま次の遷移の評価を行う。
        // state functionの実行は行われない
        if (magic_enum::enum_name(current_state_) == "ENTRY_POINT") {
          // 再帰的に評価を行うので、無限ループに注意！！！
          update();
        }
        return;
      }
    }
  }

  StatesType getCurrentState() const { return current_state_; }

private:
  StatesType current_state_;
  std::unordered_map<StatesType, std::vector<Transition>> transitions_;
};

enum class Status {
  SUCCESS,
  FAILURE,
  RUNNING,
};

using ParameterType = std::variant<double, bool, int, std::string, Point>;

class SkillInterface
{
public:
  SkillInterface() = delete;

  template <typename... Args>
  explicit SkillInterface(const std::string & name, Args &&... args)
  : SkillInterface(std::forward<Args>(args)...)
  {
    this->name = name;
    if (visualizer->layer == "skill/") {
      visualizer->layer = "skill/" + name;
    }
  }

  SkillInterface(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
  : command(std::make_shared<PositionCommandWrapper>(name, id, wm)),
    visualizer(std::make_unique<crane::VisualizerMessageBuilder>("skill/" + name))
  {
    // スキル用ビジュアライザのデフォルトduration: 0.5秒
    visualizer->withDuration(0.5);
  }

  explicit SkillInterface(std::shared_ptr<PositionCommandWrapper> & command)
  : command(command),
    visualizer(std::make_unique<crane::VisualizerMessageBuilder>("skill/" + command->name))
  {
    // スキル用ビジュアライザのデフォルトduration: 0.5秒
    visualizer->withDuration(0.5);
  }

  virtual ~SkillInterface() { visualizer->clearBuffer(); }

  std::string name;

  virtual Status run(
    std::optional<std::unordered_map<std::string, ParameterType>> parameters_opt =
      std::nullopt) = 0;

  void setParameter(const std::string & key, bool value) { parameters[key] = value; }

  void setParameter(const std::string & key, int value) { parameters[key] = value; }

  void setParameter(const std::string & key, double value) { parameters[key] = value; }

  void setParameter(const std::string & key, const std::string & value) { parameters[key] = value; }

  void setParameter(const std::string & key, const Point & value) { parameters[key] = value; }

  virtual crane_msgs::msg::PositionCommand getRobotCommand() = 0;

  template <class T>
  auto getParameter(const std::string & key) const
  {
    try {
      return std::get<T>(parameters.at(key));
    } catch (const std::out_of_range & e) {
      throw std::out_of_range("Parameter " + key + " is not found");
    }
  }

  uint8_t getID() const { return command->getRobot()->id; }

  void clearVisualizer() { visualizer->clearBuffer(); }

protected:
  std::shared_ptr<PositionCommandWrapper> command;

  std::shared_ptr<WorldModelWrapper> world_model() const { return command->getWorldModel(); }

  std::shared_ptr<RobotInfo> robot() const { return command->getRobot(); }

  std::unordered_map<std::string, ParameterType> parameters;

  crane::VisualizerMessageBuilder::SharedPtr visualizer;

  void prepareFrame()
  {
    command->clearMaxVelocityFactors();
    command->clearMaxAccelerationFactors();
    command->clearPlanningFactors();

    auto & msg = command->getEditableMsg();
    msg.current_pose.x = robot()->pose.pos.x();
    msg.current_pose.y = robot()->pose.pos.y();
    msg.current_pose.theta = robot()->pose.theta;
  }

  void finalizeFrame(Status status)
  {
    command->addPlanningFactor(name, std::string(magic_enum::enum_name(status)));
    visualizer->flush();
  }
};

class SkillBase : public SkillInterface
{
public:
  template <typename... Args>
  explicit SkillBase(Args &&... args) : SkillInterface(std::forward<Args>(args)...)
  {
  }

  Status run(
    std::optional<std::unordered_map<std::string, ParameterType>> parameters_opt =
      std::nullopt) override
  {
    if (parameters_opt) {
      parameters = parameters_opt.value();
    }

    prepareFrame();
    auto status = update();
    finalizeFrame(status);
    return status;
  }

  virtual Status update() = 0;

  crane_msgs::msg::PositionCommand getRobotCommand() override { return command->getMsg(); }

  auto & commander() { return command; }
};

template <typename StatesType>
class SkillBaseWithState : public SkillInterface
{
public:
  using StateFunctionType = std::function<Status()>;

  template <typename... Args>
  explicit SkillBaseWithState(Args &&... args)
  : SkillInterface(std::forward<Args>(args)...), state_machine_(static_cast<StatesType>(0))
  {
  }

  Status run(
    std::optional<std::unordered_map<std::string, ParameterType>> parameters_opt =
      std::nullopt) override
  {
    if (parameters_opt) {
      parameters = parameters_opt.value();
    }

    state_machine_.update();
    prepareFrame();

    auto current_state = state_machine_.getCurrentState();
    auto it = state_functions_.find(current_state);
    Status status = (it != state_functions_.end()) ? it->second() : Status::RUNNING;

    onPostUpdate();

    auto state_name = magic_enum::enum_name(current_state);
    command->addPlanningFactor(name, std::string(state_name));

    visualizer->text()
      .position(robot()->pose.pos)
      .text(std::string(state_name))
      .fontSize(50)
      .fill("white")
      .build();
    visualizer->flush();

    return status;
  }

  crane_msgs::msg::PositionCommand getRobotCommand() override { return command->getMsg(); }

  auto & commander() { return command; }

  void addStateFunction(const StatesType & state, StateFunctionType function)
  {
    if (state_functions_.find(state) != state_functions_.end()) {
      RCLCPP_WARN(
        rclcpp::get_logger("State: " + name),
        "State function already exists and is overwritten now.");
    }
    state_functions_[state] = function;
  }

  void addTransitions(
    const StatesType & from,
    std::vector<std::pair<StatesType, std::function<bool()>>> transition_targets)
  {
    for (const auto & transition_target : transition_targets) {
      state_machine_.addTransition(from, transition_target.first, transition_target.second);
    }
  }

  void addTransition(const StatesType from, const StatesType to, std::function<bool()> condition)
  {
    state_machine_.addTransition(from, to, condition);
  }

  StatesType getCurrentState() const { return state_machine_.getCurrentState(); }

protected:
  virtual void onPostUpdate() {}

  StateMachine<StatesType> state_machine_;
  std::unordered_map<StatesType, StateFunctionType> state_functions_;
};
}  // namespace crane::skills

#endif  // CRANE_ROBOT_SKILLS__SKILL_BASE_HPP_
