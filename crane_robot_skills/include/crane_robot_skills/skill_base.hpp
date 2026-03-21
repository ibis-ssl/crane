// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SKILL_BASE_HPP_
#define CRANE_ROBOT_SKILLS__SKILL_BASE_HPP_

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
class StateMachine
{
public:
  struct Transition
  {
    int to;
    std::function<bool()> condition;
  };

  explicit StateMachine(int init_state, std::function<std::string(int)> name_resolver)
  : current_state_(init_state), state_name_resolver_(std::move(name_resolver))
  {
  }

  void addTransition(int from, int to, std::function<bool()> condition)
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
        if (getCurrentStateName() == "ENTRY_POINT") {
          // 再帰的に評価を行うので、無限ループに注意！！！
          update();
        }
        return;
      }
    }
  }

  int getCurrentState() const { return current_state_; }

  std::string getCurrentStateName() const
  {
    return state_name_resolver_ ? state_name_resolver_(current_state_)
                                : std::to_string(current_state_);
  }

private:
  int current_state_;
  std::unordered_map<int, std::vector<Transition>> transitions_;
  std::function<std::string(int)> state_name_resolver_;
};

enum class Status {
  SUCCESS,
  FAILURE,
  RUNNING,
};

inline std::string statusToString(Status status)
{
  switch (status) {
    case Status::SUCCESS:
      return "SUCCESS";
    case Status::FAILURE:
      return "FAILURE";
    case Status::RUNNING:
      return "RUNNING";
    default:
      return "UNKNOWN";
  }
}

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
    // commandはdelegating constructor内で空のnameで生成されるため、ここで正しいnameを設定する
    this->command->name = name;
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
    msg.omega_limit = 10.0;
    msg.current_pose.x = robot()->pose.pos.x();
    msg.current_pose.y = robot()->pose.pos.y();
    msg.current_pose.theta = robot()->pose.theta;
  }

  void finalizeFrame(Status status)
  {
    command->addPlanningFactor(name, statusToString(status));
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

  crane_msgs::msg::RobotCommand getRobotCommand() override { return command->getMsg(); }

  auto & commander() { return command; }
};

class SkillBaseWithState : public SkillInterface
{
public:
  using StateFunctionType = std::function<Status()>;

  static constexpr double OVER_DRIBBLE_DISTANCE_THRESHOLD = 0.5;

  struct OverDribbleInfo
  {
    bool detected = false;
    Point previous_position = Point::Zero();
    double distance = 0.0;

    auto update(const Point & current_position, const Point & ball_position) -> void
    {
      if ((current_position - ball_position).norm() < 0.12) {
        if (!detected) {
          distance = 0.0;
        } else {
          distance += (current_position - previous_position).norm();
        }
        detected = true;
        previous_position = current_position;
      } else {
        detected = false;
        distance = 0.0;
      }
    }
  };

  template <typename... Args>
  explicit SkillBaseWithState(
    int init_state, std::function<std::string(int)> name_resolver, Args &&... args)
  : SkillInterface(std::forward<Args>(args)...),
    state_machine_(init_state, std::move(name_resolver))
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

    over_dribble.update(robot()->pose.pos, world_model()->ball().pos);
    if (over_dribble.distance > OVER_DRIBBLE_DISTANCE_THRESHOLD) {
      command->stopHere();
    }

    onPostUpdate();

    auto state_name = state_machine_.getCurrentStateName();
    command->addPlanningFactor(name, state_name);

    visualizer->text()
      .position(robot()->pose.pos)
      .text(state_name)
      .fontSize(50)
      .fill("white")
      .build();
    visualizer->flush();

    return status;
  }

  crane_msgs::msg::RobotCommand getRobotCommand() override { return command->getMsg(); }

  auto & commander() { return command; }

  void addStateFunction(int state, StateFunctionType function)
  {
    if (state_functions_.find(state) != state_functions_.end()) {
      RCLCPP_WARN(
        rclcpp::get_logger("State: " + name),
        "State function already exists and is overwritten now.");
    }
    state_functions_[state] = function;
  }

  void addTransitions(
    int from, std::vector<std::pair<int, std::function<bool()>>> transition_targets)
  {
    for (const auto & transition_target : transition_targets) {
      state_machine_.addTransition(from, transition_target.first, transition_target.second);
    }
  }

  void addTransition(int from, int to, std::function<bool()> condition)
  {
    state_machine_.addTransition(from, to, condition);
  }

  int getCurrentState() const { return state_machine_.getCurrentState(); }

protected:
  virtual void onPostUpdate() {}

  StateMachine state_machine_;
  std::unordered_map<int, StateFunctionType> state_functions_;
  OverDribbleInfo over_dribble;
};
}  // namespace crane::skills

#endif  // CRANE_ROBOT_SKILLS__SKILL_BASE_HPP_
