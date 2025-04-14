// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SKILL_BASE_HPP_
#define CRANE_ROBOT_SKILLS__SKILL_BASE_HPP_

#include <../magic_enum.hpp>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <format>
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
    if (auto it = std::ranges::find_if(
          transitions,
          [this](const Transition & transition) {
            return transition.from == current_state && transition.condition();
          });
        it != transitions.end()) {
      // 遷移先が"ENTRY_POINT"の場合、すぐさま次の遷移の評価を行う。state functionの実行は行われない
      if (current_state = it->to; magic_enum::enum_name(current_state) == "ENTRY_POINT") {
        // 再帰的に評価を行うので、無限ループに注意！！！
        update();
      }
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
      [&type_string](const std::string &) { type_string = "string"; },
      [&type_string](const Point &) { type_string = "Point"; },
      [&type_string](const std::optional<Point> &) { type_string = "op<Point>"; }},
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
      [&value_string](const std::string & e) { value_string = e; },
      [&value_string](const Point & e) {
        value_string = std::format("({}, {})", std::to_string(e.x()), std::to_string(e.y()));
      },
      [&value_string](const std::optional<Point> & e) {
        if (e) {
          value_string = std::format("({}, {})", std::to_string(e->x()), std::to_string(e->y()));
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
  SkillInterface() = delete;

  template <typename... Args>
  explicit SkillInterface(const std::string & name, Args &&... args)
  : SkillInterface(std::forward<Args>(args)...)
  {
    this->name = name;
  }

  SkillInterface(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
  : command(name, id, wm),
    visualizer(std::make_unique<crane::VisualizerMessageBuilder>("skill/" + name)),
    target_theta_context(getContextReference<double>("target_theta")),
    dribble_power_context(getContextReference<double>("dribble_power")),
    kick_power_context(getContextReference<double>("kick_power")),
    chip_enable_context(getContextReference<bool>("chip_enable")),
    stop_flag_context(getContextReference<bool>("stop_flag"))
  {
  }

  // SkillInterface(const std::string & name, RobotCommandWrapper & command)
  // : name(name),
  //   command(command),
  //   visualizer(std::make_unique<crane::VisualizerMessageBuilder>("skill/" + name)),
  //   target_theta_context(getContextReference<double>("target_theta")),
  //   dribble_power_context(getContextReference<double>("dribble_power")),
  //   kick_power_context(getContextReference<double>("kick_power")),
  //   chip_enable_context(getContextReference<bool>("chip_enable")),
  //   stop_flag_context(getContextReference<bool>("stop_flag"))
  // {
  // }

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
    for (const auto & [name, parameter] : parameters) {
      os << name << ": ";
      std::visit(
        overloaded{
          [&os](double e) { os << "double, " << e << std::endl; },
          [&os](int e) { os << "int, " << e << std::endl; },
          [&os](const std::string & e) { os << "string, " << e << std::endl; },
          [&os](bool e) { os << "bool, " << e << std::endl; },
          [&os](Point e) { os << "Point, " << e.x() << ", " << e.y() << std::endl; }},
        parameter);
    }
  }

  virtual void print(std::ostream &) const {}

  // operator<< がAのprivateメンバにアクセスできるようにfriend宣言
  friend std::ostream & operator<<(std::ostream & os, const SkillInterface & skill);

  uint8_t getID() const { return command.getRobot()->id; }

  void setPreUpdateFunction(std::function<void()> f) { pre_update = f; }

  void setPostUpdateFunction(std::function<void()> f) { post_update = f; }

protected:
  RobotCommandWrapper command;

  std::shared_ptr<WorldModelWrapper> world_model() const { return command.getWorldModel(); }

  std::shared_ptr<RobotInfo> robot() const { return command.getRobot(); }

  std::unordered_map<std::string, ParameterType> parameters;

  std::unordered_map<std::string, ContextType> contexts;

  crane::VisualizerMessageBuilder::SharedPtr visualizer;

  Status status = Status::RUNNING;

  void updateDefaultContexts()
  {
    target_theta_context = command.getMsg().target_theta;
    kick_power_context = command.getMsg().kick_power;
    dribble_power_context = command.getMsg().dribble_power;
    chip_enable_context = command.getMsg().chip_enable;
    stop_flag_context = command.getMsg().stop_flag;
  }

  std::function<void()> pre_update = nullptr;

  std::function<void()> post_update = nullptr;

private:
  double & target_theta_context;

  double & dribble_power_context;

  double & kick_power_context;

  bool & chip_enable_context;

  bool & stop_flag_context;
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

    command.getEditableMsg().current_pose.x = command.getRobot()->pose.pos.x();
    command.getEditableMsg().current_pose.y = command.getRobot()->pose.pos.y();
    command.getEditableMsg().current_pose.theta = command.getRobot()->pose.theta;

    if (pre_update) {
      pre_update();
    }
    auto ret = update();
    if (post_update) {
      post_update();
    }
    updateDefaultContexts();
    command.addStateFactor(name, std::string(magic_enum::enum_name(ret)));
    visualizer->flush();
    return ret;
  }

  virtual Status update() = 0;

  crane_msgs::msg::RobotCommand getRobotCommand() override { return command.getMsg(); }

  auto & commander() { return command; }

protected:
  // operator<< がAのprivateメンバにアクセスできるようにfriend宣言
  friend std::ostream & operator<<(std::ostream & os, const SkillBase & skill_base);
};

template <typename StatesType>
class SkillBaseWithState : public SkillInterface
{
public:
  using StateFunctionType = std::function<Status()>;

  template <typename... Args>
  explicit SkillBaseWithState(Args &&... args)
  : SkillInterface(std::forward<Args>(args)...),
    state_machine(static_cast<StatesType>(0)),
    state_string(getContextReference<std::string>("state"))
  {
  }

  Status run(
    std::optional<std::unordered_map<std::string, ParameterType>> parameters_opt =
      std::nullopt) override
  {
    if (parameters_opt) {
      parameters = parameters_opt.value();
    }
    state_machine.update();
    state_string = magic_enum::enum_name(state_machine.getCurrentState());

    command.getEditableMsg().current_pose.x = command.getRobot()->pose.pos.x();
    command.getEditableMsg().current_pose.y = command.getRobot()->pose.pos.y();
    command.getEditableMsg().current_pose.theta = command.getRobot()->pose.theta;

    if (pre_update) {
      pre_update();
    }
    auto ret = state_functions[state_machine.getCurrentState()]();
    if (post_update) {
      post_update();
    }
    updateDefaultContexts();
    command.addStateFactor(name, state_string);

    visualizer->text()
      .position(robot()->pose.pos)
      .text(state_string)
      .fontSize(50)
      .fill("white")
      .build();
    visualizer->flush();
    return ret;
  }

  crane_msgs::msg::RobotCommand getRobotCommand() override { return command.getMsg(); }

  auto & commander() { return command; }

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
  template <typename T>
  friend std::ostream & operator<<(std::ostream & os, const SkillBaseWithState<T> & skill_base);
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
  std::ostream & os, const crane::skills::SkillBaseWithState<StatesType> & skill)
{
  skill.print(os);
  return os;
}

template <typename StatesType>
inline std::ostream & operator<<(
  std::ostream & os, const std::shared_ptr<crane::skills::SkillBaseWithState<StatesType>> & skill)
{
  skill->print(os);
  return os;
}

#endif  // CRANE_ROBOT_SKILLS__SKILL_BASE_HPP_
