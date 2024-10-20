// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/teleop.hpp>

namespace crane::skills
{
Teleop::Teleop(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("Teleop", base), Node("teleop_skill")
{
  setParameter("rotation_deg", 0.);
  setParameter("use_local_coordinate", false);
  std::cout << "Teleop skill created" << std::endl;
  joystick_subscription = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, [this](const sensor_msgs::msg::Joy & msg) {
      std::cout << "joy message received" << std::endl;
      last_joy_msg = msg;
    });
  std::cout << "joy subscriber created" << std::endl;
}

Status Teleop::update(const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  rclcpp::spin_some(this->get_node_base_interface());
  if (last_joy_msg.buttons.empty()) {
    std::cout << "no joy message" << std::endl;
    return Status::RUNNING;
  }

  const int BUTTON_POWER_ENABLE = 9;

  const int AXIS_VEL_SURGE = 1;
  const int AXIS_VEL_SWAY = 0;
  const int AXIS_VEL_ANGULAR_R = 2;
  const int AXIS_VEL_ANGULAR_L = 5;

  const int BUTTON_KICK_TOGGLE = 4;
  const int BUTTON_KICK_STRAIGHT = 13;
  const int BUTTON_KICK_CHIP = 14;
  const int BUTTON_ADJUST_KICK = 0;

  const int BUTTON_ADJUST = 10;
  const int BUTTON_ADJUST_UP = 11;
  const int BUTTON_ADJUST_DOWN = 12;

  const int BUTTON_DRIBBLE_TOGGLE = 6;
  const int BUTTON_ADJUST_DRIBBLE = 1;

  const double MAX_VEL_SURGE = 1.0;
  const double MAX_VEL_SWAY = 1.0;
  const double MAX_VEL_ANGULAR = M_PI * 0.1;

  static bool is_kick_mode_straight = true;
  static bool is_kick_enable = false;
  static bool is_dribble_enable = false;

  if (last_joy_msg.buttons[BUTTON_KICK_CHIP]) {
    std::cout << "chip mode" << std::endl;
    is_kick_mode_straight = false;
  }
  if (last_joy_msg.buttons[BUTTON_KICK_STRAIGHT]) {
    std::cout << "straight mode" << std::endl;
    is_kick_mode_straight = true;
  }

  auto update_mode = [&](bool & mode_variable, const int button, bool & is_pushed) {
    // trigger button up
    if (last_joy_msg.buttons[button]) {
      if (!is_pushed) {
        std::cout << "toggle mode!" << std::endl;
        mode_variable = not mode_variable;
      }
      is_pushed = true;
    } else {
      is_pushed = false;
    }
  };

  static bool is_pushed_kick = false;
  static bool is_pushed_dribble = false;

  update_mode(is_kick_enable, BUTTON_KICK_TOGGLE, is_pushed_kick);
  update_mode(is_dribble_enable, BUTTON_DRIBBLE_TOGGLE, is_pushed_dribble);

  auto adjust_value = [](double & value, const double step) {
    value += step;
    value = std::clamp(value, 0.0, 1.0);
  };

  if (last_joy_msg.buttons[BUTTON_ADJUST]) {
    static bool is_pushed = false;
    if (last_joy_msg.buttons[BUTTON_ADJUST_UP]) {
      // trigger button up
      if (!is_pushed) {
        if (last_joy_msg.buttons[BUTTON_ADJUST_KICK]) {
          adjust_value(kick_power, 0.1);
          std::cout << "kick up: " << kick_power << std::endl;
        }

        if (last_joy_msg.buttons[BUTTON_ADJUST_DRIBBLE]) {
          adjust_value(dribble_power, 0.1);
          std::cout << "dribble up:" << dribble_power << std::endl;
        }
      }
      is_pushed = true;
    } else if (last_joy_msg.buttons[BUTTON_ADJUST_DOWN]) {
      // trigger button up
      if (!is_pushed) {
        if (last_joy_msg.buttons[BUTTON_ADJUST_KICK]) {
          adjust_value(kick_power, -0.1);
          std::cout << "kick down: " << kick_power << std::endl;
        }
        if (last_joy_msg.buttons[BUTTON_ADJUST_DRIBBLE]) {
          adjust_value(dribble_power, -0.1);
          std::cout << "dribble down: " << dribble_power << std::endl;
        }
      }
      is_pushed = true;
    } else {
      is_pushed = false;
    }
  }

  Point target = [&]() -> Point {
    using boost::math::constants::degree;
    double rotation_angle = getParameter<double>("rotation_deg") * degree<double>();
    if (getParameter<bool>("use_local_coordinate")) {
      rotation_angle += robot()->pose.theta;
    }
    Eigen::Rotation2Dd rotation(rotation_angle);
    return robot()->pose.pos +
           rotation.toRotationMatrix() * Point{
                                           last_joy_msg.axes[AXIS_VEL_SURGE] * MAX_VEL_SURGE,
                                           last_joy_msg.axes[AXIS_VEL_SWAY] * MAX_VEL_SWAY};
  }();

  command.setTargetPosition(target);

  double angular =
    (1.0 - last_joy_msg.axes[AXIS_VEL_ANGULAR_R]) - (1.0 - last_joy_msg.axes[AXIS_VEL_ANGULAR_L]);

  theta += angular;
  command.setTargetTheta(normalizeAngle(theta));

  if (is_kick_enable) {
    if (is_kick_mode_straight) {
      command.kickStraight(kick_power);
    } else {
      command.kickWithChip(kick_power);
    }
  } else {
    command.kickStraight(0.0);
  }

  if (is_dribble_enable) {
    command.dribble(dribble_power);
  } else {
    command.dribble(0.0);
  }

  return Status::RUNNING;
}
}  // namespace crane::skills
