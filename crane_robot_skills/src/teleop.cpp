// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/teleop.hpp>

namespace crane::skills
{
Status Teleop::update()
{
  rclcpp::spin_some(this->get_node_base_interface());
  if (last_joy_msg.buttons.empty()) {
    RCLCPP_INFO(get_logger(), "no joy message");
    return Status::RUNNING;
  }

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

  static bool is_kick_mode_straight = true;
  static bool is_kick_enable = false;
  static bool is_dribble_enable = false;

  if (last_joy_msg.buttons[BUTTON_KICK_CHIP]) {
    RCLCPP_INFO(get_logger(), "chip mode");
    is_kick_mode_straight = false;
  }
  if (last_joy_msg.buttons[BUTTON_KICK_STRAIGHT]) {
    RCLCPP_INFO(get_logger(), "straight mode");
    is_kick_mode_straight = true;
  }

  auto update_mode = [&](bool & mode_variable, const int button, bool & is_pushed) {
    // trigger button up
    if (last_joy_msg.buttons[button]) {
      if (!is_pushed) {
        RCLCPP_INFO(get_logger(), "toggle mode!");
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

  auto adjust_value = [&](double & value, const double step) {
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
          RCLCPP_INFO(get_logger(), "kick up: %f", kick_power);
        }

        if (last_joy_msg.buttons[BUTTON_ADJUST_DRIBBLE]) {
          adjust_value(dribble_power, 0.1);
          RCLCPP_INFO(get_logger(), "dribble up: %f", dribble_power);
        }
      }
      is_pushed = true;
    } else if (last_joy_msg.buttons[BUTTON_ADJUST_DOWN]) {
      // trigger button up
      if (!is_pushed) {
        if (last_joy_msg.buttons[BUTTON_ADJUST_KICK]) {
          adjust_value(kick_power, -0.1);
          RCLCPP_INFO(get_logger(), "kick down: %f", kick_power);
        }
        if (last_joy_msg.buttons[BUTTON_ADJUST_DRIBBLE]) {
          adjust_value(dribble_power, -0.1);
          RCLCPP_INFO(get_logger(), "dribble down: %f", dribble_power);
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
    Eigen::Rotation2Dd rotation(rotation_angle);  // NEW
    return robot()->pose.pos + rotation * Point(
                                            // NEW
                                            last_joy_msg.axes[AXIS_VEL_SURGE] * MAX_VEL_SURGE,
                                            last_joy_msg.axes[AXIS_VEL_SWAY] * MAX_VEL_SWAY);
  }();

  command->setTargetPosition(target);

  double angular =
    (1.0 - last_joy_msg.axes[AXIS_VEL_ANGULAR_R]) - (1.0 - last_joy_msg.axes[AXIS_VEL_ANGULAR_L]);

  theta += angular;
  command->setTargetTheta(normalizeAngle(theta));

  if (is_kick_enable) {
    if (is_kick_mode_straight) {
      command->kickStraight(kick_power);
    } else {
      command->kickWithChip(kick_power);
    }
  } else {
    command->kickStraight(0.0);
  }

  if (is_dribble_enable) {
    command->dribble(dribble_power);
  } else {
    command->dribble(0.0);
  }

  return Status::RUNNING;
}
}  // namespace crane::skills
