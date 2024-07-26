// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GUI__GUI_HPP_
#define CRANE_GUI__GUI_HPP_

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <rclcpp/rclcpp.hpp>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "visibility_control.h"

namespace crane
{

class CraneGuiComponent : public rclcpp::Node
{
public:  // function
  COMPOSITION_PUBLIC
  explicit CraneGuiComponent(const rclcpp::NodeOptions & options);
  int initializeGL();
  void initilizeImGui();

private:  // function
  void loop();

private:  // varialble
  rclcpp::TimerBase::SharedPtr timer_;
  const char * glsl_version;
  GLFWwindow * window;
  ImVec4 clear_color;
};

}  // namespace crane
#endif  // CRANE_GUI__GUI_HPP_
