// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "crane_gui/gui.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
namespace crane
{
static void glfw_error_callback(int error, const char * description)
{
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

CraneGuiComponent::CraneGuiComponent(const rclcpp::NodeOptions & options):
    rclcpp::Node("gui", options)
    ,clear_color(ImVec4(0.45f, 0.55f, 0.60f, 1.00f))
{
    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(50ms, std::bind(&CraneGuiComponent::loop, this));
    RCLCPP_INFO_STREAM(this->get_logger(), "initialized");
}
void CraneGuiComponent::loop()
{
    // RCLCPP_INFO_STREAM(this->get_logger(), "first loop");
    if(!glfwWindowShouldClose(window))
    {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        auto dock_id = ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());
        ImGui::SetNextWindowDockID(dock_id);
        {
        ImGui::Begin("Hello, world!");
        ImGui::Text(
            "Application average %.3f ms/frame (%.1f FPS)",
            1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
        }

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(
        clear_color.x * clear_color.w, clear_color.y * clear_color.w,
        clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }
    else
    {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();

        glfwDestroyWindow(window);
        glfwTerminate();
        timer_->cancel();
    }
}
int CraneGuiComponent::initializeGL()
{
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;
    glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    window = glfwCreateWindow(1080, 720, "ibis AI", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync
    // Initialize OpenGL loader

    bool err = glewInit() != GLEW_OK;
    if (err) {
        fprintf(stderr, "Failed to initialize OpenGL loader!\n");
        return 1;
    }

    return 0;
}

void CraneGuiComponent::initilizeImGui()
{
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO & io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // Enable Docking

    // Setup Dear ImGui style
    ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
}
}// namespace crane
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crane::CraneGuiComponent)

// static void glfw_error_callback(int error, const char * description)
// {
//   fprintf(stderr, "Glfw Error %d: %s\n", error, description);
// }

// int main(int, char **)
// {
//   glfwSetErrorCallback(glfw_error_callback);
//   if (!glfwInit()) {
//     return 1;
//   }

//   const char * glsl_version = "#version 150";
//   glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
//   glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
//   glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//   glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

//   GLFWwindow * window = glfwCreateWindow(1280, 720, "gui", NULL, NULL);
//   if (window == NULL) {
//     return 1;
//   }
//   glfwMakeContextCurrent(window);
//   glfwSwapInterval(1); // enable vsync

//   bool err = glewInit() != GLEW_OK;
//   if (err) {
//     fprintf(stderr, "Failed to initialize OpenGL loader!\n");
//     return 1;
//   }

//   // Setup Dear ImGui context
//   IMGUI_CHECKVERSION();
//   ImGui::CreateContext();
//   ImGuiIO & io = ImGui::GetIO();
//   io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
//   io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // Enable Docking

//   // Setup Dear ImGui style
//   ImGui::StyleColorsClassic();

//   // Setup Platform/Renderer backends
//   ImGui_ImplGlfw_InitForOpenGL(window, true);
//   ImGui_ImplOpenGL3_Init(glsl_version);

//   bool show_demo_window = true;
//   ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

//   while (!glfwWindowShouldClose(window)) {
//     // Poll and handle events (inputs, window resize, etc.)
//     // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
//     // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
//     // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
//     // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
//     glfwPollEvents();

//     ImGui_ImplOpenGL3_NewFrame();
//     ImGui_ImplGlfw_NewFrame();
//     ImGui::NewFrame();

//     auto dock_id = ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());
//     ImGui::SetNextWindowDockID(dock_id);
//     {
//       ImGui::Begin("Hello, world!");
//       ImGui::Text(
//         "Application average %.3f ms/frame (%.1f FPS)",
//         1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
//       ImGui::End();
//     }

//     ImGui::Render();
//     int display_w, display_h;
//     glfwGetFramebufferSize(window, &display_w, &display_h);
//     glViewport(0, 0, display_w, display_h);
//     glClearColor(
//       clear_color.x * clear_color.w, clear_color.y * clear_color.w,
//       clear_color.z * clear_color.w, clear_color.w);
//     glClear(GL_COLOR_BUFFER_BIT);
//     ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

//     glfwSwapBuffers(window);
//   }

//   ImGui_ImplOpenGL3_Shutdown();
//   ImGui_ImplGlfw_Shutdown();
//   ImGui::DestroyContext();

//   glfwDestroyWindow(window);
//   glfwTerminate();

//   return 0;
// }
