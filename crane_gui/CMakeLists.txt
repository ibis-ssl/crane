cmake_minimum_required(VERSION 3.11)
project(crane_gui)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# OpenGL_GL_PREFERENCE has not been set to "GLVND" or "LEGACY", so for
# compatibility with CMake 3.10 and below the legacy GL library will be used.
set(OpenGL_GL_PREFERENCE GLVND)

include(FetchContent)
fetchcontent_populate(imgui
        URL https://github.com/ocornut/imgui/archive/docking.zip
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/imgui
)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()

include_directories(${CMAKE_CURRENT_BINARY_DIR}/imgui)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/imgui/backends)

set(IMGUI_SOURCES
  ${CMAKE_CURRENT_BINARY_DIR}/imgui/imgui.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/imgui/imgui_demo.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/imgui/imgui_draw.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/imgui/imgui_widgets.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/imgui/imgui_tables.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/imgui/backends/imgui_impl_opengl3.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/imgui/backends/imgui_impl_glfw.cpp
)

ament_auto_add_library(${PROJECT_NAME}_component SHARED src/crane_gui.cpp ${IMGUI_SOURCES})
target_link_libraries(${PROJECT_NAME}_component GL glfw GLEW)

rclcpp_components_register_nodes(${PROJECT_NAME}_component "crane::GuiComponent")

ament_auto_add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_component)
add_backward(${PROJECT_NAME}_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
