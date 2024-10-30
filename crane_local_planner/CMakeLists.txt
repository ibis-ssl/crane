cmake_minimum_required(VERSION 3.5)
project(crane_local_planner)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(backward_ros REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_component SHARED DIRECTORY src)

target_precompile_headers(${PROJECT_NAME}_component
  PUBLIC
    "<crane_basics/boost_geometry.hpp>"
    "<crane_basics/eigen_adapter.hpp>"
    "<crane_basics/pid_controller.hpp>"
    "<crane_msgs/msg/robot_commands.hpp>"
    "<crane_msg_wrappers/world_model_wrapper.hpp>"
    "<crane_msg_wrappers/consai_visualizer_wrapper.hpp>"
    "<rclcpp/rclcpp.hpp>"
)

rclcpp_components_register_nodes(${PROJECT_NAME}_component "crane::LocalPlannerComponent")

ament_auto_add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)

include(/opt/ros/humble/share/backward_ros/cmake/BackwardConfig.cmake)
add_backward(${PROJECT_NAME}_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
