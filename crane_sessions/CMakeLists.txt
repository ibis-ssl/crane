# Python関連の挙動が3.12を境に変わっているらしい
# 参考: https://qiita.com/mitsu48/items/fc4e1779be0cfe14091d
cmake_minimum_required(VERSION 3.12)
project(crane_sessions)

# Default to C++20
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(NOT CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-g)
  endif()
endif ()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml-cpp REQUIRED)
ament_auto_find_build_dependencies()

# for matplotlib
find_package(Python COMPONENTS Interpreter Development NumPy REQUIRED)
#find_package(PythonLibs REQUIRED)
#include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})
include_directories(${Python_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src include
)

target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

# Enable Unity Build for faster compilation
option(CRANE_UNITY_BUILD "Enable unity build" ON)
if(CRANE_UNITY_BUILD)
  set_target_properties(${PROJECT_NAME} PROPERTIES UNITY_BUILD ON)
  set_target_properties(${PROJECT_NAME} PROPERTIES UNITY_BUILD_BATCH_SIZE 6)
endif()

target_precompile_headers(${PROJECT_NAME}
  PUBLIC
    "<crane_geometry/boost_geometry.hpp>"
    "<crane_geometry/interval.hpp>"
    "<crane_msg_wrappers/robot_command_wrapper.hpp>"
    "<crane_msg_wrappers/world_model_wrapper.hpp>"
    "<crane_msgs/msg/world_model.hpp>"
    "<crane_sessions/session_base.hpp>"
    "<crane_robot_skills/skill_base.hpp>"
    "<rclcpp/rclcpp.hpp>"
    "<Eigen/Geometry>"
    "<crane_robot_skills/kick.hpp>"
    "<yaml-cpp/yaml.h>"
)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_auto_package(INSTALL_TO_SHARE config USE_SCOPED_HEADER_INSTALL_DIR)
