# Python関連の挙動が3.12を境に変わっているらしい
# 参考: https://qiita.com/mitsu48/items/fc4e1779be0cfe14091d
cmake_minimum_required(VERSION 3.12)
project(crane_planner_plugins)

# Default to C++20
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif ()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()

# for matplotlib
find_package(Python COMPONENTS Interpreter Development NumPy REQUIRED)
#find_package(PythonLibs REQUIRED)
#include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})
include_directories(${Python_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src include
)

target_precompile_headers(${PROJECT_NAME}
  PUBLIC
    "<crane_basics/boost_geometry.hpp>"
    "<crane_basics/interval.hpp>"
    "<crane_basics/eigen_adapter.hpp>"
    "<crane_msg_wrappers/robot_command_wrapper.hpp>"
    "<crane_msg_wrappers/world_model_wrapper.hpp>"
    "<crane_msgs/msg/world_model.hpp>"
    "<crane_planner_base/planner_base.hpp>"
    "<crane_robot_skills/skill_base.hpp>"
    "<rclcpp/rclcpp.hpp>"
    "<Eigen/Geometry>"
    "<crane_msgs/srv/robot_select.hpp>"
    "<crane_robot_skills/kick.hpp>"
)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_auto_package()
