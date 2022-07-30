cmake_minimum_required(VERSION 3.5)
project(crane_receive_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()
#find_package(std_msgs REQUIRED)
#find_package(crane_msgs REQUIRED)
#find_package(crane_msg_wrappers REQUIRED)

ament_auto_add_library(${PROJECT_NAME}_component SHARED
        src/crane_receive_planner.cpp include/crane_receive_planner/receive_planner.hpp)
rclcpp_components_register_nodes(${PROJECT_NAME}_component "crane::ReceivePlannerComponent")

ament_auto_add_executable(${PROJECT_NAME}_node
        src/${PROJECT_NAME}_node.cpp
        )

# for matplotlib
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)

include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

include_directories(include ${CMAKE_CURRENT_BINARY_DIR})

#ament_target_dependencies(${PROJECT_NAME}_node
#  "rclcpp"
#  "std_msgs"
#  "crane_msgs"
#  "crane_msg_wrappers"
#)

ament_auto_add_executable(test_node src/test.cpp)
ament_target_dependencies(test_node "Python3")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
