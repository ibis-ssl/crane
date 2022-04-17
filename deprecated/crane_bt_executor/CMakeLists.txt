cmake_minimum_required(VERSION 3.5)
project(crane_bt_executor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# for FastRTPS error
set(CMAKE_PREFIX_PATH /opt/ros/foxy/)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(crane_msgs REQUIRED)
find_package(crane_geometry REQUIRED)
find_package(crane_msg_wrappers REQUIRED)
find_package(crane_world_observer REQUIRED)
find_package(Eigen3 REQUIRED)
add_executable(crane_bt_executor_node
  src/bt_executor_node.cpp
        src/utils/tool.cpp
        src/skill/kick_to_target.cpp
        src/utils/robot_command_builder.cpp
)

ament_target_dependencies(
  crane_bt_executor_node
  "rclcpp"
  "std_msgs"
  "crane_msgs"
  "crane_geometry"
  "crane_msg_wrappers"
  "crane_world_observer"
)

include_directories(include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${EIGEN3_INCLUDE_DIR})

set(behavior_tree_sources
        src/behavior_tree/serialize.cpp)

add_library(behavior_tree ${behavior_tree_sources})

target_link_libraries(crane_bt_executor_node
        behavior_tree)

install(TARGETS crane_bt_executor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
