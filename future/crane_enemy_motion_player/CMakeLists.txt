cmake_minimum_required(VERSION 3.5)
project(crane_enemy_motion_player)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#include_directories(include ${CMAKE_CURRENT_BINARY_DIR})
ament_auto_add_executable(enemy_motion_player_node
  src/enemy_motion_player_node.cpp
)

#ament_target_dependencies(enemy_motion_player_node
#  "rclcpp"
#  "std_msgs"
#  "crane_msgs"
#)

#install(TARGETS enemy_motion_player_node
#  EXPORT export_${PROJECT_NAME}
#  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
