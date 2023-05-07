cmake_minimum_required(VERSION 3.5)
project(consai_vision_tracker)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(PkgConfig REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
pkg_check_modules(BFL REQUIRED orocos-bfl)
ament_auto_find_build_dependencies()

link_libraries(${BFL_LIBRARIES})

# Tracker Component
ament_auto_add_library(tracker_component SHARED
  DIRECTORY src
)

ament_auto_add_executable(vision_tracker_node
  src/tracker_node.cpp
)
target_link_libraries(vision_tracker_node tracker_component)

rclcpp_components_register_nodes(tracker_component "consai_vision_tracker::Tracker")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
