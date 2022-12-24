cmake_minimum_required(VERSION 3.5)
project(consai_vision_tracker)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rclcpp_components REQUIRED)

include_directories(
  include
  ${BFL_INCLUDE_DIRS}
)

# Tracker Component
ament_auto_add_library(tracker_component SHARED
  src/tracker_component.cpp
  src/ball_tracker.cpp
  src/robot_tracker.cpp
)
#target_compile_definitions(tracker_component
#  PRIVATE "CONSAI_VISION_TRACKER_BUILDING_DLL")

rclcpp_components_register_nodes(tracker_component "consai_vision_tracker::Tracker")

# Exports
#ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
