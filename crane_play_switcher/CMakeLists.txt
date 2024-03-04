cmake_minimum_required(VERSION 3.5)
project(crane_play_switcher)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(play_switcher SHARED src/play_switcher.cpp)

rclcpp_components_register_nodes(play_switcher "crane::PlaySwitcher")

ament_auto_add_executable(play_switcher_node
  src/play_switcher_node.cpp
)
add_backward(play_switcher_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
